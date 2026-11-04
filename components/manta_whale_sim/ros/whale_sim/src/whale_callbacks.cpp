#include "whale_sim/whale_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <exception>
#include <limits>
#include <stdexcept>

//----------------------------------------------------
// Handle functions
//----------------------------------------------------

/** Validate and execute the received whale simulation batch, then publish its result. */
void WhaleNode::handle_sim_step() {
    const auto &step = sim_state_.received_sim_step;
    auto &result = sim_state_.step_result;
    result = whale_interfaces::msg::WhaleStepResult{};
    result.completed_cycle = step.current_cycle;
    result.succeeded_b = false;

    try {
        if (step.step_count == 0 || !std::isfinite(step.sim_time_s) || !std::isfinite(step.timestep_s) || step.timestep_s <= 0.0 || std::abs(step.sim_time_s - sim_state_.sim_time_s) > 1.0e-6) {
            throw std::runtime_error("Invalid or out-of-order whale simulation step.");
        }

        // A batch may contain many physics steps. Stop it at the first
        // scheduled call so the vehicles can supply the matching state.
        find_next_call();
        std::uint32_t completed_steps = step.step_count;
        const double requested_end_s = step.sim_time_s + static_cast<double>(completed_steps) * step.timestep_s;
        if (sim_state_.next_whale_call_available_b &&
            sim_state_.next_whale_call.sim_time_s <= requested_end_s + 1.0e-9) {
            const double steps_to_call = std::ceil((sim_state_.next_whale_call.sim_time_s - step.sim_time_s) / step.timestep_s);
            completed_steps = static_cast<std::uint32_t>(std::min<double>(completed_steps, std::max(1.0, steps_to_call)));
        }

        sim_state_.pending_sim_step = {step.current_cycle, step.sim_time_s, step.timestep_s, completed_steps};
        update_whales();

        result.completed_cycle = sim_state_.completed_cycle;
        result.completed_step_count = completed_steps;
        if (sim_state_.next_whale_call_available_b &&
            sim_state_.next_whale_call.sim_time_s <= sim_state_.sim_time_s + 1.0e-9) {
            publish_pending_call();
            result.whale_call_published_b = true;
            result.whale_call = sim_state_.next_whale_call;
        }
        result.succeeded_b = true;
    } catch (const std::exception &error) {
        result.detail = error.what();
        RCLCPP_ERROR(get_logger(), "%s", result.detail.c_str());
    }
    step_result_pub_->publish(result);
}

/**
 * Find the first scheduled whale call after the current simulation time.
 *
 * Calls at or before the current whale simulation time have already been processed
 * and are discarded. Store the earliest remaining call and its position in state.
 */
void WhaleNode::find_next_call() {
    // Clear the previous result before searching for the next call.
    sim_state_.next_whale_call = whale_interfaces::msg::WhaleCall{};
    sim_state_.next_whale_call_available_b = false;

    // No whale has been selected yet.
    std::size_t next_whale_index = sim_state_.whales.size();
    double next_time_s = std::numeric_limits<double>::infinity();

    // Search every whale for its earliest unprocessed call.
    for (std::size_t whale_index = 0; whale_index < sim_state_.whales.size(); ++whale_index) {
        Whale &whale = sim_state_.whales[whale_index];
        // Discard call times already passed by the whale simulation clock.
        while (whale.next_call_index < whale.whale_call_times_s.size() && whale.whale_call_times_s[whale.next_call_index] <= sim_state_.sim_time_s) {
            ++whale.next_call_index;
        }

        // Skip whales that have no scheduled calls remaining.
        if (whale.next_call_index >= whale.whale_call_times_s.size()) continue;
        const double candidate_time_s = whale.whale_call_times_s[whale.next_call_index];

        // Retain the earliest call found across all whales.
        if (candidate_time_s < next_time_s) {
            next_time_s = candidate_time_s;
            next_whale_index = whale_index;
        }
    }

    // Report that no whale calls remain in the simulation.
    if (next_whale_index == sim_state_.whales.size()) return;

    // Identify the next call and its scheduled simulation time.
    sim_state_.next_whale_call_available_b = true;
    sim_state_.next_call_whale_index = next_whale_index;
    sim_state_.next_whale_call.whale_id = sim_state_.whales[next_whale_index].id;
    sim_state_.next_whale_call.cycle = 0;
    sim_state_.next_whale_call.sim_time_s = next_time_s;

    // Calculate the whale's location at the exact time of the call.
    sim_state_.trajectory_query.whale_index = next_whale_index;
    sim_state_.trajectory_query.sim_time_s = next_time_s;
    const WhaleWaypoint call_position = trajectory_at_time();

    sim_state_.next_whale_call.whale_call_north_m = call_position.whale_north_m;
    sim_state_.next_whale_call.whale_call_east_m = call_position.whale_east_m;

}

/**
 * Shut down the whale-simulation node when requested by the orchestrator.
 *
 * The subscriber latches a true shutdown request in the state. Once requested,
 * stop the shared ROS context so the executor can exit normally.
 */
void WhaleNode::handle_shutdown() {
    if (!sim_state_.shutdown_requested_b) return;
    RCLCPP_INFO(get_logger(), "Shutting down whale node."); // Report the shutdown before stopping the ROS context.
    rclcpp::shutdown();                                     // Stop the ROS executor and allow the process to exit normally.
}

/** Publish the next scheduled call at the whale simulator's completed cycle. */
void WhaleNode::publish_pending_call() {
    if (!sim_state_.next_whale_call_available_b) return;
    sim_state_.next_whale_call.cycle = sim_state_.completed_cycle;
    whale_call_pub_->publish(sim_state_.next_whale_call);
    ++sim_state_.whales.at(sim_state_.next_call_whale_index).next_call_index;
    sim_state_.next_whale_call_available_b = false;
}

//----------------------------------------------------
// Helper functions
//----------------------------------------------------

/**
 * Calculate a whale's position and velocity at a requested simulation time.
 *
 * The position is interpolated from the constant velocity stored for the active trajectory segment. Before activation or after trajectory completion,
 * the whale remains stationary.
 *
 * @return Whale position and velocity at the requested time.
 */
WhaleWaypoint WhaleNode::trajectory_at_time() const {
    const Whale &whale = sim_state_.whales.at(sim_state_.trajectory_query.whale_index);
    const double time_s = sim_state_.trajectory_query.sim_time_s;
    if (whale.trajectory.empty()) throw std::runtime_error("Whale trajectory contains no points.");

    const WhaleWaypoint &first = whale.trajectory.front();
    const WhaleWaypoint &last = whale.trajectory.back();

    // Before activation, keep the whale stationary at its initial location.
    if (time_s < first.sim_time_s) return {time_s, first.whale_north_m, first.whale_east_m, 0.0, 0.0};

    // After the generated trajectory ends, keep the whale stationary at its final location rather than extrapolating beyond the monitored corridor.
    if (time_s >= last.sim_time_s) return {time_s, last.whale_north_m, last.whale_east_m, 0.0, 0.0};

    // Find the first trajectory point after the requested time. The preceding point describes the active constant-velocity segment.
    const auto next = std::upper_bound(
        whale.trajectory.begin(), whale.trajectory.end(), time_s, [](double requested_time_s, const WhaleWaypoint &point) {
            return requested_time_s < point.sim_time_s;
        }
    );

    const WhaleWaypoint &segment = *(next - 1);
    const double elapsed_time_s = time_s - segment.sim_time_s;

    // Advance from the segment's starting position using its stored velocity.
    return {
        time_s,
        segment.whale_north_m + segment.whale_velocity_north_mps * elapsed_time_s,
        segment.whale_east_m + segment.whale_velocity_east_mps * elapsed_time_s,
        segment.whale_velocity_north_mps,
        segment.whale_velocity_east_mps,
    };
}

/**
 * Advance all whale states to the end of the pending simulation batch, publish
 * ground truth, and record every scheduled trajectory sample crossed by it.
 */
void WhaleNode::update_whales() {
    const WhaleStepInput &step = sim_state_.pending_sim_step;
    // Reject an empty batch because subtracting one from step_count would underflow.
    if (step.step_count == 0) {
        RCLCPP_ERROR(get_logger(), "Received a simulation batch with zero steps.");
        return;
    }

    // Calculate the time and cycle reached at the end of the batch.
    sim_state_.sim_time_s = step.sim_time_s + static_cast<double>(step.step_count) * step.timestep;
    sim_state_.completed_cycle = step.current_cycle + step.step_count - 1;

    sim_state_.current_whale_states = whale_interfaces::msg::WhaleStateArray{};
    sim_state_.current_whale_states.cycle = sim_state_.completed_cycle;
    sim_state_.current_whale_states.whales.reserve(sim_state_.whales.size());

    for (std::size_t whale_index = 0; whale_index < sim_state_.whales.size(); ++whale_index) {
        Whale &whale = sim_state_.whales[whale_index];
        // Update each whale from its precomputed trajectory. Absolute-time lookup keeps whale movement independent of the ROS batch size.
        sim_state_.trajectory_query.whale_index = whale_index;
        sim_state_.trajectory_query.sim_time_s = sim_state_.sim_time_s;
        const WhaleWaypoint position = trajectory_at_time();
        whale.whale_north_m = position.whale_north_m;
        whale.whale_east_m = position.whale_east_m;
        whale.whale_velocity_north_mps = position.whale_velocity_north_mps;
        whale.whale_velocity_east_mps = position.whale_velocity_east_mps;
        whale.whale_active_b = sim_state_.sim_time_s >= whale.start_time_s && sim_state_.sim_time_s < whale.trajectory.back().sim_time_s;
        whale.whale_complete_b = sim_state_.sim_time_s >= whale.trajectory.back().sim_time_s;

        // Build and publish the ground-truth state at the completed batch time.
        whale_interfaces::msg::WhaleState state;
        state.whale_active_b_b = whale.whale_active_b;
        state.whale_id = whale.id;
        state.cycle = sim_state_.completed_cycle;
        state.whale_north_m = whale.whale_north_m;
        state.whale_east_m = whale.whale_east_m;
        state.whale_velocity_north_mps = whale.whale_velocity_north_mps;
        state.whale_velocity_east_mps = whale.whale_velocity_east_mps;
        sim_state_.current_whale_states.whales.push_back(state);
    }

    whale_states_pub_->publish(sim_state_.current_whale_states);

    // Record every requested output time crossed by this batch.
    while (sim_state_.next_whale_log_time_s <= sim_state_.sim_time_s + 1.0e-9) {
        // Skip stale output boundaries that precede the current batch.
        if (sim_state_.next_whale_log_time_s < step.sim_time_s) {
            sim_state_.next_whale_log_time_s += sim_state_.whale_sample_period_s;
            continue;
        }

        // Map the requested sample time to the corresponding cycle in this batch.
        const double cycle_offset = (sim_state_.next_whale_log_time_s - step.sim_time_s) / step.timestep;
        const std::uint64_t sample_offset = std::min<std::uint64_t>(step.step_count - 1, static_cast<std::uint64_t>(std::max(0.0, std::floor(cycle_offset))));
        const std::uint64_t sample_cycle = step.current_cycle + sample_offset;

        // Record every whale at the exact scheduled output time.
        for (std::size_t whale_index = 0; whale_index < sim_state_.whales.size(); ++whale_index) {
            const Whale &whale = sim_state_.whales[whale_index];
            sim_state_.trajectory_query.whale_index = whale_index;
            sim_state_.trajectory_query.sim_time_s = sim_state_.next_whale_log_time_s;
            const WhaleWaypoint sample = trajectory_at_time();
            whale_log_ << sample_cycle << ',' << sim_state_.next_whale_log_time_s << ',' << whale.id << ',' << whale.group_index + 1 << ',' << sample.whale_north_m << ',' << sample.whale_east_m << ',' << '\n';
        }

        sim_state_.next_whale_log_time_s += sim_state_.whale_sample_period_s;
    }
}
