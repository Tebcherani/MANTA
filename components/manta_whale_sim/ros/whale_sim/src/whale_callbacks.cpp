#include "whale_sim/whale_node.hpp"

//----------------------------------------------------
// Handle functions
//----------------------------------------------------

/**
 * Find the first scheduled whale call after a requested simulation time.
 *
 * Calls at or before `request.whale_call_after_time_s` have already been processed and are discarded. The earliest remaining call across every whale is returned with
 * the whale's position at that exact simulated time.
 *
 * @param request Request containing the last processed simulation time.
 * @param response Response populated with the next available whale call.
 */
void WhaleNode::handle_get_next_call(const whale_interfaces::srv::GetNextWhaleCall::Request &request, whale_interfaces::srv::GetNextWhaleCall::Response &response) {
    // Clear the previous result before searching for the next call.
    sim_state_.next_whale_call = whale_interfaces::msg::WhaleCall{};
    sim_state_.next_whale_call_available_b = false;

    // No whale has been selected yet.
    Whale *next_whale = nullptr;
    double next_time_s = std::numeric_limits<double>::infinity();

    // Search every whale for its earliest unprocessed call.
    for (auto &whale : sim_state_.whales) {
        // Discard calls that the orchestrator has already processed.
        while (whale.next_call_index < whale.whale_call_times_s.size() && whale.whale_call_times_s[whale.next_call_index] <= request.whale_call_after_time_s) {
            ++whale.next_call_index;
        }

        // Skip whales that have no scheduled calls remaining.
        if (whale.next_call_index >= whale.whale_call_times_s.size()) continue;
        const double candidate_time_s = whale.whale_call_times_s[whale.next_call_index];

        // Retain the earliest call found across all whales.
        if (candidate_time_s < next_time_s) {
            next_time_s = candidate_time_s;
            next_whale = &whale;
        }
    }

    // Report that no whale calls remain in the simulation.
    if (next_whale == nullptr) {
        response.whale_call_available_b = sim_state_.next_whale_call_available_b;
        response.whale_call = sim_state_.next_whale_call;
        return;
    }

    // Identify the next call and its scheduled simulation time.
    sim_state_.next_whale_call_available_b = true;
    sim_state_.next_whale_call.whale_id = next_whale->id;
    sim_state_.next_whale_call.cycle = 0;
    sim_state_.next_whale_call.sim_time_s = next_time_s;

    // Calculate the whale's location at the exact time of the call.
    const WhaleWaypoint call_position = trajectory_at_time(*next_whale, next_time_s);

    sim_state_.next_whale_call.whale_call_north_m = call_position.whale_north_m;
    sim_state_.next_whale_call.whale_call_east_m = call_position.whale_east_m;

    response.whale_call_available_b = sim_state_.next_whale_call_available_b;
    response.whale_call = sim_state_.next_whale_call;
}

/**
 * Shut down the whale-simulation node when requested by the orchestrator.
 *
 * Messages containing `false` are ignored. A message containing `true` stops the shared ROS context so the executor can exit normally.
 *
 * @param msg Shutdown request published by the simulation orchestrator.
 */
void WhaleNode::handle_shutdown(const std_msgs::msg::Bool::SharedPtr msg) {
    if (!msg->data) return;                             // Ignore messages that do not request shutdown.
    sim_state_.shutdown_requested_b = true;
    RCLCPP_INFO(get_logger(), "Shutting down whale node."); // Report the shutdown before stopping the ROS context.
    rclcpp::shutdown();                                     // Stop the ROS executor and allow the process to exit normally.
}

/**
 * Handle a simulation-step message from the orchestrator.
 *
 * The callback passes the received simulation batch to the whale-update routine, which advances every whale to the end of the batch, publishes
 * ground-truth states, and records any trajectory samples crossed by it.
 *
 * @param step Simulation batch containing the current time and number of steps.
 */
void WhaleNode::handle_sim_step(const manta_interfaces::msg::SimStep::SharedPtr step) {
    update_whales(*step);   // Advance and publish the whales using the received simulation batch.
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
 * @param whale Whale whose trajectory will be evaluated.
 * @param time Requested simulation time.
 * @return Whale position and velocity at the requested time.
 */
WhaleNode::WhaleWaypoint WhaleNode::trajectory_at_time(const Whale &whale, double time_s) const {
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
 * Advance all whale states to the end of a simulation batch, publish ground truth, and record every scheduled trajectory sample crossed by the batch.
 *
 * @param step Simulation batch published by the orchestrator.
 */
void WhaleNode::update_whales(const manta_interfaces::msg::SimStep &step) {
    // Reject an empty batch because subtracting one from step_count would underflow.
    if (step.step_count == 0) {
        RCLCPP_ERROR(get_logger(), "Received a simulation batch with zero steps.");
        return;
    }

    // Calculate the time and cycle reached at the end of the batch.
    sim_state_.sim_time_s = step.sim_time_s + static_cast<double>(step.step_count) * step.timestep;
    sim_state_.completed_cycle = step.current_cycle + step.step_count - 1;

    whale_interfaces::msg::WhaleStateArray msg;
    msg.cycle = sim_state_.completed_cycle;
    msg.whales.reserve(sim_state_.whales.size());

    for (auto &whale : sim_state_.whales) {
        // Update each whale from its precomputed trajectory. Absolute-time lookup keeps whale movement independent of the ROS batch size.
        const WhaleWaypoint position = trajectory_at_time(whale, sim_state_.sim_time_s);
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
        msg.whales.push_back(state);
    }

    whale_states_pub_->publish(msg);

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
        for (const auto &whale : sim_state_.whales) {
            const WhaleWaypoint sample = trajectory_at_time(whale, sim_state_.next_whale_log_time_s);
            whale_log_ << sample_cycle << ',' << sim_state_.next_whale_log_time_s << ',' << whale.id << ',' << whale.group_index + 1 << ',' << sample.whale_north_m << ',' << sample.whale_east_m << ',' << '\n';
        }

        sim_state_.next_whale_log_time_s += sim_state_.whale_sample_period_s;
    }
}