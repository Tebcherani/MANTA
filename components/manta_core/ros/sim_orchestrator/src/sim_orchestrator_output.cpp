#include "sim_orchestrator/sim_orchestrator_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>

extern "C" {
    #include "odyssey_sim.h"
}

/**
 * Select and publish the next batch of simulation cycles.
 *
 * The normal configured batch size is shortened when the mission is near completion or when a scheduled whale call occurs within the batch.
 */
void SimOrchestratorNode::publish_current_step() {
    manta_interfaces::msg::SimStep step;

    // Describe the first cycle, starting time, and physics timestep of the batch that the vehicle simulators are about to process.
    step.current_cycle = static_cast<std::uint64_t>(sim_state_.current_cycle);
    step.sim_time_s = sim_state_.sim_time_s;
    step.timestep_s = sim_config_.timestep_s;

    // Use the configured maximum batch size unless fewer simulation cycles remain.
    const int remaining_cycles = sim_config_.num_cycles - sim_state_.current_cycle;
    int batch_steps = std::min(steps_per_message_, remaining_cycles);

    // Stop at the next communication boundary anchored to time zero.
    if (communication_steps_ > 0) {
        const int steps_into_interval = sim_state_.current_cycle % communication_steps_;
        const int steps_until_communication = communication_steps_ - steps_into_interval;
        batch_steps = std::min(batch_steps, steps_until_communication);
    }

    // Shorten the batch if it would pass the next scheduled whale call.
    current_batch_reaches_whale_call_b_ = false;
    if (next_whale_call_available_b_) {
        // Calculate how many simulated seconds remain before the next scheduled whale call.
        const double time_until_call_s = next_whale_call_.sim_time_s - sim_state_.sim_time_s;

        if (time_until_call_s > 0.0) {
            // Convert the remaining time to a number of physics steps. Every published batch must contain at least one step.
            const int steps_until_call = std::max(1, static_cast<int>(std::ceil(time_until_call_s / sim_config_.timestep_s)));

            // If the call occurs within the normal batch, shorten the batch so it ends at the call.
            if (steps_until_call <= batch_steps) {
                batch_steps = steps_until_call;
                current_batch_reaches_whale_call_b_ = true;
            }
        }
    }

    // Send the selected number of internal physics steps to every vehicle simulator.
    current_batch_steps_ = batch_steps;
    step.step_count = static_cast<std::uint32_t>(batch_steps);
    step.control_period = static_cast<std::uint64_t>(sim_config_.control_period);
    step.world.gravity_mps2 = world_state_->gravity_mps2;
    step.world.magnetic_north_gauss = world_state_->magnetic_north_gauss;
    step.world.magnetic_east_gauss = world_state_->magnetic_east_gauss;
    step.world.magnetic_down_gauss = world_state_->magnetic_down_gauss;
    step.world.seawater_density_kgpm3 = world_state_->seawater_density_kgpm3;

    sim_step_pub_->publish(step);
}

/**
 * Publish a whale-call event and record the exact event in the output log.
 *
 * @param call Whale-call event scheduled by the whale simulator.
 */
void SimOrchestratorNode::publish_whale_call(const whale_interfaces::msg::WhaleCall &call) {
    whale_call_pub_->publish(call); // Publish the event for the whale-detection node.
    whale_call_log_ << call.cycle << ',' << std::setprecision(17) << call.sim_time_s << ',' << call.whale_id << ',' << call.whale_call_north_m << ',' << call.whale_call_east_m << '\n';   // Record the published event for simulation analysis and review.
}