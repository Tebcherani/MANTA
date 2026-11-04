#include "sim_orchestrator/sim_orchestrator_node.hpp"

#include <algorithm>
#include <cstdint>
#include <iomanip>

/** Select the next batch and ask the whale node to advance first. */
void SimOrchestratorNode::publish_current_step() {
    auto &step = state_.current_step;
    step = manta_interfaces::msg::SimStep{};
    step.current_cycle = static_cast<std::uint64_t>(state_.sim_state.current_cycle);
    step.sim_time_s = state_.sim_state.sim_time_s;
    step.timestep_s = state_.sim_config.timestep_s;

    const int remaining_cycles = state_.sim_config.num_cycles - state_.sim_state.current_cycle;
    int batch_steps = std::min(state_.steps_per_message, remaining_cycles);
    if (state_.communication_steps > 0) {
        const int steps_into_interval = state_.sim_state.current_cycle % state_.communication_steps;
        batch_steps = std::min(batch_steps, state_.communication_steps - steps_into_interval);
    }

    step.step_count = static_cast<std::uint32_t>(batch_steps);
    step.control_period = static_cast<std::uint64_t>(state_.sim_config.control_period);
    step.whale_call_expected_b = false;
    step.world.gravity_mps2 = state_.world_state->gravity_mps2;
    step.world.magnetic_north_gauss = state_.world_state->magnetic_north_gauss;
    step.world.magnetic_east_gauss = state_.world_state->magnetic_east_gauss;
    step.world.magnetic_down_gauss = state_.world_state->magnetic_down_gauss;
    step.world.seawater_density_kgpm3 = state_.world_state->seawater_density_kgpm3;

    state_.waiting_for_whale_step_b = true;
    whale_step_pub_->publish(step);
    log_state_if_changed();
}

/** Record the whale call reported by the whale simulator. */
void SimOrchestratorNode::record_whale_call() {
    const auto &call = state_.current_whale_call;
    whale_call_log_ << call.cycle << ',' << std::setprecision(17) << call.sim_time_s << ','
                    << call.whale_id << ',' << call.whale_call_north_m << ','
                    << call.whale_call_east_m << '\n';
}
