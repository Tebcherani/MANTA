#include "sim_orchestrator/sim_orchestrator_node.hpp"

#include <chrono>
#include <cmath>

//----------------------------------------------------
// Handle functions
//----------------------------------------------------

/** Use the whale node's actual batch length before stepping vehicles. */
void SimOrchestratorNode::handle_whale_step_result() {
    const auto &result = state_.received_whale_step_result;
    if (!state_.waiting_for_whale_step_b) return;
    if (!result.succeeded_b || result.completed_step_count == 0 ||
        result.completed_step_count > state_.current_step.step_count ||
        result.completed_cycle != state_.current_step.current_cycle + result.completed_step_count - 1) {
        RCLCPP_ERROR(get_logger(), "Invalid whale step result: %s", result.detail.c_str());
        request_simulation_shutdown(true);
        return;
    }

    state_.current_batch_steps = static_cast<int>(result.completed_step_count);
    state_.current_step.step_count = result.completed_step_count;
    state_.current_batch_has_whale_call_b = result.whale_call_published_b;
    state_.current_step.whale_call_expected_b = result.whale_call_published_b;
    if (result.whale_call_published_b) {
        state_.current_whale_call = result.whale_call;
        if (state_.current_whale_call.cycle != result.completed_cycle) {
            RCLCPP_ERROR(get_logger(), "Whale call cycle does not match the completed whale step.");
            request_simulation_shutdown(true);
            return;
        }
    }

    state_.waiting_for_whale_step_b = false;
    sim_step_pub_->publish(state_.current_step);
}

/**
 * Complete orchestrator shutdown after the shutdown message propagation delay.
 */
void SimOrchestratorNode::handle_shutdown_timer() {
    shutdown_timer_->cancel();  // Prevent the periodic wall timer from firing again.
    rclcpp::shutdown();         // Stop the orchestrator's ROS context.
}

/**
 * Process one vehicle's result for the current simulation-step batch.
 *
 * The handler validates the vehicle identity and completed cycle, rejects duplicate or incomplete results, handles terminal mission states, and checks
 * whether every active vehicle has completed the current batch.
 *
 * Reads the received result and subscribed vehicle ID stored in state.
 */
void SimOrchestratorNode::handle_step_result() {
    const auto *msg = &state_.received_step_result;
    const std::string &expected_vehicle_id = state_.expected_step_vehicle_id;
    VehicleState &state = state_.vehicle_states.at(expected_vehicle_id);

    if (msg->vehicle_id != expected_vehicle_id) {
        RCLCPP_WARN(
            get_logger(), "Ignored step result from unexpected vehicle '%s' on the %s topic.",
            msg->vehicle_id.c_str(), expected_vehicle_id.c_str());
        return;
    }

    const std::uint64_t first_cycle = static_cast<std::uint64_t>(state_.sim_state.current_cycle);
    const std::uint64_t last_cycle = first_cycle + static_cast<std::uint64_t>(state_.current_batch_steps) - 1;

    // Terminal results may occur partway through a batch.
    if (msg->completed_cycle < first_cycle || msg->completed_cycle > last_cycle) {
        RCLCPP_WARN(
            get_logger(), "Ignored result from %s for cycle %llu; expected cycle %llu through %llu.", 
            msg->vehicle_id.c_str(), static_cast<unsigned long long>(msg->completed_cycle), static_cast<unsigned long long>(first_cycle), static_cast<unsigned long long>(last_cycle)
        );
        return;
    }

    // A running vehicle must have completed the entire batch.
    if (msg->status == manta_interfaces::msg::SimStepResult::RUNNING && msg->completed_cycle != last_cycle) {
        RCLCPP_WARN(
            get_logger(), "Ignored incomplete batch result from %s at cycle %llu; expected cycle %llu.", 
            msg->vehicle_id.c_str(), static_cast<unsigned long long>(msg->completed_cycle), static_cast<unsigned long long>(last_cycle)
        );
        return;
    }

    if (state.step_complete_b) {
        RCLCPP_WARN(
            get_logger(), "Ignored duplicate result from %s for cycle %llu.",
            msg->vehicle_id.c_str(), static_cast<unsigned long long>(msg->completed_cycle)
        );
        return;
    }

    if (msg->status == manta_interfaces::msg::SimStepResult::ERROR) {
        RCLCPP_ERROR(
            get_logger(), "Vehicle %s failed during cycle %llu: %s",
            msg->vehicle_id.c_str(), static_cast<unsigned long long>(msg->completed_cycle), msg->detail.c_str()
        );
        rclcpp::shutdown();
        return;
    }

    // Mission status handling
    if (msg->status == manta_interfaces::msg::SimStepResult::MISSION_COMPLETE) {
        RCLCPP_INFO(
            get_logger(), "Vehicle %s completed its mission during cycle %llu.", 
            msg->vehicle_id.c_str(), static_cast<unsigned long long>(msg->completed_cycle)
        );
        state.active_b = false;
        state.step_complete_b = true;
        check_all_step_results_received();
        return;
    }

    if (msg->status == manta_interfaces::msg::SimStepResult::ABORTED) {
        RCLCPP_WARN(
            get_logger(), "Vehicle %s aborted its mission during cycle %llu: %s", 
            msg->vehicle_id.c_str(), static_cast<unsigned long long>(msg->completed_cycle), msg->detail.c_str()
        );                
        state.active_b = false;
        state.step_complete_b = true;
        check_all_step_results_received();
        return;
    }

    // Step for this vehicle is complete
    state.step_complete_b = true;

    // Check if all vehicles have completed this step
    check_all_step_results_received();
}

/**
 * Continue simulation coordination after all active vehicles finish a batch.
 *
 * This method waits for every active vehicle to acknowledge the current simulation batch. Once all results have arrived, it checks for mission
 * completion, advances the shared simulation clock, and publishes the next simulation batch.
 *
 * A vehicle marked inactive is excluded from subsequent synchronization. Simulation shutdown is requested if all vehicles finish, the configured
 * cycle limit is reached, or the shared simulation state cannot be advanced.
 */
void SimOrchestratorNode::check_all_step_results_received() {
    // Wait until every active vehicle has returned a result for the current simulation cycle.
    for (const auto &entry : state_.vehicle_states) {
        const VehicleState &state = entry.second;
        if (state.active_b && !state.step_complete_b) return;
    }

    // Determine whether any vehicle still has an active mission.
    bool any_vehicle_active = false;

    for (const auto &entry : state_.vehicle_states) {
        if (entry.second.active_b) {
            any_vehicle_active = true;
            break;
        }
    }

    // End the simulation when every vehicle mission has completed.
    if (!any_vehicle_active) {
        RCLCPP_INFO(get_logger(), "All vehicle missions have completed.");
        request_simulation_shutdown();
        return;
    }

    // Use the actual published batch size because a whale-call event may have shortened the batch below the configured maximum.
    const int completed_steps = state_.current_batch_steps;

    // Clear vehicle acknowledgements before beginning the next batch.
    for (auto &entry : state_.vehicle_states) entry.second.step_complete_b = false;

    // Advance the shared clock through every completed native timestep.
    for (int step = 0; step < completed_steps; ++step) {
        if (!update_sim_state(&state_.sim_state, state_.sim_config.timestep_s)) {
            RCLCPP_ERROR(get_logger(), "Failed to advance the simulation state.");
            request_simulation_shutdown(true);
            return;
        }
    }

    if (state_.current_batch_has_whale_call_b) {
        record_whale_call();
        state_.current_batch_has_whale_call_b = false;
    }

    if (state_.sim_state.current_cycle >= state_.sim_config.num_cycles) {
        RCLCPP_INFO(get_logger(), "Simulation completed all %d configured cycles.", state_.sim_config.num_cycles);
        request_simulation_shutdown();
        return;
    }

    publish_current_step();
}

/**
 * Start coordinated simulation stepping once every vehicle is ready.
 *
 * This method may be called whenever a vehicle publishes a readiness update. It returns until every configured vehicle has completed initialization.
 * Once all nodes are ready, it marks the simulation as started and sends the first whale step.
 */
void SimOrchestratorNode::check_all_vehicles_initialized() {
    // Prevent startup from being triggered more than once.
    if (state_.simulation_started_b) return;

    // Wait until every configured vehicle reports that it is ready.
    for (const auto &entry : state_.vehicle_states) {
        const VehicleState &state = entry.second;
        if (!state.ready_b) return;
    }

    if (state_.wait_for_whale_estimators_b && state_.ready_whale_estimators.size() != state_.vehicle_states.size()) return;

    if (!state_.whale_ready_b || !state_.detection_ready_b) return;

    // Mark coordinated startup as complete.
    state_.simulation_started_b = true;
    RCLCPP_INFO(get_logger(), "All %zu vehicles%s are initialized.", state_.vehicle_states.size(),
        state_.wait_for_whale_estimators_b ? " and whale estimators" : "");
    
    publish_current_step();
}

/**
 * Request an orderly shutdown of every simulation node.
 *
 * The shutdown message is published before stopping the orchestrator's ROS context. A short timer gives the reliable message time to reach the vehicle,
 * whale, and detection nodes.
 *
 * @param failed true if the simulation is stopping because of an error; otherwise, false.
 */
void SimOrchestratorNode::request_simulation_shutdown(bool failed) {            
    // Preserve the final simulation outcome for main() to report.    
    state_.simulation_failed_b = failed;
    
    // Notify every simulation node that it should shut down.
    std_msgs::msg::Bool shutdown_message;
    shutdown_message.data = true;

    RCLCPP_INFO(get_logger(), "Requesting shutdown of all simulation nodes.");
    shutdown_pub_->publish(shutdown_message);

    // Delay local shutdown briefly so the published message can reach all subscribers before the orchestrator stops its ROS context.    
    shutdown_timer_ = create_wall_timer(
        std::chrono::milliseconds(500), [this]() {handle_shutdown_timer();}
    );
}
