#include "sim_orchestrator/sim_orchestrator_node.hpp"

#include <chrono>
#include <cmath>

//----------------------------------------------------
// Handle functions
//----------------------------------------------------

/**
 * Record one vehicle's completion of the current bearing estimate.
 *
 * Acknowledgments may arrive before the detector's completion message.
 * Both valid and invalid estimates count as finished processing.
 */
void SimOrchestratorNode::handle_bearing_processed(const acoustics_interfaces::msg::BearingProcessed::SharedPtr msg) {
    // Accept acknowledgments only for the call currently being processed.
    if (!state_.waiting_for_whale_call_processing_b) return;

    if (msg->call_cycle != state_.next_whale_call.cycle ||
        std::abs(msg->sim_time_s - state_.next_whale_call.sim_time_s) > 1.0e-9) {
        return;
    }

    // Ignore messages from vehicles outside the configured fleet.
    if (state_.vehicle_states.find(msg->vehicle_id) == state_.vehicle_states.end()) {
        return;
    }

    // A set automatically ignores duplicate acknowledgments.
    state_.completed_bearing_processors.insert(msg->vehicle_id);

    check_whale_call_processing_complete();
}

/**
 * Process the whale simulator's response containing the next scheduled call.
 *
 * If no future call is available, the orchestrator resumes simulation stepping. If the returned call occurs at or before the current simulation time, it is
 * published immediately. Otherwise, simulation stepping continues toward the scheduled event.
 *
 * @param future Asynchronous service result containing the next whale call.
 */
void SimOrchestratorNode::handle_next_whale_call_response(rclcpp::Client<whale_interfaces::srv::GetNextWhaleCall>::SharedFuture future) {
    const auto response = future.get();

    // Continue normal stepping when no future whale call remains.
    if (!response->whale_call_available_b) {
        state_.next_whale_call_available_b = false;
        publish_current_step();
        return;
    }

    // Store the next scheduled whale call.
    state_.next_whale_call = response->whale_call;
    state_.next_whale_call_available_b = true;

    // Publish calls that fall within the timestep vehicles already reached.
    if (state_.next_whale_call.sim_time_s <= state_.sim_state.sim_time_s) {
        state_.next_whale_call.cycle = state_.sim_state.current_cycle > 0 ? static_cast<std::uint64_t>(state_.sim_state.current_cycle - 1) : 0;

        // Prepare the barrier before any acknowledgments can arrive.
        state_.whale_detection_complete_b = false;
        state_.expected_bearing_processors.clear();
        state_.completed_bearing_processors.clear();
        state_.waiting_for_whale_call_processing_b = true;

        publish_whale_call(state_.next_whale_call);
        state_.next_whale_call_available_b = false;
        return;
    }

    // Advance the simulation toward the scheduled whale call.
    publish_current_step();
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
 * Process acknowledgement of a published whale-call event.
 *
 * The acknowledgement is accepted only when the orchestrator is waiting for one and its whale ID, cycle, and simulation time match the pending call.
 * Once accepted, the orchestrator requests the next scheduled whale call.
 *
 * @param msg ROS message identifying the whale call that was processed.
 */
void SimOrchestratorNode::handle_whale_call_processed(const acoustics_interfaces::msg::WhaleCallProcessed::SharedPtr msg) {
    // Ignore acknowledgements when no whale-call processing is pending.
    if (!state_.waiting_for_whale_call_processing_b) {
        RCLCPP_WARN(get_logger(), "Ignored unexpected whale-call acknowledgement.");
        return;
    }

    // Ensure the acknowledgement corresponds to the pending whale call.
    const bool wrong_whale = msg->whale_id != state_.next_whale_call.whale_id;
    const bool wrong_cycle = msg->cycle != state_.next_whale_call.cycle;
    const bool wrong_time = std::fabs(msg->sim_time_s - state_.next_whale_call.sim_time_s) > 1.0e-9;

    if (wrong_whale || wrong_cycle || wrong_time) {
        RCLCPP_WARN(
            get_logger(), "Ignored acknowledgement for unexpected whale call %s at cycle %llu.",
            msg->whale_id.c_str(), static_cast<unsigned long long>(msg->cycle)
        );
        return;
    }

    // The detector has finished and reported which receivers got bearings.
    state_.whale_detection_complete_b = true;

    state_.expected_bearing_processors.clear();

    // Wait for local receipt/storage by every AUV that detected this call.
    if (state_.wait_for_bearing_processing_b) {
        for (const auto &receiver_id : msg->bearing_receiver_ids) {
            state_.expected_bearing_processors.insert(receiver_id);
        }
    }
    
    check_whale_call_processing_complete();
}

//----------------------------------------------------
// Helper functions
//----------------------------------------------------

/**
 * Continue only after detection and all required estimators finish.
 */
void SimOrchestratorNode::check_whale_call_processing_complete() {
    if (!state_.waiting_for_whale_call_processing_b ||
        !state_.whale_detection_complete_b) {
        return;
    }

    // Wait for every required vehicle, regardless of acknowledgment order.
    for (const auto &vehicle_id : state_.expected_bearing_processors) {
        if (state_.completed_bearing_processors.count(vehicle_id) == 0) return;
    }

    state_.waiting_for_whale_call_processing_b = false;
    request_next_whale_call(state_.next_whale_call.sim_time_s);
}

/**
 * Continue simulation coordination after all active vehicles finish a batch.
 *
 * This method waits for every active vehicle to acknowledge the current simulation batch. Once all results have arrived, it checks for mission
 * completion, advances the shared simulation clock, publishes any whale call reached by the batch, and otherwise publishes the next simulation batch.
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

    // End the simulation when this batch reaches the configured cycle limit.
    if (state_.sim_state.current_cycle + completed_steps >= state_.sim_config.num_cycles) {
        RCLCPP_INFO(get_logger(), "Simulation completed all %d configured cycles.", state_.sim_config.num_cycles);
        request_simulation_shutdown();
        return;
    }

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

    // If the shortened batch reached a whale call, publish the event before allowing the simulation to continue.
    if (state_.current_batch_reaches_whale_call_b) {
        state_.next_whale_call.cycle = static_cast<std::uint64_t>(state_.sim_state.current_cycle - 1);

        // Prepare the barrier before any acknowledgments can arrive.
        state_.whale_detection_complete_b = false;
        state_.expected_bearing_processors.clear();
        state_.completed_bearing_processors.clear();
        state_.waiting_for_whale_call_processing_b = true;

        publish_whale_call(state_.next_whale_call);

        state_.next_whale_call_available_b = false;
        state_.current_batch_reaches_whale_call_b = false;
        return;
    }

    // The pending call was farther away than one maximum-sized batch. Continue toward the already known event without requesting it again.
    publish_current_step();
}

/**
 * Start coordinated simulation stepping once every vehicle is ready.
 *
 * This method may be called whenever a vehicle publishes a readiness update. It returns until every configured vehicle has completed initialization.
 * Once all vehicles are ready, it marks the simulation as started and requests the first scheduled whale call.
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

    // Mark coordinated startup as complete.
    state_.simulation_started_b = true;
    RCLCPP_INFO(get_logger(), "All %zu vehicles%s are initialized.", state_.vehicle_states.size(),
        state_.wait_for_whale_estimators_b ? " and whale estimators" : "");
    
    // Obtain the first whale call before selecting the first simulation batch.
    request_next_whale_call(state_.sim_state.sim_time_s);
}

/**
 * Request the first scheduled whale call after a given simulation time.
 *
 * The request is sent asynchronously so the orchestrator can continue processing ROS events while the whale node determines the next call.
 * If the scheduling service is unavailable, the simulation is stopped with an error.
 *
 * @param whale_call_after_time_s Simulation time after which the requested call must occur.
 */
void SimOrchestratorNode::request_next_whale_call(double whale_call_after_time_s) {
    // Wait briefly for the whale-call scheduling service to become available.
    if (!next_whale_call_client_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(get_logger(), "Whale-call scheduling service is unavailable.");
        request_simulation_shutdown(true);
        return;
    }

    // Request the first call strictly after the supplied simulation time.
    auto request = std::make_shared<whale_interfaces::srv::GetNextWhaleCall::Request>();
    request->whale_call_after_time_s = whale_call_after_time_s;

    // Forward the asynchronous response to the corresponding callback.
    next_whale_call_client_->async_send_request(
        request, [this](rclcpp::Client<whale_interfaces::srv::GetNextWhaleCall>::SharedFuture future) {handle_next_whale_call_response(future);}
    );
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
