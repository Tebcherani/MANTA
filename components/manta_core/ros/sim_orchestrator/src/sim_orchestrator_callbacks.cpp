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
    if (!waiting_for_whale_call_processing_) return;

    if (msg->call_cycle != next_whale_call_.cycle ||
        std::abs(msg->sim_time - next_whale_call_.sim_time) > 1.0e-9) {
        return;
    }

    // Ignore messages from vehicles outside the configured fleet.
    if (vehicle_states_.find(msg->vehicle_id) == vehicle_states_.end()) {
        return;
    }

    // A set automatically ignores duplicate acknowledgments.
    completed_bearing_processors_.insert(msg->vehicle_id);

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
    if (!response->available) {
        next_whale_call_available_ = false;
        publish_current_step();
        return;
    }

    // Store the next scheduled whale call.
    next_whale_call_ = response->call;
    next_whale_call_available_ = true;

    // Publish calls that fall within the timestep vehicles already reached.
    if (next_whale_call_.sim_time <= sim_state_.sim_time) {
        next_whale_call_.cycle = sim_state_.current_cycle > 0 ? static_cast<std::uint64_t>(sim_state_.current_cycle - 1) : 0;

        // Prepare the barrier before any acknowledgments can arrive.
        whale_detection_complete_ = false;
        expected_bearing_processors_.clear();
        completed_bearing_processors_.clear();
        waiting_for_whale_call_processing_ = true;

        publish_whale_call(next_whale_call_);
        next_whale_call_available_ = false;
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
 * @param msg ROS message containing the vehicle's completed cycle and status.
 * @param expected_vehicle_id Vehicle associated with the subscribed result topic.
 */
void SimOrchestratorNode::handle_step_result(const manta_interfaces::msg::SimStepResult::SharedPtr msg, const std::string &expected_vehicle_id) {
    VehicleState &state = vehicle_states_.at(expected_vehicle_id);

    if (msg->vehicle_id != expected_vehicle_id) {
        RCLCPP_WARN(
            get_logger(), "Ignored step result from unexpected vehicle '%s' on the %s topic.",
            msg->vehicle_id.c_str(), expected_vehicle_id.c_str());
        return;
    }

    const std::uint64_t first_cycle = static_cast<std::uint64_t>(sim_state_.current_cycle);
    const std::uint64_t last_cycle = first_cycle + static_cast<std::uint64_t>(current_batch_steps_) - 1;

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

    if (state.step_complete) {
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
        state.active = false;
        state.step_complete = true;
        check_all_step_results_received();
        return;
    }

    if (msg->status == manta_interfaces::msg::SimStepResult::ABORTED) {
        RCLCPP_WARN(
            get_logger(), "Vehicle %s aborted its mission during cycle %llu: %s", 
            msg->vehicle_id.c_str(), static_cast<unsigned long long>(msg->completed_cycle), msg->detail.c_str()
        );                
        state.active = false;
        state.step_complete = true;
        check_all_step_results_received();
        return;
    }

    // Step for this vehicle is complete
    state.step_complete = true;

    // Check if all vehicles have completed this step
    check_all_step_results_received();
}

/**
 * Record a vehicle's readiness and check whether the fleet can start.
 *
 * This handler is called when a vehicle publishes its initialization status. Once every configured vehicle is ready, the orchestrator can begin advancing
 * the simulation.
 *
 * @param msg ROS message containing the vehicle's readiness state.
 * @param vehicle_id Identifier of the vehicle that published the message.
 */
void SimOrchestratorNode::handle_vehicle_ready( const std_msgs::msg::Bool::SharedPtr msg, const std::string &vehicle_id) {
    vehicle_states_.at(vehicle_id).ready = msg->data;   // Store the vehicle's latest readiness state.
    check_all_vehicles_initialized();                   // Begin the simulation if the entire fleet is ready.
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
    if (!waiting_for_whale_call_processing_) {
        RCLCPP_WARN(get_logger(), "Ignored unexpected whale-call acknowledgement.");
        return;
    }

    // Ensure the acknowledgement corresponds to the pending whale call.
    const bool wrong_whale = msg->whale_id != next_whale_call_.whale_id;
    const bool wrong_cycle = msg->cycle != next_whale_call_.cycle;
    const bool wrong_time = std::fabs(msg->sim_time - next_whale_call_.sim_time) > 1.0e-9;

    if (wrong_whale || wrong_cycle || wrong_time) {
        RCLCPP_WARN(
            get_logger(), "Ignored acknowledgement for unexpected whale call %s at cycle %llu.",
            msg->whale_id.c_str(), static_cast<unsigned long long>(msg->cycle)
        );
        return;
    }

    // The detector has finished and reported which receivers got bearings.
    whale_detection_complete_ = true;

    expected_bearing_processors_.clear();

    // Wait for local receipt/storage by every AUV that detected this call.
    if (wait_for_bearing_processing_) {
        for (const auto &receiver_id : msg->bearing_receiver_ids) {
            expected_bearing_processors_.insert(receiver_id);
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
    if (!waiting_for_whale_call_processing_ ||
        !whale_detection_complete_) {
        return;
    }

    // Wait for every required vehicle, regardless of acknowledgment order.
    for (const auto &vehicle_id : expected_bearing_processors_) {
        if (completed_bearing_processors_.count(vehicle_id) == 0) return;
    }

    waiting_for_whale_call_processing_ = false;
    request_next_whale_call(next_whale_call_.sim_time);
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
    for (const auto &entry : vehicle_states_) {
        const VehicleState &state = entry.second;
        if (state.active && !state.step_complete) return;
    }

    // Determine whether any vehicle still has an active mission.
    bool any_vehicle_active = false;

    for (const auto &entry : vehicle_states_) {
        if (entry.second.active) {
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
    const int completed_steps = current_batch_steps_;

    // End the simulation when this batch reaches the configured cycle limit.
    if (sim_state_.current_cycle + completed_steps >= sim_config_.num_cycles) {
        RCLCPP_INFO(get_logger(), "Simulation completed all %d configured cycles.", sim_config_.num_cycles);
        request_simulation_shutdown();
        return;
    }

    // Clear vehicle acknowledgements before beginning the next batch.
    for (auto &entry : vehicle_states_) entry.second.step_complete = false;

    // Advance the shared clock through every completed native timestep.
    for (int step = 0; step < completed_steps; ++step) {
        if (!update_sim_state(&sim_state_, sim_config_.timestep)) {
            RCLCPP_ERROR(get_logger(), "Failed to advance the simulation state.");
            request_simulation_shutdown(true);
            return;
        }
    }

    // If the shortened batch reached a whale call, publish the event before allowing the simulation to continue.
    if (current_batch_reaches_whale_call_) {
        next_whale_call_.cycle = static_cast<std::uint64_t>(sim_state_.current_cycle - 1);

        // Prepare the barrier before any acknowledgments can arrive.
        whale_detection_complete_ = false;
        expected_bearing_processors_.clear();
        completed_bearing_processors_.clear();
        waiting_for_whale_call_processing_ = true;

        publish_whale_call(next_whale_call_);

        next_whale_call_available_ = false;
        current_batch_reaches_whale_call_ = false;
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
    if (simulation_started_) return;

    // Wait until every configured vehicle reports that it is ready.
    for (const auto &entry : vehicle_states_) {
        const VehicleState &state = entry.second;
        if (!state.ready) return;
    }

    // Mark coordinated startup as complete.
    simulation_started_ = true;
    RCLCPP_INFO(get_logger(), "All %zu vehicles are initialized.", vehicle_states_.size());
    
    // Obtain the first whale call before selecting the first simulation batch.
    request_next_whale_call(sim_state_.sim_time);
}

/**
 * Request the first scheduled whale call after a given simulation time.
 *
 * The request is sent asynchronously so the orchestrator can continue processing ROS events while the whale node determines the next call.
 * If the scheduling service is unavailable, the simulation is stopped with an error.
 *
 * @param after_time Simulation time after which the requested call must occur.
 */
void SimOrchestratorNode::request_next_whale_call(double after_time) {
    // Wait briefly for the whale-call scheduling service to become available.
    if (!next_whale_call_client_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(get_logger(), "Whale-call scheduling service is unavailable.");
        request_simulation_shutdown(true);
        return;
    }

    // Request the first call strictly after the supplied simulation time.
    auto request = std::make_shared<whale_interfaces::srv::GetNextWhaleCall::Request>();
    request->after_time = after_time;

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
    simulation_failed_ = failed;
    
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