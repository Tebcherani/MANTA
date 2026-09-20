#include "sim_orchestrator/sim_orchestrator_node.hpp"

#include <cmath>

/**
 * Process the whale simulator's response containing the next scheduled call.
 *
 * If no future call is available, the orchestrator resumes simulation stepping. If the returned call occurs at or before the current simulation time, it is
 * published immediately. Otherwise, simulation stepping continues toward the scheduled event.
 *
 * @param future Asynchronous service result containing the next whale call.
 */
void SimOrchestratorNode::handle_next_whale_call_response(rclcpp::Client<manta_interfaces::srv::GetNextWhaleCall>::SharedFuture future) {
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

        publish_whale_call(next_whale_call_);
        next_whale_call_available_ = false;
        waiting_for_whale_call_processing_ = true;
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
void SimOrchestratorNode::handle_whale_call_processed(const manta_interfaces::msg::WhaleCallProcessed::SharedPtr msg) {
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

    // Allow the next whale call to be requested.
    waiting_for_whale_call_processing_ = false;

    // Search only after every vehicle has evaluated the previous call.
    request_next_whale_call(msg->sim_time);
}