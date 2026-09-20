#include "whale_detection_sim/whale_detection_node.hpp"

/**
 * Shut down the whale-detection node when requested by the orchestrator.
 *
 * A message containing `false` is ignored because it does not request a shutdown. A message containing `true` stops the shared ROS context.
 *
 * @param message Shutdown request published by the simulation orchestrator.
 */
void WhaleDetectionNode::handle_shutdown(const std_msgs::msg::Bool::SharedPtr message) {
    if (!message->data) return;                                 // Ignore messages that do not request a shutdown.
    RCLCPP_INFO(get_logger(), "Shutting down whale detector."); // Report the shutdown before stopping the ROS context.
    rclcpp::shutdown();                                         // Stop this node's executor and allow the process to exit normally.
}

/**
 * Store a vehicle's latest state and attempt to process a pending whale call.
 *
 * The whale detector must have the state of every configured vehicle at the whale call's simulation cycle before it can evaluate detection distances.
 *
 * @param message Latest state published by the vehicle.
 * @param vehicle_id Configured identifier of the vehicle that published the state.
 */
void WhaleDetectionNode::handle_vehicle_state(const manta_interfaces::msg::VehicleState::SharedPtr message, const std::string &vehicle_id) {
    latest_vehicle_states_[vehicle_id] = *message;  // Store or replace this vehicle's most recently received state.
    try_process_pending_call();                     // Process the pending whale call if every vehicle has now reached its cycle.
}

/**
 * Store a newly received whale call and attempt to process it.
 *
 * Only one whale call may be pending at a time. The orchestrator waits for a WhaleCallProcessed acknowledgement before publishing the next call, so
 * receiving another call while one is pending indicates a synchronization error.
 *
 * @param call Whale-call event published by the whale simulator.
 */
void WhaleDetectionNode::handle_whale_call(const manta_interfaces::msg::WhaleCall::SharedPtr call) {
    // Reject a new call if the previous call has not yet been evaluated.
    if (pending_call_available_) {
        RCLCPP_ERROR(
            get_logger(), "Received whale call %s before the previous call was processed.",
            call->whale_id.c_str()
        );
        return;
    }

    // Store the call until every configured vehicle reaches its event cycle.
    pending_call_ = *call;
    pending_call_available_ = true;

    // Process immediately if all required vehicle states are already available.
    try_process_pending_call();
}