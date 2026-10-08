#include "whale_detection_sim/whale_detection_node.hpp"

#include <cmath>
#include <iomanip>
#include <vector>

//----------------------------------------------------
// Handle functions
//----------------------------------------------------

/**
 * Shut down the whale-detection node when requested by the orchestrator.
 *
 * A message containing `false` is ignored because it does not request a shutdown. A message containing `true` stops the shared ROS context.
 *
 * @param msg Shutdown request published by the simulation orchestrator.
 */
void WhaleDetectionNode::handle_shutdown(const std_msgs::msg::Bool::SharedPtr msg) {
    if (!msg->data) return;                                     // Ignore messages that do not request a shutdown.
    RCLCPP_INFO(get_logger(), "Shutting down whale detector."); // Report the shutdown before stopping the ROS context.
    rclcpp::shutdown();                                         // Stop this node's executor and allow the process to exit normally.
}

/**
 * Store a vehicle's latest state and attempt to process a pending whale call.
 *
 * The whale detector must have the state of every configured vehicle at the whale call's simulation cycle before it can evaluate detection distances.
 *
 * @param msg Latest state published by the vehicle.
 * @param vehicle_id Configured identifier of the vehicle that published the state.
 */
void WhaleDetectionNode::handle_vehicle_state(const manta_interfaces::msg::VehicleState::SharedPtr msg, const std::string &vehicle_id) {
    latest_vehicle_states_[vehicle_id] = *msg;  // Store or replace this vehicle's most recently received state.
    try_process_pending_call();                 // Process the pending whale call if every vehicle has now reached its cycle.
}

/**
 * Store a newly received whale call and attempt to process it.
 *
 * Only one whale call may be pending at a time. The orchestrator waits for a WhaleCallProcessed acknowledgement before publishing the next call, so
 * receiving another call while one is pending indicates a synchronization error.
 *
 * @param call Whale-call event published by the whale simulator.
 */
void WhaleDetectionNode::handle_whale_call(const whale_interfaces::msg::WhaleCall::SharedPtr call) {
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

//----------------------------------------------------
// Helper functions
//----------------------------------------------------

/**
 * Process the pending whale call once every vehicle has published its state for the call's simulation cycle.
 *
 * The method waits until all configured vehicle states exactly match the pending call cycle. It then calculates the three-dimensional distance from
 * each vehicle to the whale, publishes and records detections within range, and acknowledges that the call has been fully processed.
 */
void WhaleDetectionNode::try_process_pending_call() {
    // Nothing can be processed until a whale call has been received.
    if (!pending_call_available_) return;

    // Wait until every configured vehicle has published its state for the exact cycle associated with the pending whale call.    
    for (const std::string &vehicle_id : vehicle_ids_) {
        const auto iterator = latest_vehicle_states_.find(vehicle_id);
        
        // This vehicle has not published any state yet.
        if (iterator == latest_vehicle_states_.end()) return;
        
        // This vehicle has not yet reached the whale-call cycle.
        if (iterator->second.cycle < pending_call_.cycle) return;

        // Processing is no longer synchronized if a vehicle has already moved beyond the call cycle.
        if (iterator->second.cycle > pending_call_.cycle) {
            RCLCPP_ERROR(get_logger(), "Vehicle %s advanced to cycle %llu before call cycle %llu was processed.",
                vehicle_id.c_str(), static_cast<unsigned long long>(iterator->second.cycle), static_cast<unsigned long long>(pending_call_.cycle)
            );
            return;
        }
    }

    // Record which vehicles actually receive a bearing for this call.
    std::vector<std::string> bearing_receiver_ids;

    // Every vehicle is at the call cycle, so evaluate its detection distance.
    for (const std::string &vehicle_id : vehicle_ids_) {
        const auto &vehicle = latest_vehicle_states_.at(vehicle_id);

        // Calculate horizontal vehicle-to-whale distance.
        const double north_difference = pending_call_.north - vehicle.north;
        const double east_difference = pending_call_.east - vehicle.east;
        const double distance = std::hypot(north_difference, east_difference);

        // Ignore vehicles outside the configured acoustic detection range.
        if (!receive_all_calls_ && distance > detection_range_) continue;

        // Perfect bearing measurement: clockwise from north, in radians.
        const double delta_north = pending_call_.north - vehicle.north;
        const double delta_east = pending_call_.east - vehicle.east;

        // Direction is undefined when receiver and whale occupy the same position.
        if (std::hypot(delta_north, delta_east) > 1.0e-6) {
            acoustics_interfaces::msg::BearingObservation observation;
            observation.receiver_id = vehicle_id;
            observation.call_cycle = pending_call_.cycle;
            observation.sim_time = pending_call_.sim_time;
            observation.bearing_rad = std::atan2(delta_east, delta_north);
            observation.bearing_std_rad = 0.0;

            bearing_pubs_.at(vehicle_id)->publish(observation);
            bearing_receiver_ids.push_back(vehicle_id);
        }

        // Populate the detection with the exact vehicle and whale states used.
        acoustics_interfaces::msg::WhaleDetection detection;
        detection.vehicle_id = vehicle_id;
        detection.whale_id = pending_call_.whale_id;
        detection.cycle = pending_call_.cycle;
        detection.sim_time = pending_call_.sim_time;
        detection.vehicle_north = vehicle.north;
        detection.vehicle_east = vehicle.east;
        detection.whale_north = pending_call_.north;
        detection.whale_east = pending_call_.east;
        detection.distance = distance;

        // Add noise to the whale call
        if (position_std_m_ > 0.0) {
            detection.whale_north += position_std_m_ * normal_(rng_);
            detection.whale_east += position_std_m_ * normal_(rng_);
        }

        // Record the exact detection for later analysis and review.
        detection_log_ << detection.cycle << ',' << std::setprecision(17) << detection.sim_time << ',' << detection.vehicle_id << ',' << detection.whale_id << ',' << detection.vehicle_north << ',' << detection.vehicle_east << ',' << detection.whale_north << ',' << detection.whale_east << ',' << detection.distance << '\n';
    }

    // Acknowledge that every vehicle has evaluated the pending whale call.
    acoustics_interfaces::msg::WhaleCallProcessed processed;
    processed.whale_id = pending_call_.whale_id;
    processed.cycle = pending_call_.cycle;
    processed.sim_time = pending_call_.sim_time;
    processed.bearing_receiver_ids = bearing_receiver_ids;

    // Clear the pending state before publishing the acknowledgement.
    pending_call_available_ = false;
    processed_pub_->publish(processed);
}