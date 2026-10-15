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
 * @param msg Shutdown request published by the simulation orchestrator.
 */
void WhaleDetectionNode::handle_shutdown(const std_msgs::msg::Bool::SharedPtr msg) {
    const bool shutdown_requested = msg->data;
    if (!shutdown_requested) return;                              
    RCLCPP_INFO(get_logger(), "Shutting down whale detector.");
    rclcpp::shutdown();
}

/**
 * Store a vehicle's latest state and attempt to process a pending whale call.
 *
 * @param msg Latest state published by the vehicle.
 * @param vehicle_id Configured identifier of the vehicle that published the state.
 */
void WhaleDetectionNode::handle_vehicle_state(const manta_interfaces::msg::VehicleState::SharedPtr msg, const std::string &vehicle_id) {
    latest_vehicle_states_[vehicle_id] = *msg;  // Store or replace this vehicle's most recently received state.
    try_process_pending_whale_call();                 
}

/**
 * Store a newly received whale call and attempt to process it.
 *
 * @param msg Whale-call event published by the whale simulator.
 */
void WhaleDetectionNode::handle_whale_call(const whale_interfaces::msg::WhaleCall::SharedPtr msg) {
    // Reject a new call if the previous call has not yet been evaluated. This indicates a synchronization error.
    if (pending_whale_call_available_b_) {
        RCLCPP_ERROR(get_logger(), "Received whale call %s before the previous call was processed.", msg->whale_id.c_str());
        return;
    }

    // Store the call until every configured vehicle reaches its event cycle.
    pending_whale_call_ = *msg;
    pending_whale_call_available_b_ = true;

    // Process immediately if all required vehicle states are already available.
    try_process_pending_whale_call();
}

//----------------------------------------------------
// Helper functions
//----------------------------------------------------

/**
 * Process the pending whale call once every vehicle has published its state for the call's simulation cycle.
 *
 * The method waits until all configured vehicle states exactly match the pending call cycle. It then calculates the two-dimensional distance from
 * each vehicle to the whale, publishes and records detections within range, and acknowledges that the call has been fully processed.
 */
void WhaleDetectionNode::try_process_pending_whale_call() {
    // Nothing can be processed until a whale call has been received.
    if (!pending_whale_call_available_b_) return;

    // Wait until every configured vehicle has published its state for the exact cycle associated with the pending whale call.   
    for (const std::string &vehicle_id : vehicle_ids_) {                    // Loop through every vehicle ID
        const auto vehicle_state = latest_vehicle_states_.find(vehicle_id); // Loop up the vehicle state of the vehicle with this vehicle ID

        if (vehicle_state == latest_vehicle_states_.end()) return;              // This vehicle has not published any state yet.
        if (vehicle_state->second.cycle < pending_whale_call_.cycle) return;    // This vehicle has not yet reached the whale-call cycle.
        if (vehicle_state->second.cycle > pending_whale_call_.cycle) {          // Processing is no longer synchronized if a vehicle has already moved beyond the call cycle.
            RCLCPP_ERROR(get_logger(), "Vehicle %s advanced to cycle %llu before call cycle %llu was processed.", vehicle_id.c_str(), static_cast<unsigned long long>(vehicle_state->second.cycle), static_cast<unsigned long long>(pending_whale_call_.cycle));
            return;
        }
    }

    // Record which vehicles actually receive a bearing for this call.
    std::vector<std::string> bearing_receiver_ids;

    // Every vehicle is at the call cycle, so evaluate its detection distance.
    for (const std::string &vehicle_id : vehicle_ids_) {                    // Loop through every vehicle ID
        const auto &vehicle_state = latest_vehicle_states_.at(vehicle_id);  // Loop up the vehicle state of the vehicle with this vehicle ID

        // Calculate two-dimensional vehicle-to-whale distance.
        const double vehicle_to_whale_north_distance_m = pending_whale_call_.whale_call_north_m - vehicle_state.vehicle_north_m;
        const double vehicle_to_whale_east_distance_m = pending_whale_call_.whale_call_east_m - vehicle_state.vehicle_east_m;
        const double distance = std::hypot(vehicle_to_whale_north_distance_m, vehicle_to_whale_east_distance_m);

        // Ignore vehicles outside the configured acoustic detection range.
        if (distance > detection_range_m_) continue;

        // A bearing is undefined at zero separation.
        if (distance <= 1.0e-6) continue;

        // Direction is undefined when receiver and whale occupy the same position.
        if (distance > 1.0e-6) {
            acoustics_interfaces::msg::BearingObservation observation;
            observation.receiver_id = vehicle_id;
            observation.call_cycle = pending_whale_call_.cycle;
            observation.sim_time_s = pending_whale_call_.sim_time_s;
            
            // Add an independent Gaussian angular error to this receiver's bearing.
            const double true_bearing = std::atan2(vehicle_to_whale_east_distance_m, vehicle_to_whale_north_distance_m);
            const double bearing_error = bearing_std_rad_ > 0.0 ? bearing_std_rad_ * normal_(rng_) : 0.0;
            const double measured_bearing = true_bearing + bearing_error;

            // Wrap the measured angle to the interval [-pi, pi].
            observation.bearing_rad = std::atan2(std::sin(measured_bearing), std::cos(measured_bearing));
            observation.bearing_std_rad = bearing_std_rad_;
                        
            bearing_observation_pubs.at(vehicle_id)->publish(observation);
            bearing_receiver_ids.push_back(vehicle_id);
        }

        // Populate the detection with the exact vehicle and whale states used.
        acoustics_interfaces::msg::WhaleDetection detection;
        detection.vehicle_id = vehicle_id;
        detection.whale_id = pending_whale_call_.whale_id;
        detection.cycle = pending_whale_call_.cycle;
        detection.sim_time_s = pending_whale_call_.sim_time_s;
        detection.vehicle_north_m = vehicle_state.vehicle_north_m;
        detection.vehicle_east_m = vehicle_state.vehicle_east_m;
        detection.whale_north_m = pending_whale_call_.whale_call_north_m;
        detection.whale_east_m = pending_whale_call_.whale_call_east_m;
        detection.vehicle_to_whale_range_m = distance;

        // Record the exact detection for later analysis and review.
        whale_detection_log_ << detection.cycle << ',' << std::setprecision(17) << detection.sim_time_s << ',' << detection.vehicle_id << ',' << detection.whale_id << ',' << detection.vehicle_north_m << ',' << detection.vehicle_east_m << ',' << detection.whale_north_m << ',' << detection.whale_east_m << ',' << detection.vehicle_to_whale_range_m << '\n';
    }

    // Acknowledge that every vehicle has evaluated the pending whale call.
    acoustics_interfaces::msg::WhaleCallProcessed processed;
    processed.whale_id = pending_whale_call_.whale_id;
    processed.cycle = pending_whale_call_.cycle;
    processed.sim_time_s = pending_whale_call_.sim_time_s;
    processed.bearing_receiver_ids = bearing_receiver_ids;

    // Clear the pending state before publishing the acknowledgement.
    pending_whale_call_available_b_ = false;
    whale_call_processed_pub_->publish(processed);
}