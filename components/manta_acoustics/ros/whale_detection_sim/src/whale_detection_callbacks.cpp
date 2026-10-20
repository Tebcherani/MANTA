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
    if (!msg->data) return;
    sim_state_.shutdown_requested_b = true;
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
    sim_state_.latest_vehicle_states[vehicle_id] = *msg;  // Store or replace this vehicle's most recently received state.
    try_process_pending_whale_call();                 
}

/**
 * Store a newly received whale call and attempt to process it.
 *
 * @param msg Whale-call event published by the whale simulator.
 */
void WhaleDetectionNode::handle_whale_call(const whale_interfaces::msg::WhaleCall::SharedPtr msg) {
    // Reject a new call if the previous call has not yet been evaluated. This indicates a synchronization error.
    if (sim_state_.pending_whale_call_available_b) {
        RCLCPP_ERROR(get_logger(), "Received whale call %s before the previous call was processed.", msg->whale_id.c_str());
        return;
    }

    // Store the call until every configured vehicle reaches its event cycle.
    sim_state_.pending_whale_call = *msg;
    sim_state_.pending_whale_call_available_b = true;

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
    if (!sim_state_.pending_whale_call_available_b) return;

    // Wait until every configured vehicle has published its state for the exact cycle associated with the pending whale call.   
    for (const std::string &vehicle_id : sim_state_.vehicle_ids) {                    // Loop through every vehicle ID
        const auto vehicle_state = sim_state_.latest_vehicle_states.find(vehicle_id); // Loop up the vehicle state of the vehicle with this vehicle ID

        if (vehicle_state == sim_state_.latest_vehicle_states.end()) return;              // This vehicle has not published any state yet.
        if (vehicle_state->second.cycle < sim_state_.pending_whale_call.cycle) return;    // This vehicle has not yet reached the whale-call cycle.
        if (vehicle_state->second.cycle > sim_state_.pending_whale_call.cycle) {          // Processing is no longer synchronized if a vehicle has already moved beyond the call cycle.
            RCLCPP_ERROR(get_logger(), "Vehicle %s advanced to cycle %llu before call cycle %llu was processed.", vehicle_id.c_str(), static_cast<unsigned long long>(vehicle_state->second.cycle), static_cast<unsigned long long>(sim_state_.pending_whale_call.cycle));
            return;
        }
    }

    // The retained detection is valid only if this call produces a detection.
    sim_state_.whale_detection_available_b = false;

    // Record which vehicles actually receive a bearing for this call.
    std::vector<std::string> bearing_receiver_ids;

    // Every vehicle is at the call cycle, so evaluate its detection distance.
    for (const std::string &vehicle_id : sim_state_.vehicle_ids) {                    // Loop through every vehicle ID
        const auto &vehicle_state = sim_state_.latest_vehicle_states.at(vehicle_id);  // Loop up the vehicle state of the vehicle with this vehicle ID

        // Calculate two-dimensional vehicle-to-whale distance.
        const double vehicle_to_whale_north_distance_m = sim_state_.pending_whale_call.whale_call_north_m - vehicle_state.vehicle_north_m;
        const double vehicle_to_whale_east_distance_m = sim_state_.pending_whale_call.whale_call_east_m - vehicle_state.vehicle_east_m;
        const double vehicle_to_whale_range_m = std::hypot(vehicle_to_whale_north_distance_m, vehicle_to_whale_east_distance_m);

        // Ignore vehicles outside the configured acoustic detection range.
        if (vehicle_to_whale_range_m > sim_state_.detection_range_m) continue;

        // A bearing is undefined at zero separation.
        if (vehicle_to_whale_range_m <= 1.0e-6) continue;

        // Direction is undefined when receiver and whale occupy the same position.
        if (vehicle_to_whale_range_m > 1.0e-6) {
            acoustics_interfaces::msg::BearingObservation bearing_observation;
            bearing_observation.receiver_id = vehicle_id;
            bearing_observation.call_cycle = sim_state_.pending_whale_call.cycle;
            bearing_observation.sim_time_s = sim_state_.pending_whale_call.sim_time_s;
            
            // Add an independent Gaussian angular error to this receiver's bearing.
            const double true_bearing_rad = std::atan2(vehicle_to_whale_east_distance_m, vehicle_to_whale_north_distance_m);
            const double bearing_error_rad = sim_state_.bearing_std_rad > 0.0 ? sim_state_.bearing_std_rad * sim_state_.normal(sim_state_.rng) : 0.0;
            const double measured_bearing_rad = true_bearing_rad + bearing_error_rad;

            // Wrap the measured angle to the interval [-pi, pi].
            bearing_observation.bearing_rad = std::atan2(std::sin(measured_bearing_rad), std::cos(measured_bearing_rad));
            bearing_observation.bearing_std_rad = sim_state_.bearing_std_rad;
                        
            bearing_observation_pubs.at(vehicle_id)->publish(bearing_observation);
            bearing_receiver_ids.push_back(vehicle_id);
        }

        // Populate the detection with the exact vehicle and whale states used.
        auto &whale_detection = sim_state_.whale_detection;
        whale_detection = acoustics_interfaces::msg::WhaleDetection{};
        whale_detection.vehicle_id = vehicle_id;
        whale_detection.whale_id = sim_state_.pending_whale_call.whale_id;
        whale_detection.cycle = sim_state_.pending_whale_call.cycle;
        whale_detection.sim_time_s = sim_state_.pending_whale_call.sim_time_s;
        whale_detection.vehicle_north_m = vehicle_state.vehicle_north_m;
        whale_detection.vehicle_east_m = vehicle_state.vehicle_east_m;
        whale_detection.whale_north_m = sim_state_.pending_whale_call.whale_call_north_m;
        whale_detection.whale_east_m = sim_state_.pending_whale_call.whale_call_east_m;
        whale_detection.vehicle_to_whale_range_m = vehicle_to_whale_range_m;
        sim_state_.whale_detection_available_b = true;

        // Record the exact detection for later analysis and review.
        whale_detection_log_ << whale_detection.cycle << ',' << std::setprecision(17) << whale_detection.sim_time_s << ',' << whale_detection.vehicle_id << ',' << whale_detection.whale_id << ',' << whale_detection.vehicle_north_m << ',' << whale_detection.vehicle_east_m << ',' << whale_detection.whale_north_m << ',' << whale_detection.whale_east_m << ',' << whale_detection.vehicle_to_whale_range_m << '\n';
    }

    // Acknowledge that every vehicle has evaluated the pending whale call.
    acoustics_interfaces::msg::WhaleCallProcessed whale_call_processed_b;
    whale_call_processed_b.whale_id = sim_state_.pending_whale_call.whale_id;
    whale_call_processed_b.cycle = sim_state_.pending_whale_call.cycle;
    whale_call_processed_b.sim_time_s = sim_state_.pending_whale_call.sim_time_s;
    whale_call_processed_b.bearing_receiver_ids = bearing_receiver_ids;

    // Clear the pending state before publishing the acknowledgement.
    sim_state_.pending_whale_call_available_b = false;
    whale_call_processed_pub_->publish(whale_call_processed_b);
}