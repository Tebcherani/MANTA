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
 * Reads the latched shutdown request from sim_state_.shutdown_requested_b.
 */
void WhaleDetectionNode::handle_shutdown() {
    if (!sim_state_.shutdown_requested_b) return;
    RCLCPP_INFO(get_logger(), "Shutting down whale detector.");
    rclcpp::shutdown();
}

/**
 * Validate the incoming whale call stored by the subscription callback.
 *
 * Reads sim_state_.received_whale_call. An existing pending call is preserved
 * and the incoming event rejected if processing has not yet completed.
 */
void WhaleDetectionNode::handle_whale_call() {
    if (sim_state_.pending_whale_call_available_b) {
        RCLCPP_ERROR(get_logger(), "Received whale call %s before the previous call was processed.", sim_state_.received_whale_call.whale_id.c_str());
        return;
    }

    sim_state_.pending_whale_call = sim_state_.received_whale_call;
    sim_state_.pending_whale_call_available_b = true;
    try_process_pending_whale_call();
}

//----------------------------------------------------
// Helper functions
//----------------------------------------------------

/**
 * Calculate horizontal range for the receiver selected in sim_state_.
 *
 * Reads current_vehicle_id, latest_vehicle_states, and pending_whale_call.
 * Stores vehicle_to_whale_range_m in metres; depth is excluded.
 */
void WhaleDetectionNode::calculate_range() {
    const auto &vehicle_state = sim_state_.latest_vehicle_states.at(sim_state_.current_vehicle_id);
    const auto &whale_call = sim_state_.pending_whale_call;

    const double north_offset_m = whale_call.whale_call_north_m - vehicle_state.vehicle_north_m;
    const double east_offset_m = whale_call.whale_call_east_m - vehicle_state.vehicle_east_m;

    sim_state_.vehicle_to_whale_range_m = std::hypot(north_offset_m, east_offset_m);
}

/**
 * Build the current receiver's complete bearing observation in sim_state_.
 *
 * Requires a selected receiver with nonzero range. Reads pending_whale_call,
 * the receiver's stored position, bearing uncertainty, and the noise generator.
 * Stores bearing_observation and marks it available. Angle is in [-pi, pi].
 */
void WhaleDetectionNode::calculate_bearing() {
    const auto &vehicle_state = sim_state_.latest_vehicle_states.at(sim_state_.current_vehicle_id);
    const auto &whale_call = sim_state_.pending_whale_call;

    const double north_offset_m = whale_call.whale_call_north_m - vehicle_state.vehicle_north_m;
    const double east_offset_m = whale_call.whale_call_east_m - vehicle_state.vehicle_east_m;

    const double true_bearing_rad = std::atan2(east_offset_m, north_offset_m);
    const double bearing_error_rad = sim_state_.bearing_std_rad > 0.0 ? sim_state_.bearing_std_rad * sim_state_.normal(sim_state_.rng) : 0.0;
    const double measured_bearing_rad = true_bearing_rad + bearing_error_rad;

    auto &observation = sim_state_.bearing_observation;
    observation = acoustics_interfaces::msg::BearingObservation{};
    observation.receiver_id = sim_state_.current_vehicle_id;
    observation.call_cycle = whale_call.cycle;
    observation.sim_time_s = whale_call.sim_time_s;
    observation.bearing_rad = std::atan2(std::sin(measured_bearing_rad), std::cos(measured_bearing_rad));
    observation.bearing_std_rad = sim_state_.bearing_std_rad;
    sim_state_.bearing_observation_available_b = true;
}

/**
 * Store and log the current receiver's detection before selecting another one.
 *
 * Reads current_vehicle_id, its latest vehicle state, pending_whale_call, and
 * vehicle_to_whale_range_m. Sets whale_detection_available_b and writes the
 * per-receiver state snapshot and detection CSV row.
 */
void WhaleDetectionNode::record_detection() {
    const auto &vehicle_state = sim_state_.latest_vehicle_states.at(sim_state_.current_vehicle_id);
    const auto &whale_call = sim_state_.pending_whale_call;

    auto &whale_detection = sim_state_.whale_detection;
    whale_detection = acoustics_interfaces::msg::WhaleDetection{};
    whale_detection.vehicle_id = sim_state_.current_vehicle_id;
    whale_detection.whale_id = whale_call.whale_id;
    whale_detection.cycle = whale_call.cycle;
    whale_detection.sim_time_s = whale_call.sim_time_s;
    whale_detection.vehicle_north_m = vehicle_state.vehicle_north_m;
    whale_detection.vehicle_east_m = vehicle_state.vehicle_east_m;
    whale_detection.whale_north_m = whale_call.whale_call_north_m;
    whale_detection.whale_east_m = whale_call.whale_call_east_m;
    whale_detection.vehicle_to_whale_range_m = sim_state_.vehicle_to_whale_range_m;
    sim_state_.whale_detection_available_b = true;

    log_state_if_changed();
    whale_detection_log_ << whale_detection.cycle << ',' << std::setprecision(17) << whale_detection.sim_time_s << ',' << whale_detection.vehicle_id << ',' << whale_detection.whale_id << ',' << whale_detection.vehicle_north_m << ',' << whale_detection.vehicle_east_m << ',' << whale_detection.whale_north_m << ',' << whale_detection.whale_east_m << ',' << whale_detection.vehicle_to_whale_range_m << '\n';
}

/**
 * Check that every configured vehicle is synchronized with the pending call.
 *
 * @return True when every latest vehicle state matches the pending call cycle;
 *         false if a state is missing, behind, or ahead. Ahead states log an error.
 */
bool WhaleDetectionNode::vehicle_states_match_call_cycle() const {
    for (const std::string &vehicle_id : sim_state_.vehicle_ids) {                    // Loop through every vehicle ID
        const auto vehicle_state = sim_state_.latest_vehicle_states.find(vehicle_id); // Loop up the vehicle state of the vehicle with this vehicle ID

        if (vehicle_state == sim_state_.latest_vehicle_states.end()) return false;              // This vehicle has not published any state yet.
        if (vehicle_state->second.cycle < sim_state_.pending_whale_call.cycle) return false;    // This vehicle has not yet reached the whale-call cycle.
        if (vehicle_state->second.cycle > sim_state_.pending_whale_call.cycle) {          // Processing is no longer synchronized if a vehicle has already moved beyond the call cycle.
            RCLCPP_ERROR(get_logger(), "Vehicle %s advanced to cycle %llu before call cycle %llu was processed.", vehicle_id.c_str(), static_cast<unsigned long long>(vehicle_state->second.cycle), static_cast<unsigned long long>(sim_state_.pending_whale_call.cycle));
            return false;
        }
    }

    return true;
}

/**
 * Complete the pending call, including calls with no bearing receivers.
 *
 * Uses sim_state_.bearing_receiver_ids; an empty list acknowledges no detections.
 */
void WhaleDetectionNode::acknowledge_whale_call() {
    auto &whale_call_processed_b = sim_state_.whale_call_processed;
    whale_call_processed_b = acoustics_interfaces::msg::WhaleCallProcessed{};
    whale_call_processed_b.whale_id = sim_state_.pending_whale_call.whale_id;
    whale_call_processed_b.cycle = sim_state_.pending_whale_call.cycle;
    whale_call_processed_b.sim_time_s = sim_state_.pending_whale_call.sim_time_s;
    whale_call_processed_b.bearing_receiver_ids = sim_state_.bearing_receiver_ids;

    sim_state_.pending_whale_call_available_b = false;
    whale_call_processed_pub_->publish(whale_call_processed_b);
}

/**
 * Process the pending whale call once every vehicle has published its state for the call's simulation cycle.
 *
 * The method waits until all configured vehicle states exactly match the pending call cycle. It then calculates the two-dimensional distance from
 * each vehicle to the whale, publishes and records detections within range, and acknowledges that the call has been fully processed.
 */
void WhaleDetectionNode::try_process_pending_whale_call() {
    // Nothing can be processed until a whale call has been received.
    if (!sim_state_.pending_whale_call_available_b) return;

    if (!vehicle_states_match_call_cycle()) return;

    // The retained detection is valid only if this call produces a detection.
    sim_state_.whale_detection_available_b = false;

    // Record which vehicles actually receive a bearing for this call.
    sim_state_.bearing_receiver_ids.clear();

    // Helpers read the receiver context selected here; do not reorder these steps.
    for (const std::string &vehicle_id : sim_state_.vehicle_ids) {
        sim_state_.current_vehicle_id = vehicle_id;
        sim_state_.bearing_observation_available_b = false;
        sim_state_.bearing_observation = acoustics_interfaces::msg::BearingObservation{};
        calculate_range();

        if (sim_state_.vehicle_to_whale_range_m > sim_state_.detection_range_m) continue;
        if (sim_state_.vehicle_to_whale_range_m <= 1.0e-6) continue;

        calculate_bearing();
        bearing_observation_pubs.at(sim_state_.current_vehicle_id)->publish(sim_state_.bearing_observation);
        sim_state_.bearing_receiver_ids.push_back(sim_state_.current_vehicle_id);
        record_detection();
    }

    acknowledge_whale_call();
}
