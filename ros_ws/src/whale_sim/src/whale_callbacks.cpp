#include "whale_sim/whale_node.hpp"

/**
 * Find the first scheduled whale call after a requested simulation time.
 *
 * Calls at or before `request.after_time` have already been processed and are discarded. The earliest remaining call across every whale is returned with
 * the whale's position at that exact simulated time.
 *
 * @param request Request containing the last processed simulation time.
 * @param response Response populated with the next available whale call.
 */
void WhaleNode::handle_get_next_call(const manta_interfaces::srv::GetNextWhaleCall::Request &request, manta_interfaces::srv::GetNextWhaleCall::Response &response) {
    // No whale has been selected yet.
    Whale *next_whale = nullptr;
    double next_time = std::numeric_limits<double>::infinity();

    // Search every whale for its earliest unprocessed call.
    for (auto &whale : whales_) {
        // Discard calls that the orchestrator has already processed.
        while (whale.next_call_index < whale.call_times.size() && whale.call_times[whale.next_call_index] <= request.after_time) {
            ++whale.next_call_index;
        }

        // Skip whales that have no scheduled calls remaining.
        if (whale.next_call_index >= whale.call_times.size()) continue;
        const double candidate_time = whale.call_times[whale.next_call_index];

        // Retain the earliest call found across all whales.
        if (candidate_time < next_time) {
            next_time = candidate_time;
            next_whale = &whale;
        }
    }

    // Report that no whale calls remain in the simulation.
    if (next_whale == nullptr) {
        response.available = false;
        return;
    }

    // Identify the next call and its scheduled simulation time.
    response.available = true;
    response.call.whale_id = next_whale->id;
    response.call.cycle = 0;
    response.call.sim_time = next_time;

    // Calculate the whale's location at the exact time of the call.
    const TrajectoryPoint call_position = trajectory_at_time(*next_whale, next_time);

    response.call.north = call_position.north;
    response.call.east = call_position.east;
    response.call.depth = next_whale->depth;
}

/**
 * Shut down the whale-simulation node when requested by the orchestrator.
 *
 * Messages containing `false` are ignored. A message containing `true` stops the shared ROS context so the executor can exit normally.
 *
 * @param message Shutdown request published by the simulation orchestrator.
 */
void WhaleNode::handle_shutdown(const std_msgs::msg::Bool::SharedPtr message) {
    if (!message->data) return;                             // Ignore messages that do not request shutdown.
    RCLCPP_INFO(get_logger(), "Shutting down whale node."); // Report the shutdown before stopping the ROS context.
    rclcpp::shutdown();                                     // Stop the ROS executor and allow the process to exit normally.
}

/**
 * Handle a simulation-step message from the orchestrator.
 *
 * The callback passes the received simulation batch to the whale-update routine, which advances every whale to the end of the batch, publishes
 * ground-truth states, and records any trajectory samples crossed by it.
 *
 * @param step Simulation batch containing the current time and number of steps.
 */
void WhaleNode::handle_sim_step(const manta_interfaces::msg::SimStep::SharedPtr step) {
    update_whales(*step);   // Advance and publish the whales using the received simulation batch.
}