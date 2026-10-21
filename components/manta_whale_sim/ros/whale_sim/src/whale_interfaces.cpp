#include "whale_sim/whale_node.hpp"

#include <memory>

/**
 * Create all publishers, subscribers, and services used by the whale node.
 */
void WhaleNode::create_ros_interfaces() {
    create_publishers();
    create_subscribers();
    create_services();
}

void WhaleNode::create_publishers() {
    whale_states_pub_ = create_publisher<whale_interfaces::msg::WhaleStateArray>("/sim/whales/states", 10);
}

/**
 * Create subscriptions for simulation steps and shutdown requests.
 */
void WhaleNode::create_subscribers() {
    // Receive simulation batches from the orchestrator.
    sim_step_sub_ = create_subscription<manta_interfaces::msg::SimStep>(
        "/sim/step", 10, [this](const manta_interfaces::msg::SimStep::SharedPtr step) {
            sim_state_.pending_sim_step = {step->current_cycle, step->sim_time_s, step->timestep, step->step_count};
            update_whales();
            log_state_if_changed();
        }
    );

    // Receive the orchestrator's request to stop the simulation.
    shutdown_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/sim/shutdown", 10, [this](const std_msgs::msg::Bool::SharedPtr message) {
            if (message->data) sim_state_.shutdown_requested_b = true;
            handle_shutdown();
            log_state_if_changed();
        }
    );
}

/**
 * Create services provided by the whale node.
 */
void WhaleNode::create_services() {
    // Return the first scheduled whale call after a requested simulation time.
    next_whale_call_service_ = create_service<whale_interfaces::srv::GetNextWhaleCall>("/sim/whales/get_next_call", [this](
        const std::shared_ptr<whale_interfaces::srv::GetNextWhaleCall::Request> request,
        std::shared_ptr<whale_interfaces::srv::GetNextWhaleCall::Response> response) {
        sim_state_.whale_call_after_time_s = request->whale_call_after_time_s;
        handle_get_next_call();
        response->whale_call_available_b = sim_state_.next_whale_call_available_b;
        response->whale_call = sim_state_.next_whale_call;
        log_state_if_changed();
    });
}
