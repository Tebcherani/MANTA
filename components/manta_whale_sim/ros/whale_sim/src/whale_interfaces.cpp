#include "whale_sim/whale_node.hpp"

/** Create the publishers and subscribers used by the whale node. */
void WhaleNode::create_ros_interfaces() {
    create_publishers();
    create_subscribers();
}

void WhaleNode::create_publishers() {
    whale_states_pub_ = create_publisher<whale_interfaces::msg::WhaleStateArray>("/sim/whales/states", 10);
    whale_call_pub_ = create_publisher<whale_interfaces::msg::WhaleCall>("/sim/whales/calls", rclcpp::QoS(100).reliable());
    step_result_pub_ = create_publisher<whale_interfaces::msg::WhaleStepResult>("/sim/whales/step_result", rclcpp::QoS(1).reliable());
    ready_pub_ = create_publisher<std_msgs::msg::Bool>("/sim/whales/ready", rclcpp::QoS(1).reliable().transient_local());
}

/** Create subscriptions for simulation steps and shutdown requests. */
void WhaleNode::create_subscribers() {
    sim_step_sub_ = create_subscription<manta_interfaces::msg::SimStep>(
        "/sim/whales/step", rclcpp::QoS(1).reliable(),
        [this](const manta_interfaces::msg::SimStep::SharedPtr step) {
            sim_state_.received_sim_step = *step;
            handle_sim_step();
            log_state_if_changed();
        }
    );

    shutdown_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/sim/shutdown", 10, [this](const std_msgs::msg::Bool::SharedPtr message) {
            if (message->data) sim_state_.shutdown_requested_b = true;
            handle_shutdown();
            log_state_if_changed();
        }
    );
}
