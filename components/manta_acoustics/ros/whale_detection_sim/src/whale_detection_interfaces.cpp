#include "whale_detection_sim/whale_detection_node.hpp"

void WhaleDetectionNode::create_publishers() {
    for (const std::string &vehicle_id : sim_state_.vehicle_ids) bearing_observation_pubs[vehicle_id] = create_publisher<acoustics_interfaces::msg::BearingObservation>("/" + vehicle_id + "/sensors/bearing", rclcpp::QoS(100).reliable());
    whale_call_processed_pub_ = create_publisher<acoustics_interfaces::msg::WhaleCallProcessed>("/sim/whales/call_processed", 100);
}

void WhaleDetectionNode::create_ros_interfaces() {
    create_publishers();
    create_subscribers();
}

void WhaleDetectionNode::create_subscribers() {
    for (const std::string &vehicle_id : sim_state_.vehicle_ids) {
        auto subscription = create_subscription<manta_interfaces::msg::VehicleState>("/" + vehicle_id + "/state", 10, [this, vehicle_id](const manta_interfaces::msg::VehicleState::SharedPtr message) {handle_vehicle_state(message, vehicle_id); log_state_if_changed();});
        vehicle_state_subs_.push_back(subscription);
    }
    whale_call_sub_ = create_subscription<whale_interfaces::msg::WhaleCall>("/sim/whales/calls", 100, [this](const whale_interfaces::msg::WhaleCall::SharedPtr call) {handle_whale_call(call); log_state_if_changed();});
    shutdown_sub_ = create_subscription<std_msgs::msg::Bool>("/sim/shutdown", 10, [this](const std_msgs::msg::Bool::SharedPtr message) {handle_shutdown(message); log_state_if_changed();});
}