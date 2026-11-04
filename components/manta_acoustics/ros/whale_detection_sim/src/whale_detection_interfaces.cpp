#include "whale_detection_sim/whale_detection_node.hpp"

void WhaleDetectionNode::create_publishers() {
    for (const std::string &vehicle_id : sim_state_.vehicle_ids) bearing_observation_pubs[vehicle_id] = create_publisher<acoustics_interfaces::msg::BearingObservation>("/" + vehicle_id + "/sensors/bearing", rclcpp::QoS(100).reliable());
    whale_call_processed_pub_ = create_publisher<acoustics_interfaces::msg::WhaleCallProcessed>("/sim/whales/call_processed", 100);
    ready_pub_ = create_publisher<std_msgs::msg::Bool>("/sim/whales/detector_ready", rclcpp::QoS(1).reliable().transient_local());
}

void WhaleDetectionNode::create_ros_interfaces() {
    create_publishers();
    create_subscribers();
}

void WhaleDetectionNode::create_subscribers() {
    for (const std::string &vehicle_id : sim_state_.vehicle_ids) {
        auto subscription = create_subscription<manta_interfaces::msg::VehicleState>("/" + vehicle_id + "/state", 10, [this, vehicle_id](const manta_interfaces::msg::VehicleState::SharedPtr message) {
            sim_state_.latest_vehicle_states[vehicle_id] = *message;
            try_process_pending_whale_call();
            log_state_if_changed();
        });
        vehicle_state_subs_.push_back(subscription);
    }
    whale_call_sub_ = create_subscription<whale_interfaces::msg::WhaleCall>("/sim/whales/calls", 100, [this](const whale_interfaces::msg::WhaleCall::SharedPtr call) {
        sim_state_.received_whale_call = *call;
        handle_whale_call();
        log_state_if_changed();
    });
    shutdown_sub_ = create_subscription<std_msgs::msg::Bool>("/sim/shutdown", 10, [this](const std_msgs::msg::Bool::SharedPtr message) {
        if (message->data) sim_state_.shutdown_requested_b = true;
        handle_shutdown();
        log_state_if_changed();
    });
}
