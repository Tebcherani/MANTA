#include "whale_detection_sim/whale_detection_node.hpp"

/**
 * Create publishers for whale detections and call-processing acknowledgements.
 */
void WhaleDetectionNode::create_publishers() {
    for (const std::string &vehicle_id : vehicle_ids_) {
        bearing_observation_pubs[vehicle_id] = create_publisher<acoustics_interfaces::msg::BearingObservation>("/" + vehicle_id + "/sensors/bearing", rclcpp::QoS(100).reliable());
    }
    whale_call_processed_pub_ = create_publisher<acoustics_interfaces::msg::WhaleCallProcessed>("/sim/whales/call_processed", 100);
}

/**
 * Create all publishers and subscribers used by the whale-detection node.
 */
void WhaleDetectionNode::create_ros_interfaces() {
    create_publishers();
    create_subscribers();
}

/**
 * Create subscriptions for vehicle states, whale calls, and simulation
 * shutdown requests.
 */
void WhaleDetectionNode::create_subscribers() {
    // Subscribe to the state published by every configured vehicle.
    for (const std::string &vehicle_id : vehicle_ids_) {
        const std::string topic = "/" + vehicle_id + "/state";

        auto subscription = create_subscription<manta_interfaces::msg::VehicleState>(
            topic, 10, [this, vehicle_id](const manta_interfaces::msg::VehicleState::SharedPtr message) {handle_vehicle_state(message, vehicle_id);}
        );

        vehicle_state_subs_.push_back(subscription);
    }

    // Receive whale calls that must be evaluated against every vehicle state.
    whale_call_sub_ = create_subscription<whale_interfaces::msg::WhaleCall>(
        "/sim/whales/calls", 100, [this](const whale_interfaces::msg::WhaleCall::SharedPtr call) {handle_whale_call(call);}
    );

    // Receive the orchestrator's request to stop the simulation.
    shutdown_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/sim/shutdown", 10, [this](const std_msgs::msg::Bool::SharedPtr message) {handle_shutdown(message);}
    );
}