#include "whale_estimator/whale_estimator_node.hpp"

void WhaleEstimatorNode::create_ros_interfaces() {
    const auto qos = rclcpp::QoS(100).reliable().durability_volatile();

    estimate_pub_ = create_publisher<whale_estimator_interfaces::msg::WhaleKalmanEstimate>("/" + vehicle_id_ + "/whale/kalman_estimate", qos);

    fix_sub_ = create_subscription<odyssey_interfaces::msg::WhaleBearingPositionEstimate>("/" + vehicle_id_ + "/whale/bearing_position_estimate", qos,[this](const odyssey_interfaces::msg::WhaleBearingPositionEstimate::SharedPtr msg) {
        state_.received_vehicle_id = msg->vehicle_id;
        state_.received_call_cycle = msg->call_cycle;
        state_.received_fix = whale_position_fix_t{msg->sim_time_s, msg->north_m, msg->east_m};
        handle_whale_bearing_position();
        log_state_if_changed();
    });

    shutdown_sub_ = create_subscription<std_msgs::msg::Bool>("/sim/shutdown", rclcpp::QoS(1).reliable().transient_local(), [this](const std_msgs::msg::Bool::SharedPtr msg) {
        state_.received_shutdown_b = msg->data;
        handle_shutdown();
        log_state_if_changed();
    });
}
