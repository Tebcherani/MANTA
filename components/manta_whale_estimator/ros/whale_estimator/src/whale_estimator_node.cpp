#include "whale_estimator/whale_estimator_node.hpp"
#include "whale_estimator/whale_estimator_state_json.hpp"

#include <cmath>
#include <memory>
#include <stdexcept>

WhaleEstimatorNode::WhaleEstimatorNode() : rclcpp::Node("whale_estimator_node") {
    whale_estimator_init(&state_);
    load_config();
    state_log_.open_in_log_directory(vehicle_id_ + "_whale_estimator_state.jsonl");
    state_log_.record(state_);
    const auto qos = rclcpp::QoS(100).reliable().durability_volatile();
    estimate_pub_ = create_publisher<whale_estimator_interfaces::msg::WhaleKalmanEstimate>(
        "/" + vehicle_id_ + "/whale/kalman_estimate", qos);
    fix_sub_ = create_subscription<odyssey_interfaces::msg::WhaleBearingPositionEstimate>(
        "/" + vehicle_id_ + "/whale/bearing_position_estimate", qos,
        [this](const odyssey_interfaces::msg::WhaleBearingPositionEstimate::SharedPtr msg) { handle_position_fix(msg); });
    shutdown_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/sim/shutdown", rclcpp::QoS(1).reliable().transient_local(),
        [this](const std_msgs::msg::Bool::SharedPtr msg) { handle_shutdown(msg); });
}

void WhaleEstimatorNode::handle_shutdown(const std_msgs::msg::Bool::SharedPtr msg) {
    if (!msg->data) return;
    state_.shutdown_requested_b = 1;
    state_log_.record(state_);
    rclcpp::shutdown();
}

void WhaleEstimatorNode::load_config() {
    vehicle_id_ = declare_parameter<std::string>("vehicle_id", "");
    if (vehicle_id_.empty()) throw std::runtime_error("vehicle_id must be set.");
    auto &config = state_.config;
    config.velocity_window_max_age_s = declare_parameter<double>("velocity_window_max_age_s", config.velocity_window_max_age_s);
    config.acceleration_psd_m2_s3 = declare_parameter<double>("acceleration_psd_m2_s3", config.acceleration_psd_m2_s3);
    config.velocity_return_time_s = declare_parameter<double>("whale_velocity_return_time_s", config.velocity_return_time_s);
    config.nominal_north_velocity_mps = declare_parameter<double>("whale_baseline_velocity_north_mps", config.nominal_north_velocity_mps);
    config.nominal_east_velocity_mps = declare_parameter<double>("whale_baseline_velocity_east_mps", config.nominal_east_velocity_mps);
    config.initial_velocity_variance_m2ps2 = declare_parameter<double>("initial_velocity_variance_m2ps2", config.initial_velocity_variance_m2ps2);
    if (!std::isfinite(config.velocity_window_max_age_s) || config.velocity_window_max_age_s <= 0.0 ||
        !std::isfinite(config.acceleration_psd_m2_s3) || config.acceleration_psd_m2_s3 < 0.0 ||
        !std::isfinite(config.velocity_return_time_s) || config.velocity_return_time_s <= 0.0 ||
        !std::isfinite(config.nominal_north_velocity_mps) || !std::isfinite(config.nominal_east_velocity_mps) ||
        !std::isfinite(config.initial_velocity_variance_m2ps2) || config.initial_velocity_variance_m2ps2 < 0.0) {
        throw std::runtime_error("Invalid whale estimator configuration.");
    }
}

void WhaleEstimatorNode::handle_position_fix(const odyssey_interfaces::msg::WhaleBearingPositionEstimate::SharedPtr msg) {
    if (msg->vehicle_id != vehicle_id_) return;
    state_.received_fix = whale_position_fix_t{msg->sim_time_s, msg->north_m, msg->east_m};
    const bool accepted = whale_estimator_update(&state_);
    state_log_.record(state_);
    if (!accepted) return;
    whale_estimator_interfaces::msg::WhaleKalmanEstimate estimate;
    estimate.vehicle_id = vehicle_id_;
    estimate.call_cycle = msg->call_cycle;
    estimate.measurement_time_s = state_.measurement_time_s;
    estimate.whale_kalman_north_m = state_.north.position;
    estimate.whale_kalman_east_m = state_.east.position;
    estimate.whale_kalman_velocity_north_mps = state_.north.velocity;
    estimate.whale_kalman_velocity_east_mps = state_.east.velocity;
    estimate.whale_kalman_velocity_variance_north_m2ps2 = state_.north.velocity_variance;
    estimate.whale_kalman_velocity_variance_east_m2ps2 = state_.east.velocity_variance;
    estimate.whale_baseline_velocity_north_mps = state_.config.nominal_north_velocity_mps;
    estimate.whale_baseline_velocity_east_mps = state_.config.nominal_east_velocity_mps;
    estimate.whale_velocity_return_time_s = state_.config.velocity_return_time_s;
    estimate_pub_->publish(estimate);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WhaleEstimatorNode>());
    rclcpp::shutdown();
    return 0;
}
