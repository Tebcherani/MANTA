#ifndef MANTA_WHALE_ESTIMATOR_NODE_HPP
#define MANTA_WHALE_ESTIMATOR_NODE_HPP

#include <cstdint>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "odyssey_interfaces/msg/whale_bearing_position_estimate.hpp"
#include "whale_estimator_interfaces/msg/whale_kalman_estimate.hpp"
#include "whale_estimator/whale_estimator_state.hpp"
#include "manta_interfaces/state_log.hpp"
#include "std_msgs/msg/bool.hpp"

class WhaleEstimatorNode : public rclcpp::Node {
public:
    WhaleEstimatorNode();

private:
    void setup();
    void load_config();
    void create_ros_interfaces();
    void log_state_if_changed();
    double predict_whale_axis_position();
    double predict_whale_axis_velocity();
    int update_whale_axis();
    int update_estimator_state();
    void handle_whale_bearing_position();
    void handle_shutdown();

    whale_estimator_state_t state_{};
    manta::StateLog state_log_;
    std::string vehicle_id_;
    rclcpp::Subscription<odyssey_interfaces::msg::WhaleBearingPositionEstimate>::SharedPtr fix_sub_;
    rclcpp::Publisher<whale_estimator_interfaces::msg::WhaleKalmanEstimate>::SharedPtr estimate_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr shutdown_sub_;
};

#endif
