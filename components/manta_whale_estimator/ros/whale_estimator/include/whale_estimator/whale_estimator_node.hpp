#ifndef MANTA_WHALE_ESTIMATOR_NODE_HPP
#define MANTA_WHALE_ESTIMATOR_NODE_HPP

#include <cstdint>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "odyssey_interfaces/msg/whale_bearing_position_estimate.hpp"
#include "whale_estimator_interfaces/msg/whale_kalman_estimate.hpp"
#include "manta_interfaces/state_log.hpp"
#include "std_msgs/msg/bool.hpp"

extern "C" {
#include "whale_estimator.h"
}

class WhaleEstimatorNode : public rclcpp::Node {
public:
    WhaleEstimatorNode();

private:
    void load_config();
    void handle_position_fix(const odyssey_interfaces::msg::WhaleBearingPositionEstimate::SharedPtr msg);
    void handle_shutdown(const std_msgs::msg::Bool::SharedPtr msg);

    whale_estimator_state_t state_{};
    manta::StateLog state_log_;
    std::string vehicle_id_;
    rclcpp::Subscription<odyssey_interfaces::msg::WhaleBearingPositionEstimate>::SharedPtr fix_sub_;
    rclcpp::Publisher<whale_estimator_interfaces::msg::WhaleKalmanEstimate>::SharedPtr estimate_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr shutdown_sub_;
};

#endif
