#ifndef WHALE_DETECTION_SIM__WHALE_DETECTION_NODE_HPP_
#define WHALE_DETECTION_SIM__WHALE_DETECTION_NODE_HPP_

#include <fstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "manta_interfaces/msg/vehicle_state.hpp"
#include "manta_interfaces/msg/whale_call.hpp"
#include "manta_interfaces/msg/whale_call_processed.hpp"
#include "manta_interfaces/msg/whale_detection.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"


class WhaleDetectionNode : public rclcpp::Node {
public:
    WhaleDetectionNode();

private:
    void handle_shutdown(const std_msgs::msg::Bool::SharedPtr message);
    void handle_vehicle_state(const manta_interfaces::msg::VehicleState::SharedPtr message, const std::string &vehicle_id);
    void handle_whale_call(const manta_interfaces::msg::WhaleCall::SharedPtr call);

    void try_process_pending_call();

    std::vector<std::string> vehicle_ids_;
    double detection_range_ = 500.0;

    bool pending_call_available_ = false;
    manta_interfaces::msg::WhaleCall pending_call_;
    std::unordered_map<std::string, manta_interfaces::msg::VehicleState> latest_vehicle_states_;
    std::ofstream detection_log_;

    rclcpp::Publisher<manta_interfaces::msg::WhaleDetection>::SharedPtr detection_pub_;
    rclcpp::Publisher<manta_interfaces::msg::WhaleCallProcessed>::SharedPtr processed_pub_;

    rclcpp::Subscription<manta_interfaces::msg::WhaleCall>::SharedPtr whale_call_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr shutdown_sub_;
    std::vector<rclcpp::Subscription<manta_interfaces::msg::VehicleState>::SharedPtr> vehicle_state_subs_;
};

#endif  // WHALE_DETECTION_SIM__WHALE_DETECTION_NODE_HPP_