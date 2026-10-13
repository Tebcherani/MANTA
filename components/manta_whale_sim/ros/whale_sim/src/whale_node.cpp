#include "whale_sim/whale_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>

WhaleNode::WhaleNode() : rclcpp::Node("whale_node"), generator_(42) {
    load_config();
    validate_config();
    open_whale_log();
    seed_generator();
    initialize_whales();
    create_ros_interfaces();

    RCLCPP_INFO(get_logger(), "Initialized whale.");
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WhaleNode>());
    rclcpp::shutdown();
    return 0;
}