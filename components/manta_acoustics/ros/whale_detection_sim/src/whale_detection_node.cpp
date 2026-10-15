#include "whale_detection_sim/whale_detection_node.hpp"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <stdexcept>

WhaleDetectionNode::WhaleDetectionNode() : rclcpp::Node("whale_detection_node") {
    load_config();
    open_detection_log();
    create_ros_interfaces();
    
    RCLCPP_INFO(get_logger(), "Whale detector initialized for %zu vehicles with a %.1f m detection range.", vehicle_ids_.size(), detection_range_m_);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    try {
        rclcpp::spin(std::make_shared<WhaleDetectionNode>());
    }
    catch (const std::exception &error) {
        RCLCPP_FATAL(rclcpp::get_logger("whale_detection_node"), "%s", error.what());
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}