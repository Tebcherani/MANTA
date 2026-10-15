#include "odyssey_sim/odyssey_node.hpp"

#include <cstdlib>
#include <exception>
#include <memory>

extern "C" {
    #include "odyssey_sim.h"
}

OdysseyNode::OdysseyNode() : rclcpp::Node("odyssey_sim") {
    load_config();
    open_whale_prediction_log();
    create_ros_interfaces();

    RCLCPP_INFO(get_logger(), "%s waiting for simulation startup data.", vehicle_id_.c_str());
}

OdysseyNode::~OdysseyNode() {
    RCLCPP_INFO(get_logger(), "Shutting down vehicle '%s'.", vehicle_id_.c_str());
            
    if (!odyssey_sim_started_) return;  // No native cleanup is required if the simulator never started.
    if (!write_final_metrics()) RCLCPP_ERROR(get_logger(), "Failed to write final metrics for vehicle '%s'.", vehicle_id_.c_str());         // Save final metrics before closing the native simulator.
    if (!end_odyssey_sim(nullptr)) RCLCPP_ERROR(get_logger(), "Failed to cleanly end simulator for vehicle '%s'.", vehicle_id_.c_str());    // Flush and close the native Odyssey mission data file.
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);   // Initialize ROS 2

    try {
        auto node = std::make_shared<OdysseyNode>();    // Create Odyssey node
        rclcpp::spin(node);                             // Node listens
    }
    catch (const std::exception &error) {
        RCLCPP_FATAL(rclcpp::get_logger("odyssey_node"), "%s", error.what());   // Get error message
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}