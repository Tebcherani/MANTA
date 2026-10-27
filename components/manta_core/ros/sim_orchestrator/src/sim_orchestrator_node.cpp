#include "sim_orchestrator/sim_orchestrator_node.hpp"

#include <cstdlib>
#include <exception>
#include <memory>

extern "C" {
    #include "sim_log.h"
}

SimOrchestratorNode::SimOrchestratorNode() : rclcpp::Node("sim_orchestrator") {            
    RCLCPP_INFO(get_logger(), "Initializing simulation orchestrator.");

    load_vehicle_config();
    open_whale_call_log();
    create_ros_interfaces();
    open_simulation_log();
    initialize_simulation_config();
    initialize_simulation_state();

    RCLCPP_INFO(get_logger(), "Simulation orchestrator initialized and ready.");

    publish_startup_state();
}

SimOrchestratorNode::~SimOrchestratorNode() {
    RCLCPP_INFO(get_logger(), "Shutting down simulation orchestrator.");
    if (!close_sim_log()) RCLCPP_ERROR(get_logger(), "Failed to close simulation log.");
}

bool SimOrchestratorNode::simulation_failed() const {return simulation_failed_b_;}

int main(int argc, char *argv[]) {    
    int exit_status = EXIT_SUCCESS;
    rclcpp::init(argc, argv);   // Initialize ROS 2

    try {
        auto node = std::make_shared<SimOrchestratorNode>();  
        rclcpp::spin(node);      
        if (node->simulation_failed()) exit_status = EXIT_FAILURE;
    }
    catch (const std::exception &error) {
        RCLCPP_FATAL(rclcpp::get_logger("sim_orchestrator_node"), "%s", error.what());   // Get error message
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }

    rclcpp::shutdown();
    return exit_status;
}