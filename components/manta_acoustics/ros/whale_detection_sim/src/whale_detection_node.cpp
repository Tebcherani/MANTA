#include "whale_detection_sim/whale_detection_node.hpp"
#include "whale_detection_sim/whale_detection_state_json.hpp"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <stdexcept>

WhaleDetectionNode::WhaleDetectionNode() : rclcpp::Node("whale_detection_node") {
    load_config();
    open_detection_log();
    state_log_.open_in_log_directory("whale_detection_state.jsonl");
    log_state_if_changed();
    create_ros_interfaces();

    sim_state_.ready.data = true;
    log_state_if_changed();
    ready_pub_->publish(sim_state_.ready);
    
    RCLCPP_INFO(get_logger(), "Whale detector initialized for %zu vehicles with a %.1f m detection range.", sim_state_.vehicle_ids.size(), sim_state_.detection_range_m);
}

void WhaleDetectionNode::log_state_if_changed() {
    state_log_.record(sim_state_);
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
