#include "whale_estimator/whale_estimator_node.hpp"
#include "whale_estimator/whale_estimator_state_json.hpp"
#include <memory>

WhaleEstimatorNode::WhaleEstimatorNode() : rclcpp::Node("whale_estimator_node") {
    setup();
    create_ros_interfaces();
}

void WhaleEstimatorNode::log_state_if_changed() {
    state_log_.record(state_);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WhaleEstimatorNode>());
    rclcpp::shutdown();
    return 0;
}
