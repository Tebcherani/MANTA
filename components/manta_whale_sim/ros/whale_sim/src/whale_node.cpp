#include "whale_sim/whale_node.hpp"
#include "whale_sim/whale_state_json.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>

WhaleNode::WhaleNode() : rclcpp::Node("whale_node") {
    load_config();
    validate_config();
    open_whale_log();
    seed_generator();
    initialize_whales();
    state_log_.open_in_log_directory("whale_state.jsonl");
    log_state_if_changed();
    create_ros_interfaces();

    sim_state_.ready.data = true;
    log_state_if_changed();
    ready_pub_->publish(sim_state_.ready);

    RCLCPP_INFO(get_logger(), "Initialized %d whales.", sim_state_.whale_count);
}

void WhaleNode::log_state_if_changed() {
    state_log_.record(sim_state_, write_whale_runtime_json,
        [](std::ostream &out, const WhaleState &state) { write_json(out, state); });
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WhaleNode>());
    rclcpp::shutdown();
    return 0;
}
