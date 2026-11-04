#ifndef WHALE_SIM__WHALE_NODE_HPP_
#define WHALE_SIM__WHALE_NODE_HPP_

#include <cstddef>
#include <fstream>
#include "manta_interfaces/state_log.hpp"
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "whale_sim/whale_state.hpp"
#include "std_msgs/msg/bool.hpp"

#include "manta_interfaces/msg/sim_step.hpp"
#include "whale_interfaces/msg/whale_call.hpp"
#include "whale_interfaces/msg/whale_step_result.hpp"
#include "whale_interfaces/msg/whale_state_array.hpp"


class WhaleNode : public rclcpp::Node {
public:
    WhaleNode();

private:
    // ---------------------------------------------------------------------
    // Functions
    // ---------------------------------------------------------------------

    void log_state_if_changed();

    // Setup
    void load_config();
    void initialize_whale_groups();
    void initialize_whale_dynamics();
    void initialize_whales();
    void open_whale_log();
    void seed_generator();
    void validate_config() const;

    WhaleInitialState compute_initial_state() const;
    LineGeometry compute_line_geometry() const;
    std::vector<WhaleWaypoint> generate_trajectory();
    Whale create_whale() const;
    std::vector<double> generate_population_whale_call_times_s();
    void assign_population_whale_call_times_s();
    std::size_t get_group_index_for_whale() const;
    double sample_whale_group_crossing_distance_m();
    double sample_start_time();
    double sample_speed();

    // ROS interfaces
    void create_ros_interfaces();
    void create_publishers();
    void create_subscribers();
    
    // Callbacks
    void handle_sim_step();
    void find_next_call();
    void handle_shutdown();
    void publish_pending_call();

    WhaleWaypoint trajectory_at_time() const;
    void update_whales();

    // ---------------------------------------------------------------------
    // Variables
    // ---------------------------------------------------------------------

    WhaleState sim_state_;
    std::ofstream whale_log_;
    manta::StateLog state_log_;

    // ---------------------------------------------------------------------
    // ROS interfaces
    // ---------------------------------------------------------------------

    // Publishers
    rclcpp::Publisher<whale_interfaces::msg::WhaleStateArray>::SharedPtr whale_states_pub_;
    rclcpp::Publisher<whale_interfaces::msg::WhaleCall>::SharedPtr whale_call_pub_;
    rclcpp::Publisher<whale_interfaces::msg::WhaleStepResult>::SharedPtr step_result_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;

    // Subscribers
    rclcpp::Subscription<manta_interfaces::msg::SimStep>::SharedPtr sim_step_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr shutdown_sub_;
};

#endif  // WHALE_SIM__WHALE_NODE_HPP_
