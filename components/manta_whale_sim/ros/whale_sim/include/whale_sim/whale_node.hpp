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
#include "whale_interfaces/msg/whale_state_array.hpp"
#include "whale_interfaces/srv/get_next_whale_call.hpp"


class WhaleNode : public rclcpp::Node {
public:
    WhaleNode();

private:
    using WhaleWaypoint = WhaleState::WhaleWaypoint;
    using WhaleGroup = WhaleState::WhaleGroup;
    using Whale = WhaleState::Whale;
    using LineGeometry = WhaleState::LineGeometry;
    using WhaleInitialState = WhaleState::WhaleInitialState;

    // ---------------------------------------------------------------------
    // Functions
    // ---------------------------------------------------------------------

    void log_state_if_changed();

    // Setup
    void load_config();
    void initialize_whale_groups(double line_length);
    void initialize_whale_dynamics(Whale &whale, const WhaleInitialState &initial_state, double speed, const LineGeometry &line, const WhaleGroup &group);
    void initialize_whales();
    void open_whale_log();
    void seed_generator();
    void validate_config() const;

    WhaleInitialState compute_initial_state(const WhaleGroup &group, const LineGeometry &line, double whale_group_crossing_distance_m, double speed) const;
    LineGeometry compute_line_geometry() const;
    std::vector<WhaleWaypoint> generate_trajectory(double initial_north, double initial_east, double crossing_heading, double speed, double start_time_s, const LineGeometry &line, const WhaleGroup &group);
    Whale create_whale(std::size_t whale_index, std::size_t group_index, const WhaleGroup &group, double start_time_s) const;
    std::vector<double> generate_population_whale_call_times_s(double start_time_s, double end_time);
    void assign_population_whale_call_times_s();
    std::size_t get_group_index_for_whale(std::size_t whale_index) const;
    const WhaleGroup &get_group_for_whale(std::size_t whale_index) const;
    double sample_whale_group_crossing_distance_m(const WhaleGroup &group, double line_length);
    double sample_start_time(const WhaleGroup &group);
    double sample_speed();

    // ROS interfaces
    void create_ros_interfaces();
    void create_publishers();
    void create_subscribers();
    void create_services();
    
    // Callbacks
    void handle_get_next_call(const whale_interfaces::srv::GetNextWhaleCall::Request &request, whale_interfaces::srv::GetNextWhaleCall::Response &response);
    void handle_shutdown(const std_msgs::msg::Bool::SharedPtr message);
    void handle_sim_step(const manta_interfaces::msg::SimStep::SharedPtr step);

    WhaleWaypoint trajectory_at_time(const Whale &whale, double time) const;
    void update_whales(const manta_interfaces::msg::SimStep &step);

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
    
    // Services
    rclcpp::Service<whale_interfaces::srv::GetNextWhaleCall>::SharedPtr next_whale_call_service_;

    // Subscribers
    rclcpp::Subscription<manta_interfaces::msg::SimStep>::SharedPtr sim_step_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr shutdown_sub_;
};

#endif  // WHALE_SIM__WHALE_NODE_HPP_