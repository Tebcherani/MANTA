#ifndef WHALE_SIM__WHALE_NODE_HPP_
#define WHALE_SIM__WHALE_NODE_HPP_

#include <cstddef>
#include <fstream>
#include <random>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include "manta_interfaces/msg/sim_step.hpp"
#include "whale_interfaces/msg/whale_state_array.hpp"
#include "whale_interfaces/srv/get_next_whale_call.hpp"


class WhaleNode : public rclcpp::Node {
public:
    WhaleNode();

private:
    // ---------------------------------------------------------------------
    // Structures
    // ---------------------------------------------------------------------

    // One timestamped point in a precomputed whale trajectory.
    struct WhaleWaypoint {
        double sim_time_s;
        double whale_north_m;
        double whale_east_m;
        double whale_velocity_north_mps;
        double whale_velocity_east_mps;
    };

    // Shared movement and calling behavior for one group of whales.
    struct WhaleGroup {
        double crossing_distance_m;
        double side;
        double start_time_s;
        std::vector<double> heading_errors;
    };

    // Simulation state and precomputed behavior for one whale.
    struct Whale {
        std::string id;
        int group_index;

        double start_time_s;

        std::vector<double> call_times;
        std::size_t next_call_index = 0;

        std::vector<WhaleWaypoint> trajectory;

        // Current values published in WhaleState messages and written to CSV.
        double north;
        double east;
        double velocity_north;
        double velocity_east;

        // Whale lifecycle state.
        bool active;
        bool complete;
        double starting_side;
    };

    // Geometric properties of the picket line expressed in local north-east coordinates.
    struct LineGeometry {
        double length;
        double along_north;
        double along_east;
        double perpendicular_north;
        double perpendicular_east;
    };

    // Initial whale position and direction of travel toward the picket line.
    struct WhaleInitialState {
        double north;
        double east;
        double heading;
    };

    // ---------------------------------------------------------------------
    // Functions
    // ---------------------------------------------------------------------

    // Setup
    void load_config();
    void initialize_whale_groups(double line_length);
    void initialize_whale_dynamics(Whale &whale, const WhaleInitialState &initial_state, double speed, const LineGeometry &line, const WhaleGroup &group);
    void initialize_whales();
    void open_whale_log();
    void seed_generator();
    void validate_config() const;

    WhaleInitialState compute_initial_state(const WhaleGroup &group, const LineGeometry &line, double crossing_distance_m, double speed) const;
    LineGeometry compute_line_geometry() const;
    std::vector<WhaleWaypoint> generate_trajectory(double initial_north, double initial_east, double crossing_heading, double speed, double start_time_s, const LineGeometry &line, const WhaleGroup &group);
    Whale create_whale(std::size_t whale_index, std::size_t group_index, const WhaleGroup &group, double start_time_s) const;
    std::vector<double> generate_population_call_times(double start_time_s, double end_time);
    void assign_population_call_times();
    std::size_t get_group_index_for_whale(std::size_t whale_index) const;
    const WhaleGroup &get_group_for_whale(std::size_t whale_index) const;
    double sample_crossing_distance_m(const WhaleGroup &group, double line_length);
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

    // Output
    double next_whale_log_time_s_ = 0.0;
    std::ofstream whale_log_;
    double whale_sample_period_s_ = 60.0;

    // Scenario
    double picket_line_start_north_m_;
    double picket_line_start_east_m_;
    double picket_line_end_north_m_;
    double picket_line_end_east_m_;

    double mission_duration_s_;

    // Simulation
    std::mt19937 rng_;
    std::vector<WhaleGroup> whale_groups_;
    std::vector<Whale> whales_;

    // Whale initialization
    int whale_count_;
    bool whales_active_entire_mission_b_ = false;
    std::string whale_initialization_mode_;

    double whale_start_distance_from_picket_line_m_;
    double whale_initial_north_m_;
    double whale_initial_east_m_;
    double whale_initial_heading_deg_;

    double whale_call_bout_mean_duration_s_;
    double whale_call_bout_mean_silent_duration_s_;
    double whale_active_call_bout_rate_callph;

    double whale_group_mean_size_;
    double whale_group_position_std_m_;
    double whale_group_start_time_std_s_;
    double whale_group_heading_fraction_;

    double whale_heading_correlation_;
    double whale_heading_std_deg_;
    double whale_speed_mean_mps_;
    double whale_speed_std_mps_;
    double whale_turn_interval_s_;

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