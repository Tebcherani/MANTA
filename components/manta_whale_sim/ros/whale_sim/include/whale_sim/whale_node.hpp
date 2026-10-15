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
    struct TrajectoryPoint {
        double time;
        double north;
        double east;
        double velocity_north;
        double velocity_east;
    };

    // Shared movement and calling behavior for one group of whales.
    struct WhaleGroup {
        double crossing_distance;
        double side;
        double start_time;
        std::vector<double> heading_errors;
    };

    // Simulation state and precomputed behavior for one whale.
    struct Whale {
        std::string id;
        int group_index;

        double start_time;

        std::vector<double> call_times;
        std::size_t next_call_index = 0;

        std::vector<TrajectoryPoint> trajectory;

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

    WhaleInitialState compute_initial_state(const WhaleGroup &group, const LineGeometry &line, double crossing_distance, double speed) const;
    LineGeometry compute_line_geometry() const;
    std::vector<TrajectoryPoint> generate_trajectory(double initial_north, double initial_east, double crossing_heading, double speed, double start_time, const LineGeometry &line, const WhaleGroup &group);
    Whale create_whale(std::size_t whale_index, std::size_t group_index, const WhaleGroup &group, double start_time) const;
    std::vector<double> generate_population_call_times(double start_time, double end_time);
    void assign_population_call_times();
    std::size_t get_group_index_for_whale(std::size_t whale_index) const;
    const WhaleGroup &get_group_for_whale(std::size_t whale_index) const;
    double sample_crossing_distance(const WhaleGroup &group, double line_length);
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

    TrajectoryPoint trajectory_at_time(const Whale &whale, double time) const;
    void update_whales(const manta_interfaces::msg::SimStep &step);

    // ---------------------------------------------------------------------
    // Variables
    // ---------------------------------------------------------------------

    // Output
    double next_whale_log_time_s_ = 0.0;
    std::ofstream whale_log_;
    double whale_sample_period_s_ = 60.0;

    // Scenario
    int whale_count_;

    double picket_line_start_north_m_;
    double picket_line_start_east_m_;
    double picket_line_end_north_m_;
    double picket_line_end_east_m_;

    double mission_duration_s_;
    double whale_start_distance_from_picket_line_m_;

    // Simulation
    std::mt19937 generator_;
    std::vector<WhaleGroup> whale_groups_;
    std::vector<Whale> whales_;

    // Whale initialization
    bool active_entire_mission_ = false;
    std::string initialization_mode_;
    double initial_north_;
    double initial_east_;
    double initial_heading_degrees_;

    // Whale calling
    double bout_mean_duration_;
    double bout_mean_silent_duration_;
    double bout_active_call_rate_per_hour_;

    // Whale grouping
    double group_mean_size_;
    double group_position_std_;
    double group_start_time_std_;
    double shared_heading_fraction_;

    // Whale movement
    double heading_correlation_;
    double heading_std_degrees_;
    double speed_mean_;
    double speed_std_;
    double turn_interval_;

    // ---------------------------------------------------------------------
    // ROS interfaces
    // ---------------------------------------------------------------------

    // Publishers
    rclcpp::Publisher<whale_interfaces::msg::WhaleStateArray>::SharedPtr whale_states_pub_;
    
    // Services
    rclcpp::Service<whale_interfaces::srv::GetNextWhaleCall>::SharedPtr next_call_service_;

    // Subscribers
    rclcpp::Subscription<manta_interfaces::msg::SimStep>::SharedPtr step_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr shutdown_sub_;
};

#endif  // WHALE_SIM__WHALE_NODE_HPP_