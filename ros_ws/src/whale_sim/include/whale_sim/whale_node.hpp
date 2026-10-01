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
#include "manta_interfaces/msg/whale_state_array.hpp"
#include "manta_interfaces/srv/get_next_whale_call.hpp"


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
        std::vector<double> call_rate_multipliers;
    };

    // Simulation state and precomputed behavior for one whale.
    struct Whale {
        std::string id;
        int group_index;

        double depth;
        double start_time;
        double call_rate_per_hour;

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
    void initialize_whale_dynamics(Whale &whale, const WhaleInitialState &initial_state, double speed, const WhaleGroup &group);
    void initialize_whales();
    void open_whale_log();
    void seed_generator();
    void validate_config() const;

    WhaleInitialState compute_initial_state(const WhaleGroup &group, const LineGeometry &line, double crossing_distance, double speed) const;
    LineGeometry compute_line_geometry() const;
    std::vector<TrajectoryPoint> generate_trajectory(
        double initial_north, double initial_east,
        double crossing_heading,
        double speed,
        double start_time,
        const WhaleGroup &group
    );
    Whale create_whale(std::size_t whale_index, std::size_t group_index, const WhaleGroup &group, double start_time, double call_rate) const;
    std::vector<double> generate_call_times(double start_time, double end_time, double call_rate_per_hour, const WhaleGroup &group);
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
    void handle_get_next_call(const manta_interfaces::srv::GetNextWhaleCall::Request &request, manta_interfaces::srv::GetNextWhaleCall::Response &response);
    void handle_shutdown(const std_msgs::msg::Bool::SharedPtr message);
    void handle_sim_step(const manta_interfaces::msg::SimStep::SharedPtr step);

    TrajectoryPoint trajectory_at_time(const Whale &whale, double time) const;
    void update_whales(const manta_interfaces::msg::SimStep &step);

    // ---------------------------------------------------------------------
    // Variables
    // ---------------------------------------------------------------------

    // Output
    double next_whale_log_time_ = 0.0;
    std::ofstream whale_log_;
    double whale_sample_period_ = 60.0;

    // Scenario
    int whale_count_;

    double line_start_north_;
    double line_start_east_;
    double line_end_north_;
    double line_end_east_;

    double mission_duration_;
    double start_distance_;
    double whale_depth_;

    // Simulation
    std::mt19937 generator_;
    std::vector<WhaleGroup> whale_groups_;
    std::vector<Whale> whales_;

    // Whale calling
    double call_activity_correlation_;
    double call_activity_interval_;
    double call_rate_min_;
    double call_rate_max_;
    double group_log_rate_std_;

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
    rclcpp::Publisher<manta_interfaces::msg::WhaleStateArray>::SharedPtr whale_states_pub_;
    
    // Services
    rclcpp::Service<manta_interfaces::srv::GetNextWhaleCall>::SharedPtr next_call_service_;

    // Subscribers
    rclcpp::Subscription<manta_interfaces::msg::SimStep>::SharedPtr step_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr shutdown_sub_;
};

#endif  // WHALE_SIM__WHALE_NODE_HPP_