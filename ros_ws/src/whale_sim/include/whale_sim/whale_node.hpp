#ifndef WHALE_SIM__WHALE_NODE_HPP_
#define WHALE_SIM__WHALE_NODE_HPP_

#include <cstddef>
#include <fstream>
#include <random>
#include <string>
#include <vector>

#include "manta_interfaces/msg/sim_step.hpp"
#include "manta_interfaces/msg/whale_state_array.hpp"
#include "manta_interfaces/srv/get_next_whale_call.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"


class WhaleNode : public rclcpp::Node {
public:
    WhaleNode();

private:
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

    std::vector<TrajectoryPoint> generate_trajectory(
        double initial_north, double initial_east,
        double crossing_heading,
        double speed,
        double start_time,
        double perpendicular_north, double perpendicular_east,
        const WhaleGroup &group
    );
    TrajectoryPoint trajectory_at_time(const Whale &whale, double time) const;
    void initialize_whale_groups(double line_length);
    std::vector<double> generate_call_times(double start_time, double end_time, double call_rate_per_hour, const WhaleGroup &group);
    void initialize_whales();
    void update_whales(const manta_interfaces::msg::SimStep &step);
    void handle_get_next_call(const manta_interfaces::srv::GetNextWhaleCall::Request &request, manta_interfaces::srv::GetNextWhaleCall::Response &response);

    int whale_count_;
    double line_start_north_;
    double line_start_east_;
    double line_end_north_;
    double line_end_east_;
    double speed_min_;
    double speed_max_;
    double start_distance_;
    double whale_depth_;
    double mission_duration_;
    double call_rate_min_;
    double call_rate_max_;
    double call_activity_interval_;
    double call_activity_correlation_;
    double group_log_rate_std_;
    double whale_sample_period_ = 60.0;
    double next_whale_log_time_ = 0.0;
    double turn_interval_;
    double heading_correlation_;
    double heading_std_degrees_;
    double group_mean_size_;
    double group_position_std_;
    double group_start_time_std_;
    double shared_heading_fraction_;

    std::mt19937 generator_;
    std::vector<WhaleGroup> whale_groups_;
    std::vector<Whale> whales_;
    std::ofstream whale_log_;

    rclcpp::Publisher<manta_interfaces::msg::WhaleStateArray>::SharedPtr whale_states_pub_;

    rclcpp::Subscription<manta_interfaces::msg::SimStep>::SharedPtr step_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr shutdown_sub_;

    rclcpp::Service<manta_interfaces::srv::GetNextWhaleCall>::SharedPtr next_call_service_;
};

#endif  // WHALE_SIM__WHALE_NODE_HPP_