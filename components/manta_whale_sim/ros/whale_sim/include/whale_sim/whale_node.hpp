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

    // Simulation state and precomputed behavior for one whale.
    struct Whale {
        std::string id;

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
    void initialize_whales();
    void open_whale_log();
    void seed_generator();
    void validate_config() const;

    std::vector<TrajectoryPoint> generate_trajectory(double speed);
    std::vector<double> generate_call_times(double start_time, double end_time);
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
    double next_whale_log_time_ = 0.0;
    std::ofstream whale_log_;
    double whale_sample_period_ = 60.0;

    // Scenario
    double mission_duration_;

    // Simulation
    std::mt19937 generator_;
    std::vector<Whale> whales_;

    // Whale calling
    double bout_mean_duration_;
    double bout_mean_silent_duration_;
    double bout_active_call_rate_per_hour_;

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