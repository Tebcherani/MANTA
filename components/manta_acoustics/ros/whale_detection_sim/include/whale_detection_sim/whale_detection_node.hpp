#ifndef WHALE_DETECTION_SIM__WHALE_DETECTION_NODE_HPP_
#define WHALE_DETECTION_SIM__WHALE_DETECTION_NODE_HPP_

#include <fstream>
#include "manta_interfaces/state_log.hpp"
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include "acoustics_interfaces/msg/bearing_observation.hpp"
#include "acoustics_interfaces/msg/whale_call_processed.hpp"
#include "acoustics_interfaces/msg/whale_detection.hpp"
#include "manta_interfaces/msg/vehicle_state.hpp"
#include "whale_interfaces/msg/whale_call.hpp"
#include "whale_detection_sim/whale_detection_state.hpp"

class WhaleDetectionNode : public rclcpp::Node {
public:
    WhaleDetectionNode();

private:
    // ---------------------------------------------------------------------
    // Functions
    // ---------------------------------------------------------------------

    void log_state_if_changed();

    // Setup
    void load_config();
    void open_detection_log();

    // ROS interface setup
    void create_ros_interfaces();
    void create_publishers();
    void create_subscribers();

    // Callbacks
    void handle_shutdown();
    void handle_whale_call();

    // Detections
    void try_process_pending_whale_call();
    bool vehicle_states_match_call_cycle() const;
    void acknowledge_whale_call();
    void record_detection();
    void calculate_range();
    void calculate_bearing();

    // ---------------------------------------------------------------------
    // Variables
    // ---------------------------------------------------------------------

    // Simulation
    WhaleDetectionState sim_state_;

    // Output
    std::ofstream whale_detection_log_;
    manta::StateLog state_log_;

    // ---------------------------------------------------------------------
    // ROS interfaces
    // ---------------------------------------------------------------------

    // Publishers
    std::unordered_map<std::string, rclcpp::Publisher<acoustics_interfaces::msg::BearingObservation>::SharedPtr> bearing_observation_pubs;
    rclcpp::Publisher<acoustics_interfaces::msg::WhaleCallProcessed>::SharedPtr whale_call_processed_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;

    // Subscribers
    rclcpp::Subscription<whale_interfaces::msg::WhaleCall>::SharedPtr whale_call_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr shutdown_sub_;
    std::vector<rclcpp::Subscription<manta_interfaces::msg::VehicleState>::SharedPtr> vehicle_state_subs_;
};

#endif  // WHALE_DETECTION_SIM__WHALE_DETECTION_NODE_HPP_
