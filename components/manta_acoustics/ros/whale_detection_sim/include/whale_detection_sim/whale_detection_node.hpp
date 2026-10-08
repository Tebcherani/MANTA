#ifndef WHALE_DETECTION_SIM__WHALE_DETECTION_NODE_HPP_
#define WHALE_DETECTION_SIM__WHALE_DETECTION_NODE_HPP_

#include <fstream>
#include <random>
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

class WhaleDetectionNode : public rclcpp::Node {
public:
    WhaleDetectionNode();

private:
    // ---------------------------------------------------------------------
    // Functions
    // ---------------------------------------------------------------------

    // Setup
    void load_config();
    void open_detection_log();

    // ROS interface setup
    void create_ros_interfaces();
    void create_publishers();
    void create_subscribers();

    // Callbacks
    void handle_shutdown(const std_msgs::msg::Bool::SharedPtr msg);
    void handle_vehicle_state(const manta_interfaces::msg::VehicleState::SharedPtr msg, const std::string &vehicle_id);
    void handle_whale_call(const whale_interfaces::msg::WhaleCall::SharedPtr call);

    // Detections
    void try_process_pending_call();

    // Detection noise
    double position_std_m_ = 0.0;
    std::mt19937_64 rng_;
    std::normal_distribution<double> normal_{0.0, 1.0};

    // ---------------------------------------------------------------------
    // Variables
    // ---------------------------------------------------------------------

    // Configuration
    std::vector<std::string> vehicle_ids_;
    double detection_range_ = 11000;
    double bearing_std_rad_ = 0.0;
    double call_update_interval_ = 0.0;
    std::unordered_map<std::string, double> last_received_call_times_;

    // Simulation
    bool pending_call_available_ = false;
    std::unordered_map<std::string, manta_interfaces::msg::VehicleState> latest_vehicle_states_;
    whale_interfaces::msg::WhaleCall pending_call_;

    // Output
    std::ofstream detection_log_;

    // ---------------------------------------------------------------------
    // ROS interfaces
    // ---------------------------------------------------------------------

    // Publishers
    std::unordered_map<std::string, rclcpp::Publisher<acoustics_interfaces::msg::BearingObservation>::SharedPtr> bearing_pubs_;
    rclcpp::Publisher<acoustics_interfaces::msg::WhaleCallProcessed>::SharedPtr processed_pub_;

    // Subscribers
    rclcpp::Subscription<whale_interfaces::msg::WhaleCall>::SharedPtr whale_call_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr shutdown_sub_;
    std::vector<rclcpp::Subscription<manta_interfaces::msg::VehicleState>::SharedPtr> vehicle_state_subs_;
};

#endif  // WHALE_DETECTION_SIM__WHALE_DETECTION_NODE_HPP_