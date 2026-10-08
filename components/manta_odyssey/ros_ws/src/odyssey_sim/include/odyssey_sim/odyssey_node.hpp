#ifndef ODYSSEY_SIM__ODYSSEY_NODE_HPP_
#define ODYSSEY_SIM__ODYSSEY_NODE_HPP_

#include <cstdint>
#include <ctime>
#include <deque>
#include <map>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int64.hpp"

#include "manta_interfaces/msg/sim_step.hpp"
#include "manta_interfaces/msg/sim_step_result.hpp"
#include "manta_interfaces/msg/vehicle_state.hpp"
#include "acoustics_interfaces/msg/bearing_observation.hpp"
#include "acoustics_interfaces/msg/bearing_msg.hpp"

class OdysseyNode : public rclcpp::Node {
    public:
        OdysseyNode();
        ~OdysseyNode() override;

    private:
        // ---------------------------------------------------------------------
        // Functions
        // ---------------------------------------------------------------------

        struct BearingPosition {
            std::uint64_t cycle;
            double sim_time;
            double north;
            double east;
        };

        // Setup
        void load_config();

        // ROS interface setup
        void create_publishers(const rclcpp::QoS &qos);
        rclcpp::QoS create_qos() const;
        rclcpp::QoS create_whale_qos() const;
        void create_ros_interfaces();
        void create_subscribers(const rclcpp::QoS &qos, const rclcpp::QoS &whale_qos);

        // Callbacks
        void handle_calendar_start_time(const std_msgs::msg::Int64::SharedPtr msg);
        void handle_shared_bearing(const acoustics_interfaces::msg::BearingMsg::SharedPtr msg);
        void handle_shutdown(const std_msgs::msg::Bool::SharedPtr msg);
        void handle_sim_step(const manta_interfaces::msg::SimStep::SharedPtr msg);
        void handle_sim_time(const std_msgs::msg::Float64::SharedPtr msg);
        void handle_startup(const std_msgs::msg::Bool::SharedPtr msg);
        void handle_bearing_observation(const acoustics_interfaces::msg::BearingObservation::SharedPtr msg);

        void try_finish_init();

        // Output
        void publish_vehicle_state(std::uint64_t completed_cycle, double completed_sim_time);
        bool write_final_metrics();

        // ---------------------------------------------------------------------
        // Variables
        // ---------------------------------------------------------------------

        // Call detections
        double call_update_interval_ = 0.0;
        bool pending_call_available_ = false;
        double pending_call_north_ = 0.0;
        double pending_call_east_ = 0.0;
        double pending_call_time_ = 0.0;
        double next_call_update_time_ = 0.0;
        std::deque<BearingPosition> bearing_positions_;
        std::map<std::uint64_t, std::unordered_map<std::string, acoustics_interfaces::msg::BearingMsg>> bearing_pairs_;

        // Mission
        bool log_decimation_configured_ = false;
        bool metrics_written_ = false;
        bool mission_finished_ = false;
        bool tracking_enabled_ = false;

        // Simulation
        bool calendar_start_time_received_ = false;
        bool odyssey_initialized_ = false;
        bool odyssey_sim_started_ = false;
        bool ready_published_ = false;
        bool sim_time_initialized_ = false;
        bool sim_time_received_ = false;

        time_t calendar_start_time_ = 0;
        double sim_time_ = 0.0;

        // Vehicle
        std::string vehicle_id_;
        std::string data_directory_;
        double output_sample_period_ = 2.0;


        // ---------------------------------------------------------------------
        // ROS interfaces
        // ---------------------------------------------------------------------

        // Publishers
        rclcpp::Publisher<acoustics_interfaces::msg::BearingMsg>::SharedPtr bearing_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
        rclcpp::Publisher<manta_interfaces::msg::SimStepResult>::SharedPtr step_result_pub_;
        rclcpp::Publisher<manta_interfaces::msg::VehicleState>::SharedPtr vehicle_state_pub_;

        // Subscribers
        rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr calendar_start_time_sub_;
        rclcpp::Subscription<acoustics_interfaces::msg::BearingMsg>::SharedPtr shared_bearing_sub_;
        rclcpp::Subscription<manta_interfaces::msg::SimStep>::SharedPtr sim_step_sub_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sim_time_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr startup_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr shutdown_sub_;
        rclcpp::Subscription<acoustics_interfaces::msg::BearingObservation>::SharedPtr bearing_sub_;};

#endif