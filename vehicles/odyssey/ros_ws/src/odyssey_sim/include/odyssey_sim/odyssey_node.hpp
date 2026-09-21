#ifndef ODYSSEY_SIM__ODYSSEY_NODE_HPP_
#define ODYSSEY_SIM__ODYSSEY_NODE_HPP_

#include <cstdint>
#include <ctime>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int64.hpp"

#include "manta_interfaces/msg/sim_step.hpp"
#include "manta_interfaces/msg/sim_step_result.hpp"
#include "manta_interfaces/msg/vehicle_state.hpp"


class OdysseyNode : public rclcpp::Node {
    public:
        OdysseyNode();
        ~OdysseyNode() override;

    private:
        // Setup
        void create_publishers(const rclcpp::QoS &qos);
        rclcpp::QoS create_qos() const;
        void create_ros_interfaces();
        void create_subscribers(const rclcpp::QoS &qos);
        void load_config();
    
        // Callbacks
        void handle_calendar_start_time(const std_msgs::msg::Int64::SharedPtr msg);
        void handle_shutdown(const std_msgs::msg::Bool::SharedPtr msg);
        void handle_sim_step(const manta_interfaces::msg::SimStep::SharedPtr msg);
        void handle_sim_time(const std_msgs::msg::Float64::SharedPtr msg);
        void handle_startup(const std_msgs::msg::Bool::SharedPtr msg);

        void publish_vehicle_state(std::uint64_t completed_cycle, double completed_sim_time);
        void try_finish_init();
        bool write_final_metrics();

        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
        rclcpp::Publisher<manta_interfaces::msg::SimStepResult>::SharedPtr step_result_pub_;
        rclcpp::Publisher<manta_interfaces::msg::VehicleState>::SharedPtr vehicle_state_pub_;

        rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr calendar_start_time_sub_;
        rclcpp::Subscription<manta_interfaces::msg::SimStep>::SharedPtr sim_step_sub_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sim_time_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr startup_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr shutdown_sub_;

        bool calendar_start_time_received_ = false;
        bool odyssey_initialized_ = false;
        bool ready_published_ = false;
        bool sim_time_initialized_ = false;
        bool sim_time_received_ = false;
        bool odyssey_sim_started_ = false;
        bool mission_finished_ = false;
        bool log_decimation_configured_ = false;
        bool metrics_written_ = false;

        time_t calendar_start_time_ = 0;
        double sim_time_ = 0.0;
        double output_sample_period_ = 2.0;
        std::string vehicle_id_;
        std::string data_directory_;
};

#endif