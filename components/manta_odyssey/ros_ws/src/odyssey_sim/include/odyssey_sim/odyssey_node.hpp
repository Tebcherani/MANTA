#ifndef ODYSSEY_SIM__ODYSSEY_NODE_HPP_
#define ODYSSEY_SIM__ODYSSEY_NODE_HPP_

#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int64.hpp"

#include "manta_interfaces/msg/sim_step.hpp"
#include "manta_interfaces/msg/sim_step_result.hpp"
#include "manta_interfaces/msg/vehicle_state.hpp"
#include "odyssey_interfaces/msg/whale_bearing_position_estimate.hpp"
#include "whale_estimator_interfaces/msg/whale_kalman_estimate.hpp"
#include "acoustics_interfaces/msg/bearing_transmission.hpp"
#include "acoustics_interfaces/msg/bearing_observation.hpp"
#include "acoustics_interfaces/msg/bearing_processed.hpp"
#include "odyssey_sim/odyssey_state.hpp"

class OdysseyNode : public rclcpp::Node {
    public:
        OdysseyNode();
        ~OdysseyNode() override;

    private:
        // ---------------------------------------------------------------------
        // Functions
        // ---------------------------------------------------------------------

        using BearingPosition = OdysseyState::BearingPosition;

        // Setup
        void load_config();

        // ROS interface setup
        void create_publishers(const rclcpp::QoS &qos);
        rclcpp::QoS create_qos() const;
        rclcpp::QoS create_whale_qos() const;
        void create_ros_interfaces();
        void create_subscribers(const rclcpp::QoS &qos, const rclcpp::QoS &whale_qos);

        // Callbacks
        void handle_shared_bearing(const acoustics_interfaces::msg::BearingTransmission::SharedPtr msg);
        void handle_shutdown(const std_msgs::msg::Bool::SharedPtr msg);
        void handle_sim_step();
        void handle_startup();
        void handle_bearing_observation(const acoustics_interfaces::msg::BearingObservation::SharedPtr msg);
        void handle_whale_estimate(const whale_estimator_interfaces::msg::WhaleKalmanEstimate::SharedPtr msg);

        void try_finish_init();

        // Output
        void publish_bearing();
        void publish_vehicle_state();
        bool write_final_metrics();

        void open_whale_prediction_log();
        void log_whale_prediction();

        // ---------------------------------------------------------------------
        // Variables
        // ---------------------------------------------------------------------

        OdysseyState odyssey_state_;
        std::ofstream whale_prediction_log_;


        // ---------------------------------------------------------------------
        // ROS interfaces
        // ---------------------------------------------------------------------

        // Publishers
        rclcpp::Publisher<acoustics_interfaces::msg::BearingProcessed>::SharedPtr bearing_processed_pub_;
        rclcpp::Publisher<acoustics_interfaces::msg::BearingTransmission>::SharedPtr bearing_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
        rclcpp::Publisher<manta_interfaces::msg::SimStepResult>::SharedPtr step_result_pub_;
        rclcpp::Publisher<manta_interfaces::msg::VehicleState>::SharedPtr vehicle_state_pub_;
        rclcpp::Publisher<odyssey_interfaces::msg::WhaleBearingPositionEstimate>::SharedPtr whale_fix_pub_;

        // Subscribers
        rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr calendar_start_time_sub_;
        rclcpp::Subscription<acoustics_interfaces::msg::BearingTransmission>::SharedPtr shared_bearing_sub_;
        rclcpp::Subscription<manta_interfaces::msg::SimStep>::SharedPtr sim_sim_step_sub_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sim_time_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr startup_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr shutdown_sub_;
        rclcpp::Subscription<acoustics_interfaces::msg::BearingObservation>::SharedPtr bearing_sub_;
        rclcpp::Subscription<whale_estimator_interfaces::msg::WhaleKalmanEstimate>::SharedPtr whale_estimate_sub_;
};

#endif
