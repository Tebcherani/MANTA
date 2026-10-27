#ifndef ODYSSEY_SIM__ODYSSEY_NODE_HPP_
#define ODYSSEY_SIM__ODYSSEY_NODE_HPP_

#include <cstdint>
#include <ctime>
#include <deque>
#include <map>
#include <string>
#include <unordered_map>
#include <fstream>
#include <set>

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
            double sim_time_s;
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
        void handle_shared_bearing(const acoustics_interfaces::msg::BearingTransmission::SharedPtr msg);
        void handle_shutdown(const std_msgs::msg::Bool::SharedPtr msg);
        void handle_sim_step(const manta_interfaces::msg::SimStep::SharedPtr msg);
        void handle_sim_time(const std_msgs::msg::Float64::SharedPtr msg);
        void handle_startup(const std_msgs::msg::Bool::SharedPtr msg);
        void handle_bearing_observation(const acoustics_interfaces::msg::BearingObservation::SharedPtr msg);
        void handle_whale_estimate(const whale_estimator_interfaces::msg::WhaleKalmanEstimate::SharedPtr msg);

        void try_finish_init();

        // Output
        void publish_bearing(double simulation_time);
        void publish_vehicle_state(std::uint64_t completed_cycle, double completed_sim_time_s);
        bool write_final_metrics();

        void open_whale_prediction_log();
        void log_whale_prediction(double simulation_time);

        // ---------------------------------------------------------------------
        // Variables
        // ---------------------------------------------------------------------

        // Call detections
        std::deque<BearingPosition> bearing_positions_;
        bool latest_bearing_available_ = false;
        bool latest_bearing_unsent_ = false;
        acoustics_interfaces::msg::BearingTransmission latest_bearing_;

        // Local measurements retained so delayed remote bearings can find their match.
        std::map<std::uint64_t, acoustics_interfaces::msg::BearingTransmission> local_bearings_by_cycle_;

        // Remote measurements that arrived before the corresponding local measurement.
        std::map<std::uint64_t, acoustics_interfaces::msg::BearingTransmission::SharedPtr> pending_shared_bearings_;

        // Calls already successfully triangulated.
        std::set<std::uint64_t> processed_bearing_cycles_;
        
        double communication_interval_s_ = 0.0;
        std::uint64_t next_communication_index_ = 1;

        // Mission
        bool log_decimation_configured_ = false;
        bool metrics_written_ = false;
        bool mission_finished_ = false;
        bool tracking_enabled_ = false;

        // Simulation
        bool calendar_start_time_received_b_ = false;
        bool odyssey_initialized_b_ = false;
        bool odyssey_sim_started_b_ = false;
        bool ready_published_b_ = false;
        bool sim_time_initialized_b_ = false;
        bool sim_time_received_b_ = false;

        time_t calendar_start_time_ = 0;
        double sim_time_s_ = 0.0;

        // Vehicle
        std::string vehicle_id_;
        std::string data_directory_;
        double output_sample_period_s_ = 2.0;

        std::ofstream whale_prediction_log_;
        double next_prediction_log_time_ = 0.0;


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
