#ifndef SIM_ORCHESTRATOR__SIM_ORCHESTRATOR_NODE_HPP_
#define SIM_ORCHESTRATOR__SIM_ORCHESTRATOR_NODE_HPP_

#include <fstream>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int64.hpp"

#include "manta_interfaces/msg/sim_step.hpp"
#include "manta_interfaces/msg/sim_step_result.hpp"
#include "whale_interfaces/msg/whale_call.hpp"
#include "whale_interfaces/msg/whale_step_result.hpp"

#include "manta_interfaces/state_log.hpp"
#include "sim_orchestrator/sim_orchestrator_state.hpp"


class SimOrchestratorNode : public rclcpp::Node {
public:
    explicit SimOrchestratorNode();
    ~SimOrchestratorNode() override;

    bool simulation_failed() const;

private:
    // ---------------------------------------------------------------------
    // Functions
    // ---------------------------------------------------------------------

    void log_state_if_changed();

    // Setup
    void initialize_simulation_config();
    void initialize_simulation_state();
    void load_vehicle_config();
    void open_simulation_log();
    void open_whale_call_log();
    void publish_startup_state();

    // ROS interface setup
    void create_publishers(const rclcpp::QoS &qos);
    rclcpp::QoS create_qos() const;
    void create_ros_interfaces();
    void create_subscribers(const rclcpp::QoS &qos);

    // Callbacks
    void handle_shutdown_timer();
    void handle_step_result();
    void handle_whale_step_result();

    void check_all_step_results_received();

    // Outputs
    void publish_current_step();
    void record_whale_call();

    // Simulation coordination
    void check_all_vehicles_initialized();
    void request_simulation_shutdown(bool failed = false);

    
    // ---------------------------------------------------------------------
    // Variables
    // ---------------------------------------------------------------------

    SimOrchestratorState state_;
    std::ofstream whale_call_log_;
    manta::StateLog state_log_;

    // ---------------------------------------------------------------------
    // ROS interfaces
    // ---------------------------------------------------------------------

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr calendar_start_time_pub_;
    rclcpp::Publisher<manta_interfaces::msg::SimStep>::SharedPtr sim_step_pub_;
    rclcpp::Publisher<manta_interfaces::msg::SimStep>::SharedPtr whale_step_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr sim_time_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr startup_ready_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr shutdown_pub_;

    // Subscribers
    std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> ready_subs_;
    std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> whale_estimator_ready_subs_;
    std::vector<rclcpp::Subscription<manta_interfaces::msg::SimStepResult>::SharedPtr> step_result_subs_;
    rclcpp::Subscription<whale_interfaces::msg::WhaleStepResult>::SharedPtr whale_step_result_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr whale_ready_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr detection_ready_sub_;

    // Timers
    rclcpp::TimerBase::SharedPtr shutdown_timer_;
};

#endif
