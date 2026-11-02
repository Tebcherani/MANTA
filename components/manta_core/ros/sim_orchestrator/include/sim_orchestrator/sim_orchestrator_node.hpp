#ifndef SIM_ORCHESTRATOR__SIM_ORCHESTRATOR_NODE_HPP_
#define SIM_ORCHESTRATOR__SIM_ORCHESTRATOR_NODE_HPP_

#include <fstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int64.hpp"

#include "acoustics_interfaces/msg/bearing_processed.hpp"
#include "acoustics_interfaces/msg/whale_call_processed.hpp"
#include "manta_interfaces/msg/sim_step.hpp"
#include "manta_interfaces/msg/sim_step_result.hpp"
#include "whale_interfaces/msg/whale_call.hpp"
#include "whale_interfaces/srv/get_next_whale_call.hpp"

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

    // Setup
    void initialize_simulation_config();
    void initialize_simulation_state();
    void load_vehicle_config();
    void open_simulation_log();
    void open_whale_call_log();
    void publish_startup_state();

    // ROS interface setup
    void create_clients();
    void create_publishers(const rclcpp::QoS &qos);
    rclcpp::QoS create_qos() const;
    void create_ros_interfaces();
    void create_subscribers(const rclcpp::QoS &qos);

    // Callbacks
    void handle_bearing_processed(const acoustics_interfaces::msg::BearingProcessed::SharedPtr msg);
    void handle_next_whale_call_response(rclcpp::Client<whale_interfaces::srv::GetNextWhaleCall>::SharedFuture future);
    void handle_shutdown_timer();
    void handle_step_result();
    void handle_whale_call_processed(const acoustics_interfaces::msg::WhaleCallProcessed::SharedPtr msg);

    void check_all_step_results_received();
    void check_whale_call_processing_complete();

    // Outputs
    void publish_current_step();
    void publish_whale_call(const whale_interfaces::msg::WhaleCall &call);

    // Simulation coordination
    void check_all_vehicles_initialized();
    void request_simulation_shutdown(bool failed = false);

    // Whale call coordination
    void request_next_whale_call(double whale_call_after_time_s);
    
    // ---------------------------------------------------------------------
    // Variables
    // ---------------------------------------------------------------------

    SimOrchestratorState state_;
    std::ofstream whale_call_log_;

    // ---------------------------------------------------------------------
    // ROS interfaces
    // ---------------------------------------------------------------------

    // Clients
    rclcpp::Client<whale_interfaces::srv::GetNextWhaleCall>::SharedPtr next_whale_call_client_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr calendar_start_time_pub_;
    rclcpp::Publisher<manta_interfaces::msg::SimStep>::SharedPtr sim_step_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr sim_time_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr startup_ready_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr shutdown_pub_;
    rclcpp::Publisher<whale_interfaces::msg::WhaleCall>::SharedPtr whale_call_pub_;

    // Subscribers
    rclcpp::Subscription<acoustics_interfaces::msg::BearingProcessed>::SharedPtr bearing_processed_sub_;
    std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> ready_subs_;
    std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> whale_estimator_ready_subs_;
    std::vector<rclcpp::Subscription<manta_interfaces::msg::SimStepResult>::SharedPtr> step_result_subs_;
    rclcpp::Subscription<acoustics_interfaces::msg::WhaleCallProcessed>::SharedPtr whale_call_processed_sub_;

    // Timers
    rclcpp::TimerBase::SharedPtr shutdown_timer_;
};

#endif
