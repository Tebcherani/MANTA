#ifndef SIM_ORCHESTRATOR__SIM_ORCHESTRATOR_NODE_HPP_
#define SIM_ORCHESTRATOR__SIM_ORCHESTRATOR_NODE_HPP_

#include <fstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int64.hpp"

#include "manta_interfaces/msg/sim_step.hpp"
#include "manta_interfaces/msg/sim_step_result.hpp"
#include "manta_interfaces/msg/whale_call.hpp"
#include "manta_interfaces/msg/whale_call_processed.hpp"
#include "manta_interfaces/srv/get_next_whale_call.hpp"

extern "C" {
    #include "sim_config.h"
    #include "sim_state.h"
    #include "world_sim.h"
}


class SimOrchestratorNode : public rclcpp::Node {
public:
    explicit SimOrchestratorNode();
    ~SimOrchestratorNode() override;

    bool simulation_failed() const;

private:
    struct VehicleState {
        bool ready = false;
        bool active = true;
        bool step_complete = false;
    };

    // Callbacks
    void handle_next_whale_call_response(rclcpp::Client<manta_interfaces::srv::GetNextWhaleCall>::SharedFuture future);
    void handle_shutdown_timer();
    void handle_step_result(const manta_interfaces::msg::SimStepResult::SharedPtr msg, const std::string &expected_vehicle_id);
    void handle_vehicle_ready(const std_msgs::msg::Bool::SharedPtr msg, const std::string &vehicle_id);
    void handle_whale_call_processed(const manta_interfaces::msg::WhaleCallProcessed::SharedPtr msg);

    // Configuration
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

    void request_next_whale_call(double after_time);
    void publish_whale_call(const manta_interfaces::msg::WhaleCall &call);
    void check_all_vehicles_initialized();
    void publish_current_step();
    void check_all_step_results_received();
    void request_simulation_shutdown(bool failed = false);

    bool simulation_started_ = false;
    bool simulation_failed_ = false;
    bool next_whale_call_available_ = false;
    bool current_batch_reaches_whale_call_ = false;
    bool waiting_for_whale_call_processing_ = false;

    int steps_per_message_ = 1;
    int current_batch_steps_ = 0;

    std::vector<std::string> vehicle_ids_;
    std::unordered_map<std::string, VehicleState> vehicle_states_;
    sim_config_t sim_config_;
    sim_state_t sim_state_;
    const world_state_t *world_state_ = nullptr;
    manta_interfaces::msg::WhaleCall next_whale_call_;
    std::ofstream whale_call_log_;
    rclcpp::TimerBase::SharedPtr shutdown_timer_;

    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr calendar_start_time_pub_;
    rclcpp::Publisher<manta_interfaces::msg::SimStep>::SharedPtr sim_step_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr sim_time_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr startup_ready_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr shutdown_pub_;
    rclcpp::Publisher<manta_interfaces::msg::WhaleCall>::SharedPtr whale_call_pub_;

    std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> ready_subs_;
    std::vector<rclcpp::Subscription<manta_interfaces::msg::SimStepResult>::SharedPtr> step_result_subs_;
    rclcpp::Subscription<manta_interfaces::msg::WhaleCallProcessed>::SharedPtr whale_call_processed_sub_;

    rclcpp::Client<manta_interfaces::srv::GetNextWhaleCall>::SharedPtr next_whale_call_client_;
};

#endif