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
    // ---------------------------------------------------------------------
    // Structures
    // ---------------------------------------------------------------------

    struct VehicleState {
        bool ready_b = false;
        bool active_b = true;
        bool step_complete_b = false;
    };

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
    void handle_step_result(const manta_interfaces::msg::SimStepResult::SharedPtr msg, const std::string &expected_vehicle_id);
    void handle_vehicle_ready(const std_msgs::msg::Bool::SharedPtr msg, const std::string &vehicle_id);
    void handle_whale_estimator_ready(const std_msgs::msg::Bool::SharedPtr msg, const std::string &vehicle_id);
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

    // Simulation
    bool simulation_started_b_ = false;
    bool simulation_failed_b_ = false;

    int steps_per_message_ = 1;
    int communication_steps_ = 0;
    int current_batch_steps_ = 0;
    
    sim_config_t sim_config_;
    sim_state_t sim_state_;
    const world_state_t *world_state_ = nullptr;
    
    // Vehicle
    std::vector<std::string> vehicle_ids_;
    std::unordered_map<std::string, VehicleState> vehicle_states_;
    bool wait_for_whale_estimators_b_ = false;
    std::unordered_set<std::string> ready_whale_estimators_;

    // Whale calls
    bool next_whale_call_available_b_ = false;
    bool current_batch_reaches_whale_call_b_ = false;
    bool waiting_for_whale_call_processing_b_ = false;
    bool whale_detection_complete_b_ = false;
    bool wait_for_bearing_processing_b_ = false;

    std::unordered_set<std::string> expected_bearing_processors_;
    std::unordered_set<std::string> completed_bearing_processors_;

    whale_interfaces::msg::WhaleCall next_whale_call_;
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
