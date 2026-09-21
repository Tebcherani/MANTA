#include "sim_orchestrator/sim_orchestrator_node.hpp"

#include <cstdint>
#include <filesystem>
#include <stdexcept>

extern "C" {
    #include "sim_log.h"
}

//-------------------------------------------------------------
// Configuration
//-------------------------------------------------------------

/**
 * Initialize the native simulation configuration and apply ROS parameter overrides.
 *
 * The native configuration provides default values. ROS parameters then replace those defaults with values generated from the MANTA configuration.
 *
 * @throws std::runtime_error If native initialization fails or the configured number of steps per message is invalid.
 */
void SimOrchestratorNode::initialize_simulation_config() {
    // Initialize the configuration with the native simulator defaults.
    if (init_sim_config(&sim_config_)) RCLCPP_INFO(get_logger(), "Simulation configuration initialized.");
    else throw std::runtime_error("Failed to initialize simulation configuration.");

    // Override the native defaults with the configured ROS parameters.
    sim_config_.timestep = declare_parameter<double>("timestep", sim_config_.timestep);
    sim_config_.control_period = static_cast<int>(declare_parameter<std::int64_t>("control_period", sim_config_.control_period));
    sim_config_.num_cycles = static_cast<int>(declare_parameter<std::int64_t>("max_cycles", sim_config_.num_cycles));
    steps_per_message_ = static_cast<int>(declare_parameter<std::int64_t>("steps_per_message", 1));

    // Every published batch must contain at least one simulation step.
    if (steps_per_message_ < 1) throw std::runtime_error("steps_per_message must be at least 1.");
}

/**
 * Initialize the native world and shared simulation state.
 *
 * @throws std::runtime_error If either native subsystem cannot be initialized.
 */
void SimOrchestratorNode::initialize_simulation_state() {
    // Initialize the native environmental world model.    
    if (!init_world_sim()) throw std::runtime_error("Failed to initialize world simulation.");

    // Obtain the world state used in every published simulation step.
    world_state_ = get_world_state();
    if (world_state_ == nullptr) throw std::runtime_error("Failed to get world state.");
          
    // Initialize the shared simulation clock and cycle state.
    if (!init_sim_state(&sim_state_)) throw std::runtime_error("Failed to initialize simulation state.");
    RCLCPP_INFO(get_logger(), "Simulation state initialized.");
}

/**
 * Load the configured vehicle IDs and initialize their orchestration state.
 */
void SimOrchestratorNode::load_vehicle_config() {
    // Load the list of vehicles participating in the simulation.
    vehicle_ids_ = declare_parameter<std::vector<std::string>>("vehicle_ids", std::vector<std::string>{});
    if (vehicle_ids_.empty()) throw std::runtime_error("The vehicle_ids parameter must contain at least one vehicle.");
    
    // Create one state entry for every uniquely identified vehicle.
    for (const std::string &vehicle_id : vehicle_ids_) {
        const bool inserted = vehicle_states_.emplace(vehicle_id, VehicleState{}).second;
        if (!inserted) throw std::runtime_error("Duplicate vehicle ID: " + vehicle_id);
    }

    RCLCPP_INFO(get_logger(), "Configured %zu simulation vehicles.", vehicle_ids_.size());
}

/**
 * Open the native simulator log.
 *
 * @throws std::runtime_error If the native simulator cannot open its log.
 */
void SimOrchestratorNode::open_simulation_log() {
    if (!open_sim_log()) throw std::runtime_error("Failed to open simulation log.");
    RCLCPP_INFO(get_logger(), "Simulation log opened.");
}

/**
 * Open the CSV file used to record every published whale call.
 */
void SimOrchestratorNode::open_whale_call_log() {
    // Load the directory where simulation results will be written.
    const std::string data_directory = declare_parameter<std::string>("data_directory", "/odyssey/output/Data");

    // Ensure the output directory exists before opening the results file.
    std::filesystem::create_directories(data_directory);
    whale_call_log_.open(data_directory + "/whale_calls.csv", std::ios::out | std::ios::trunc);
    if (!whale_call_log_.is_open()) throw std::runtime_error("Failed to open whale call results file.");

    // Write the CSV header before any whale-call records are produced.
    whale_call_log_ << "cycle,sim_time,whale_id,north,east,depth\n";
}

/**
 * Publish the initial state required by the vehicle simulation nodes.
 */
void SimOrchestratorNode::publish_startup_state() {
    std_msgs::msg::Bool startup_msg;
    std_msgs::msg::Float64 sim_time_msg;
    std_msgs::msg::Int64 calendar_start_time_msg;

    // Permit the Odyssey nodes to begin their native initialization.
    startup_msg.data = true;
    startup_ready_pub_->publish(startup_msg);
    RCLCPP_INFO(get_logger(), "Ready to begin Odyssey initialization.");

    // Publish the initial elapsed simulation time.
    sim_time_msg.data = sim_state_.sim_time;
    sim_time_pub_->publish(sim_time_msg);

    // Publish the calendar timestamp corresponding to simulation time zero.
    calendar_start_time_msg.data = static_cast<std::int64_t>(sim_state_.calendar_start_time);
    calendar_start_time_pub_->publish(calendar_start_time_msg);    
}

//-------------------------------------------------------------
// ROS Interfaces
//-------------------------------------------------------------

/**
 * Create service clients used by the simulation orchestrator.
 */
void SimOrchestratorNode::create_clients() {
    next_whale_call_client_ = create_client<manta_interfaces::srv::GetNextWhaleCall>("/sim/whales/get_next_call");  // Request the first scheduled whale call after a given simulation time.
}

/**
 * Create publishers used to coordinate the simulation and whale-call events.
 *
 * Simulation coordination topics retain their latest message so nodes that subscribe during startup can receive the current simulation state. Whale
 * calls use a deeper queue because multiple event messages may be processed.
 *
 * @param qos Reliable, retained QoS used by simulation coordination topics.
 */
void SimOrchestratorNode::create_publishers(const rclcpp::QoS &qos) {
    calendar_start_time_pub_ = create_publisher<std_msgs::msg::Int64>("/sim/calendar_start_time", qos); // Publish the calendar timestamp used to initialize the vehicle simulators.
    shutdown_pub_ = create_publisher<std_msgs::msg::Bool>("/sim/shutdown", qos);                        // Request an orderly shutdown of every simulation node.
    sim_step_pub_ = create_publisher<manta_interfaces::msg::SimStep>("/sim/step", qos);                 // Publish batches of simulation cycles for each node to process.
    sim_time_pub_ = create_publisher<std_msgs::msg::Float64>("/sim/time", qos);                         // Publish the initial shared simulation time.
    startup_ready_pub_ = create_publisher<std_msgs::msg::Bool>("/sim/startup_ready", qos);              // Notify simulation nodes that coordinated startup is complete.
    whale_call_pub_ = create_publisher<manta_interfaces::msg::WhaleCall>("/sim/whales/calls", 100);     // Publish individual whale-call events for the detection node to evaluate.
}

/**
 * Create the reliable, retained QoS used by simulation coordination topics.
 *
 * A queue depth of one retains only the latest message. Transient-local durability makes that message available to late subscribers, and reliable
 * delivery prevents it from being silently dropped.
 */
rclcpp::QoS SimOrchestratorNode::create_qos() const {
    rclcpp::QoS qos(1);
    qos.transient_local();
    qos.reliable();
    return qos;
}

/**
 * Create all ROS publishers and subscribers used by the simulation orchestrator node.
 */
void SimOrchestratorNode::create_ros_interfaces() {
    const rclcpp::QoS qos = create_qos();
    create_publishers(qos);
    create_subscribers(qos);
    create_clients();
}

/**
 * Create subscriptions for vehicle readiness, step results, and whale-call
 * processing acknowledgements.
 *
 * Each configured vehicle has its own readiness and step-result topics.
 * Whale-call processing acknowledgements are shared across the simulation.
 *
 * @param qos Reliable, retained QoS used by simulation coordination topics.
 */
void SimOrchestratorNode::create_subscribers(const rclcpp::QoS &qos) {
    // Create the readiness and step-result subscriptions for every vehicle.
    for (const std::string &vehicle_id : vehicle_ids_) {
        const std::string ready_topic = "/" + vehicle_id + "/ready";
        
        // Receive notification that this vehicle has completed initialization.
        auto subscription = create_subscription<std_msgs::msg::Bool>(
            ready_topic, qos, [this, vehicle_id](const std_msgs::msg::Bool::SharedPtr msg) {handle_vehicle_ready(msg, vehicle_id);}
        );
        ready_subs_.push_back(subscription);
    }

    for (const std::string &vehicle_id : vehicle_ids_) {
        const std::string result_topic = "/" + vehicle_id + "/step_result";
        // Receive this vehicle's result for each simulation batch.
        auto subscription = create_subscription<manta_interfaces::msg::SimStepResult>(
            result_topic, qos, [this, vehicle_id](const manta_interfaces::msg::SimStepResult::SharedPtr msg) {handle_step_result(msg, vehicle_id);}
        );
        step_result_subs_.push_back(subscription);
    }

    // Receive confirmation that the current whale call has been evaluated.
    whale_call_processed_sub_ = create_subscription<manta_interfaces::msg::WhaleCallProcessed>(
        "/sim/whales/call_processed", 100, [this](const manta_interfaces::msg::WhaleCallProcessed::SharedPtr msg) {handle_whale_call_processed(msg);}
    );
}