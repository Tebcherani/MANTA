#include "sim_orchestrator/sim_orchestrator_node.hpp"

#include <cstdint>
#include <cmath>
#include <limits>
#include <filesystem>
#include <stdexcept>

extern "C" {
    #include "sim_log.h"
}

/**
 * Initialize the native simulation configuration and apply ROS parameter overrides.
 *
 * The native configuration provides default values. ROS parameters then replace those defaults with values generated from the MANTA configuration.
 *
 * @throws std::runtime_error If native initialization fails or the configured number of steps per message is invalid.
 */
void SimOrchestratorNode::initialize_simulation_config() {
    wait_for_bearing_processing_ = declare_parameter<bool>("wait_for_bearing_processing", false);
    
    // Initialize the configuration with the native simulator defaults.
    if (init_sim_config(&sim_config_)) RCLCPP_INFO(get_logger(), "Simulation configuration initialized.");
    else throw std::runtime_error("Failed to initialize simulation configuration.");

    // Override the native defaults with the configured ROS parameters.
    sim_config_.timestep = declare_parameter<double>("timestep", sim_config_.timestep);
    sim_config_.control_period = static_cast<int>(declare_parameter<std::int64_t>("control_period", sim_config_.control_period));
    sim_config_.num_cycles = static_cast<int>(declare_parameter<std::int64_t>("max_steps", sim_config_.num_cycles));
    
    steps_per_message_ = static_cast<int>(declare_parameter<std::int64_t>("steps_per_message", 1));
    if (steps_per_message_ < 1) throw std::runtime_error("steps_per_message must be at least 1.");

    const double communication_interval = declare_parameter<double>("communication_interval", 0.0);
    if (!std::isfinite(communication_interval) || communication_interval < 0.0) throw std::runtime_error("communication_interval must be finite and nonnegative.");

    if (communication_interval > 0.0) {
        if (!std::isfinite(sim_config_.timestep) || sim_config_.timestep <= 0.0) throw std::runtime_error("timestep must be finite and positive.");

        const double interval_steps = communication_interval / sim_config_.timestep;
        const double rounded_steps = std::round(interval_steps);

        if (!std::isfinite(interval_steps) ||
            rounded_steps < 1.0 ||
            rounded_steps > std::numeric_limits<int>::max() ||
            std::abs(interval_steps - rounded_steps) > 1.0e-6) {
            throw std::runtime_error("communication_interval must be a whole number of simulation timesteps.");
        }

        communication_steps_ = static_cast<int>(rounded_steps);
    }
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
    whale_call_log_ << "cycle,sim_time_s,whale_id,north,east,depth\n";
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
    sim_time_msg.data = sim_state_.sim_time_s;
    sim_time_pub_->publish(sim_time_msg);

    // Publish the calendar timestamp corresponding to simulation time zero.
    calendar_start_time_msg.data = static_cast<std::int64_t>(sim_state_.calendar_start_time);
    calendar_start_time_pub_->publish(calendar_start_time_msg);    
}