#include "sim_orchestrator/sim_orchestrator_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <stdexcept>

extern "C" {
    #include "sim_log.h"
}

SimOrchestratorNode::SimOrchestratorNode() : rclcpp::Node("sim_orchestrator") {            
            std_msgs::msg::Int64 calendar_start_time_message;
            std_msgs::msg::Float64 sim_time_message;
            std_msgs::msg::Bool startup_message;

            RCLCPP_INFO(get_logger(), "Initializing simulation orchestrator.");

            vehicle_ids_ = declare_parameter<std::vector<std::string>>("vehicle_ids", std::vector<std::string>{});
            if (vehicle_ids_.empty()) throw std::runtime_error("The vehicle_ids parameter must contain at least one vehicle.");
            for (const std::string &vehicle_id : vehicle_ids_) {
                const bool inserted = vehicle_states_.emplace(vehicle_id, VehicleState{}).second;
                if (!inserted) throw std::runtime_error("Duplicate vehicle ID: " + vehicle_id);
            }

            RCLCPP_INFO(get_logger(), "Configured %zu simulation vehicles.", vehicle_ids_.size());

            // Open the standard results file containing every published whale call.
            const std::string data_directory = declare_parameter<std::string>("data_directory", "/odyssey/output/Data");
            std::filesystem::create_directories(data_directory);
            whale_call_log_.open(data_directory + "/whale_calls.csv", std::ios::out | std::ios::trunc);
            if (!whale_call_log_.is_open()) throw std::runtime_error("Failed to open whale call results file.");
            whale_call_log_ << "cycle,sim_time,whale_id,north,east,depth\n";

            // ----------------------------------------------------------------- 
            // QoS 
            // -----------------------------------------------------------------
            // Startup messages should be retained for late subscribers.
            rclcpp::QoS startup_qos(1);
            startup_qos.transient_local();
            startup_qos.reliable();

            // Retain the current step or result until its subscriber has discovered the publisher.
            rclcpp::QoS step_qos(1);
            step_qos.transient_local();
            step_qos.reliable();

            // ----------------------------------------------------------------- 
            // Create publishers 
            // -----------------------------------------------------------------
            calendar_start_time_pub_ = create_publisher<std_msgs::msg::Int64>("/sim/calendar_start_time", startup_qos);
            startup_ready_pub_ = create_publisher<std_msgs::msg::Bool>("/sim/startup_ready", startup_qos);
            sim_step_pub_ = create_publisher<manta_interfaces::msg::SimStep>("/sim/step", step_qos);
            sim_time_pub_ = create_publisher<std_msgs::msg::Float64>("/sim/time", startup_qos);
            shutdown_pub_ = create_publisher<std_msgs::msg::Bool>("/sim/shutdown", startup_qos);
            whale_call_pub_ = create_publisher<manta_interfaces::msg::WhaleCall>( "/sim/whales/calls", 100);

            // ----------------------------------------------------------------- 
            // Create subscribers
            // -----------------------------------------------------------------
            // Ready subscription
            for (const std::string &vehicle_id : vehicle_ids_) {
                const std::string ready_topic = "/" + vehicle_id + "/ready";
                auto subscription = create_subscription<std_msgs::msg::Bool>(
                    ready_topic, startup_qos, [this, vehicle_id](const std_msgs::msg::Bool::SharedPtr msg) {handle_vehicle_ready(msg, vehicle_id);}
                );
                ready_subs_.push_back(subscription);
            }

            // Step result subscription
            for (const std::string &vehicle_id : vehicle_ids_) {
                const std::string result_topic = "/" + vehicle_id + "/step_result";
                auto subscription = create_subscription<manta_interfaces::msg::SimStepResult>(
                    result_topic, step_qos, [this, vehicle_id](const manta_interfaces::msg::SimStepResult::SharedPtr msg) {handle_step_result(msg, vehicle_id);}
                );
                step_result_subs_.push_back(subscription);
            }

            whale_call_processed_sub_ = create_subscription<manta_interfaces::msg::WhaleCallProcessed>(
                "/sim/whales/call_processed", 100, [this](const manta_interfaces::msg::WhaleCallProcessed::SharedPtr msg) {handle_whale_call_processed(msg);}
            );

            // ----------------------------------------------------------------- 
            // Create clients
            // -----------------------------------------------------------------
            next_whale_call_client_ = create_client<manta_interfaces::srv::GetNextWhaleCall>("/sim/whales/get_next_call");

            // ----------------------------------------------------------------- 
            // Open sim log
            // -----------------------------------------------------------------
            if (open_sim_log()) RCLCPP_INFO(get_logger(), "Simulation log opened.");
            else throw std::runtime_error("Failed to open simulation log.");

            // ----------------------------------------------------------------- 
            // Init sim config
            // -----------------------------------------------------------------
            if (init_sim_config(&sim_config_)) RCLCPP_INFO(get_logger(), "Simulation configuration initialized.");
            else throw std::runtime_error("Failed to initialize simulation configuration.");

            sim_config_.timestep = declare_parameter<double>("timestep", sim_config_.timestep);
            sim_config_.control_period = static_cast<int>(declare_parameter<std::int64_t>("control_period", sim_config_.control_period));
            sim_config_.num_cycles = static_cast<int>(declare_parameter<std::int64_t>("max_cycles", sim_config_.num_cycles));
            steps_per_message_ = static_cast<int>(declare_parameter<std::int64_t>("steps_per_message", 1));

            if (steps_per_message_ < 1) throw std::runtime_error("steps_per_message must be at least 1.");

            // ----------------------------------------------------------------- 
            // Init world state
            // -----------------------------------------------------------------
            if (init_world_sim()) RCLCPP_INFO(get_logger(), "World simulation initialized.");
            else throw std::runtime_error("Failed to initialize world simulation.");
            world_state_ = get_world_state();
            if (world_state_ == nullptr) throw std::runtime_error("Failed to get world state.");
            
            // ----------------------------------------------------------------- 
            // Init sim state
            // -----------------------------------------------------------------
            if (init_sim_state(&sim_state_)) RCLCPP_INFO(get_logger(), "Simulation state initialized.");
            else throw std::runtime_error("Failed to initialize simulation state.");

            RCLCPP_INFO(get_logger(), "Simulation orchestrator initialized and ready.");

            // ----------------------------------------------------------------- 
            // Publish that Odysseys can begin init
            // -----------------------------------------------------------------
            startup_message.data = true;
            startup_ready_pub_->publish(startup_message);
            RCLCPP_INFO(get_logger(), "Ready to begin Odyssey initialization.");

            // ----------------------------------------------------------------- 
            // Publish sim time
            // -----------------------------------------------------------------
            sim_time_message.data = sim_state_.sim_time;
            sim_time_pub_->publish(sim_time_message);

            // ----------------------------------------------------------------- 
            // Publish calendar start time
            // -----------------------------------------------------------------
            calendar_start_time_message.data = static_cast<std::int64_t>(sim_state_.calendar_start_time);
            calendar_start_time_pub_->publish(calendar_start_time_message);
        }

SimOrchestratorNode::~SimOrchestratorNode() {
            RCLCPP_INFO(get_logger(), "Shutting down simulation orchestrator.");
            if (!close_sim_log()) RCLCPP_ERROR(get_logger(), "Failed to close simulation log.");
        }

bool SimOrchestratorNode::simulation_failed() const {
            return simulation_failed_;
        }

void SimOrchestratorNode::request_next_whale_call(double after_time) {
            if (!next_whale_call_client_->wait_for_service(std::chrono::seconds(5))) {
                RCLCPP_ERROR(get_logger(), "Whale-call scheduling service is unavailable.");
                request_simulation_shutdown(true);
                return;
            }

            auto request = std::make_shared<manta_interfaces::srv::GetNextWhaleCall::Request>();

            // Request the first call strictly after the event we just handled.
            request->after_time = after_time;

            next_whale_call_client_->async_send_request(
                request, [this](rclcpp::Client<manta_interfaces::srv::GetNextWhaleCall>::SharedFuture future) {handle_next_whale_call_response(future);}
            );
        }

void SimOrchestratorNode::publish_whale_call(const manta_interfaces::msg::WhaleCall &call) {
            // Publish the event for acoustic sensors and autonomy nodes.
            whale_call_pub_->publish(call);

            // Save the exact published event for the review plot.
            whale_call_log_
                << call.cycle << ','
                << std::setprecision(17) << call.sim_time << ','
                << call.whale_id << ','
                << call.north << ','
                << call.east << ','
                << call.depth << '\n';
        }

void SimOrchestratorNode::check_all_vehicles_initialized() {
            if (simulation_started_) return;

            for (const auto &entry : vehicle_states_) {
                const VehicleState &state = entry.second;
                if (!state.ready) return;
            }

            simulation_started_ = true;
            RCLCPP_INFO(get_logger(), "All %zu vehicles are initialized.", vehicle_states_.size());
            request_next_whale_call(sim_state_.sim_time);
        }

void SimOrchestratorNode::publish_current_step() {
            manta_interfaces::msg::SimStep step;

            // Describe the first cycle, starting time, and physics timestep of the batch that the vehicle simulators are about to process.
            step.current_cycle = static_cast<std::uint64_t>(sim_state_.current_cycle);
            step.sim_time = sim_state_.sim_time;
            step.timestep = sim_config_.timestep;

            // Use the configured maximum batch size unless fewer simulation cycles remain.
            const int remaining_cycles = sim_config_.num_cycles - sim_state_.current_cycle;
            int batch_steps = std::min(steps_per_message_, remaining_cycles);

            current_batch_reaches_whale_call_ = false;
            if (next_whale_call_available_) {
                // Calculate how many simulated seconds remain before the next scheduled whale call.
                const double time_until_call = next_whale_call_.sim_time - sim_state_.sim_time;

                if (time_until_call > 0.0) {
                    // Convert the remaining time to a number of physics steps. Every published batch must contain at least one step.
                    const int steps_until_call = std::max(1, static_cast<int>(std::ceil(time_until_call / sim_config_.timestep)));

                    // If the call occurs within the normal batch, shorten the batch so it ends at the call.
                    if (steps_until_call <= batch_steps) {
                        batch_steps = steps_until_call;
                        current_batch_reaches_whale_call_ = true;
                    }
                }
            }

            // Send the selected number of internal physics steps to every vehicle simulator.
            current_batch_steps_ = batch_steps;
            step.step_count = static_cast<std::uint32_t>(batch_steps);
            step.control_period = static_cast<std::uint64_t>(sim_config_.control_period);
            step.world.gravity = world_state_->gravity;
            step.world.magnetic_north = world_state_->magnetic_north;
            step.world.magnetic_east = world_state_->magnetic_east;
            step.world.magnetic_down = world_state_->magnetic_down;
            step.world.seawater_density = world_state_->seawater_density;

            sim_step_pub_->publish(step);
        }

void SimOrchestratorNode::check_all_step_results_received() {
            // Wait until every active vehicle has returned a result for the current simulation cycle.
            for (const auto &entry : vehicle_states_) {
                const VehicleState &state = entry.second;
                if (state.active && !state.step_complete) return;
            }

            // Determine whether any vehicle still has an active mission.
            bool any_vehicle_active = false;

            for (const auto &entry : vehicle_states_) {
                if (entry.second.active) {
                    any_vehicle_active = true;
                    break;
                }
            }

            if (!any_vehicle_active) {
                RCLCPP_INFO(get_logger(), "All vehicle missions have completed.");
                request_simulation_shutdown();
                return;
            }

            // Advance by the actual batch size that was published. This may be smaller than steps_per_message_ when a whale call occurs mid-batch.
            const int completed_steps = current_batch_steps_;

            // Stop when the completed batch reaches the configured cycle limit.
            if (sim_state_.current_cycle + completed_steps >= sim_config_.num_cycles) {
                RCLCPP_INFO(get_logger(), "Simulation completed all %d configured cycles.", sim_config_.num_cycles);
                request_simulation_shutdown();
                return;
            }

            // Reset acknowledgements before publishing the next batch.
            for (auto &entry : vehicle_states_) entry.second.step_complete = false;

            // Advance the shared clock through every completed native timestep.
            for (int step = 0; step < completed_steps; ++step) {
                if (!update_sim_state(&sim_state_, sim_config_.timestep)) {
                    RCLCPP_ERROR(get_logger(), "Failed to advance the simulation state.");
                    request_simulation_shutdown(true);
                    return;
                }
            }

            // A shortened batch has now brought every vehicle to the pending whale-call event. Publish the call before starting another batch.
            if (current_batch_reaches_whale_call_) {
                next_whale_call_.cycle = static_cast<std::uint64_t>(sim_state_.current_cycle - 1);

                publish_whale_call(next_whale_call_);

                next_whale_call_available_ = false;
                current_batch_reaches_whale_call_ = false;
                waiting_for_whale_call_processing_ = true;
                return;
            }

            // The pending call was farther away than one maximum-sized batch. Continue toward the already known event without requesting it again.
            publish_current_step();
        }

void SimOrchestratorNode::request_simulation_shutdown(bool failed) {            
            simulation_failed_ = failed;
            std_msgs::msg::Bool shutdown_message;
            shutdown_message.data = true;

            RCLCPP_INFO(get_logger(), "Requesting shutdown of all simulation nodes.");
            shutdown_pub_->publish(shutdown_message);

            // Allow the reliable shutdown message time to reach the vehicle nodes before stopping the orchestrator's ROS context.
            shutdown_timer_ = create_wall_timer(
                std::chrono::milliseconds(500), [this]() {handle_shutdown_timer();}
            );
        }


int main(int argc, char *argv[]) {    
    int exit_status = EXIT_SUCCESS;
    rclcpp::init(argc, argv);   // Initialize ROS 2

    try {
        auto node = std::make_shared<SimOrchestratorNode>();  
        rclcpp::spin(node);      
        if (node->simulation_failed()) exit_status = EXIT_FAILURE;
    }
    catch (const std::exception &error) {
        RCLCPP_FATAL(rclcpp::get_logger("sim_orchestrator_node"), "%s", error.what());   // Get error message
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }

    rclcpp::shutdown();
    return exit_status;
}