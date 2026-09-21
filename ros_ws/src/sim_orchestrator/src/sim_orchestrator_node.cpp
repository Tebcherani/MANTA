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
    RCLCPP_INFO(get_logger(), "Initializing simulation orchestrator.");

    load_vehicle_config();
    open_whale_call_log();
    create_ros_interfaces();
    open_simulation_log();
    initialize_simulation_config();
    initialize_simulation_state();

    RCLCPP_INFO(get_logger(), "Simulation orchestrator initialized and ready.");

    publish_startup_state();
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