#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <stdexcept>
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
#include "manta_interfaces/srv/get_next_whale_call.hpp"
#include "manta_interfaces/msg/whale_call_processed.hpp"

extern "C" {
    #include "sim_config.h"
    #include "sim_log.h"
    #include "sim_state.h"
    #include "world_sim.h"
}

struct VehicleState {
    bool ready = false;
    bool active = true;
    bool step_complete = false;
};

class SimOrchestratorNode : public rclcpp::Node {
    public:
        explicit SimOrchestratorNode() : Node("sim_orchestrator") {
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

            // Simulation steps should not be retained/replayed.
            rclcpp::QoS step_qos(1);
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
                    ready_topic, startup_qos, [this, vehicle_id](const std_msgs::msg::Bool::SharedPtr msg) {
                        vehicle_states_.at(vehicle_id).ready = msg->data;
                        check_all_vehicles_initialized();
                });
                ready_subs_.push_back(subscription);
            }

            // Step result subscription
            for (const std::string &vehicle_id : vehicle_ids_) {
                const std::string result_topic = "/" + vehicle_id + "/step_result";
                auto subscription = create_subscription<manta_interfaces::msg::SimStepResult>(
                    result_topic, step_qos, [this, vehicle_id](const manta_interfaces::msg::SimStepResult::SharedPtr msg) {
                        VehicleState &state = vehicle_states_.at(vehicle_id);
                        handle_step_result(msg, vehicle_id, state.active, state.step_complete);
                });
                step_result_subs_.push_back(subscription);
            }

            whale_call_processed_sub_ = create_subscription<manta_interfaces::msg::WhaleCallProcessed>("/sim/whales/call_processed", 100,
                [this](const manta_interfaces::msg::WhaleCallProcessed::SharedPtr message) {
                    if (!waiting_for_whale_call_processing_) {
                        RCLCPP_WARN(get_logger(), "Ignored unexpected whale-call acknowledgement.");
                        return;
                    }

                    if (
                        message->whale_id != next_whale_call_.whale_id ||
                        message->cycle != next_whale_call_.cycle ||
                        std::fabs(message->sim_time - next_whale_call_.sim_time) > 1.0e-9
                    ) {
                        RCLCPP_WARN(get_logger(), "Ignored acknowledgement for unexpected whale call %s at cycle %llu.",
                            message->whale_id.c_str(), static_cast<unsigned long long>(message->cycle)
                        );
                        return;
                    }

                    waiting_for_whale_call_processing_ = false;

                    // Only search for the next call after every vehicle has evaluated this one.
                    request_next_whale_call(message->sim_time);
                }
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
            sim_config_.control_period = static_cast<int>(declare_parameter<int64_t>("control_period", sim_config_.control_period));
            sim_config_.num_cycles = static_cast<int>(declare_parameter<int64_t>("max_cycles", sim_config_.num_cycles));
            steps_per_message_ = static_cast<int>(declare_parameter<int64_t>("steps_per_message", 1));

            if (steps_per_message_ < 1) throw std::runtime_error("steps_per_message must be at least 1.");

            // ----------------------------------------------------------------- 
            // Init world state
            // -----------------------------------------------------------------
            if (init_world_sim()) RCLCPP_INFO(get_logger(), "World simulation initialized.");
            else throw std::runtime_error("Failed to initialize world simulation.");
            world_state_ = get_world_state();
            if (world_state_ == NULL) throw std::runtime_error("Failed to get world state.");
            
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
            calendar_start_time_message.data = static_cast<int64_t>(sim_state_.calendar_start_time);
            calendar_start_time_pub_->publish(calendar_start_time_message);
        }

        ~SimOrchestratorNode() {
            RCLCPP_INFO(get_logger(), "Shutting down simulation orchestrator.");
            if (!close_sim_log()) RCLCPP_ERROR(get_logger(), "Failed to close simulation log.");
        }

        bool simulation_failed() const {
            return simulation_failed_;
        }

    private:
        void request_next_whale_call(double after_time) {
            if (!next_whale_call_client_->wait_for_service(std::chrono::seconds(5))) {
                RCLCPP_ERROR(get_logger(), "Whale-call scheduling service is unavailable.");
                request_simulation_shutdown(true);
                return;
            }

            auto request = std::make_shared<manta_interfaces::srv::GetNextWhaleCall::Request>();

            // Request the first call strictly after the event we just handled.
            request->after_time = after_time;

            next_whale_call_client_->async_send_request(request, [this](rclcpp::Client<manta_interfaces::srv::GetNextWhaleCall>::SharedFuture future) {
                    const auto response = future.get();

                    if (!response->available) {
                        next_whale_call_available_ = false;
                        publish_current_step();
                        return;
                    }

                    next_whale_call_ = response->call;
                    next_whale_call_available_ = true;

                    // More than one call can occur during one physics timestep.
                    // Vehicles have already reached the end of that timestep, so publish any remaining calls from it before advancing again.
                    if (next_whale_call_.sim_time <= sim_state_.sim_time) {
                        next_whale_call_.cycle = sim_state_.current_cycle > 0 ? static_cast<uint64_t>(sim_state_.current_cycle - 1) : 0;

                        publish_whale_call(next_whale_call_);
                        next_whale_call_available_ = false;
                        waiting_for_whale_call_processing_ = true;
                        return;
                    }

                    publish_current_step();
                }
            );
        }

        void publish_whale_call(const manta_interfaces::msg::WhaleCall &call) {
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

        void check_all_vehicles_initialized() {
            if (simulation_started_) return;

            for (const auto &entry : vehicle_states_) {
                const VehicleState &state = entry.second;
                if (!state.ready) return;
            }

            simulation_started_ = true;
            RCLCPP_INFO(get_logger(), "All %zu vehicles are initialized.", vehicle_states_.size());
            request_next_whale_call(sim_state_.sim_time);
        }

        void publish_current_step() {
            manta_interfaces::msg::SimStep step;

            // Describe the first cycle, starting time, and physics timestep of the batch that the vehicle simulators are about to process.
            step.current_cycle = static_cast<uint64_t>(sim_state_.current_cycle);
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
            step.step_count = static_cast<uint32_t>(batch_steps);
            step.control_period = static_cast<uint64_t>(sim_config_.control_period);
            step.world.gravity = world_state_->gravity;
            step.world.magnetic_north = world_state_->magnetic_north;
            step.world.magnetic_east = world_state_->magnetic_east;
            step.world.magnetic_down = world_state_->magnetic_down;
            step.world.seawater_density = world_state_->seawater_density;

            sim_step_pub_->publish(step);
        }

        void check_all_step_results_received() {
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
                next_whale_call_.cycle = static_cast<uint64_t>(sim_state_.current_cycle - 1);

                publish_whale_call(next_whale_call_);

                next_whale_call_available_ = false;
                current_batch_reaches_whale_call_ = false;
                waiting_for_whale_call_processing_ = true;
                return;
            }

            // The pending call was farther away than one maximum-sized batch. Continue toward the already known event without requesting it again.
            publish_current_step();
        }

        const char *status_name(uint8_t status) const {
            using SimStepResult = manta_interfaces::msg::SimStepResult;

            switch (status) {
                case SimStepResult::RUNNING: return "RUNNING";
                case SimStepResult::MISSION_COMPLETE: return "MISSION_COMPLETE";
                case SimStepResult::ABORTED: return "ABORTED";
                case SimStepResult::ERROR: return "ERROR";
                default: return "UNKNOWN";
            }
        }

        void handle_step_result(const manta_interfaces::msg::SimStepResult::SharedPtr msg, const std::string &expected_vehicle_id, bool &vehicle_active, bool &step_complete) {
            // Error handling
            if (msg->vehicle_id != expected_vehicle_id) {
                RCLCPP_WARN(get_logger(), "Ignored step result from unexpected vehicle '%s' on the %s topic.", msg->vehicle_id.c_str(), expected_vehicle_id.c_str());
                return;
            }

            const uint64_t first_cycle = static_cast<uint64_t>(sim_state_.current_cycle);
            const uint64_t last_cycle = first_cycle + static_cast<uint64_t>(current_batch_steps_) - 1;

            // Terminal results may occur partway through a batch.
            if (msg->completed_cycle < first_cycle || msg->completed_cycle > last_cycle) {
                RCLCPP_WARN(get_logger(), "Ignored result from %s for cycle %llu; expected cycle %llu through %llu.", msg->vehicle_id.c_str(),
                    static_cast<unsigned long long>(msg->completed_cycle),
                    static_cast<unsigned long long>(first_cycle),
                    static_cast<unsigned long long>(last_cycle)
                );
                return;
            }

            // A running vehicle must have completed the entire batch.
            if (msg->status == manta_interfaces::msg::SimStepResult::RUNNING && msg->completed_cycle != last_cycle) {
                RCLCPP_WARN(get_logger(), "Ignored incomplete batch result from %s at cycle %llu; expected cycle %llu.", msg->vehicle_id.c_str(),
                    static_cast<unsigned long long>(msg->completed_cycle),
                    static_cast<unsigned long long>(last_cycle)
                );
                return;
            }

            if (step_complete) {
                RCLCPP_WARN(get_logger(), "Ignored duplicate result from %s for cycle %llu.", msg->vehicle_id.c_str(), static_cast<unsigned long long>(msg->completed_cycle));
                return;
            }

            if (msg->status == manta_interfaces::msg::SimStepResult::ERROR) {
                RCLCPP_ERROR(get_logger(), "Vehicle %s failed during cycle %llu: %s", msg->vehicle_id.c_str(), static_cast<unsigned long long>(msg->completed_cycle), msg->detail.c_str());
                rclcpp::shutdown();
                return;
            }

            // Mission status handling
            if (msg->status == manta_interfaces::msg::SimStepResult::MISSION_COMPLETE) {
                RCLCPP_INFO(get_logger(), "Vehicle %s completed its mission during cycle %llu.", msg->vehicle_id.c_str(), static_cast<unsigned long long>(msg->completed_cycle));
                vehicle_active = false;
                step_complete = true;
                check_all_step_results_received();
                return;
            }

            if (msg->status == manta_interfaces::msg::SimStepResult::ABORTED) {
                RCLCPP_WARN(get_logger(), "Vehicle %s aborted its mission during cycle %llu: %s", msg->vehicle_id.c_str(), static_cast<unsigned long long>(msg->completed_cycle), msg->detail.c_str());                
                vehicle_active = false;
                step_complete = true;
                check_all_step_results_received();
                return;
            }

            // Step for this vehicle is complete
            step_complete = true;

            // Check if all vehicles have completed this step
            check_all_step_results_received();
        }

        void request_simulation_shutdown(bool failed = false) {
            simulation_failed_ = failed;
            std_msgs::msg::Bool shutdown_message;
            shutdown_message.data = true;

            RCLCPP_INFO(get_logger(), "Requesting shutdown of all simulation nodes.");
            shutdown_pub_->publish(shutdown_message);

            // Allow the reliable shutdown message time to reach the vehicle nodes before stopping the orchestrator's ROS context.
            shutdown_timer_ = create_wall_timer(
                std::chrono::milliseconds(500), [this]() {
                    shutdown_timer_->cancel(); 
                    rclcpp::shutdown();
            });
        }

        bool simulation_started_ = false;
        bool simulation_failed_ = false;
        bool next_whale_call_available_ = false;
        bool current_batch_reaches_whale_call_ = false;
        bool waiting_for_whale_call_processing_ = false;

        std::vector<std::string> vehicle_ids_;
        std::unordered_map<std::string, VehicleState> vehicle_states_;
        sim_config_t sim_config_;
        sim_state_t sim_state_;
        const world_state_t *world_state_ = nullptr;
        int steps_per_message_ = 1;
        int current_batch_steps_ = 0;
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