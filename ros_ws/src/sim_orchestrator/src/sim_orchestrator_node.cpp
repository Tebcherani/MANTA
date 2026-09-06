#include <cstdint>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int64.hpp"

#include "manta_interfaces/msg/sim_step.hpp"
#include "manta_interfaces/msg/sim_step_result.hpp"

extern "C" {
    #include "sim_config.h"
    #include "sim_log.h"
    #include "sim_state.h"
    #include "world_sim.h"
}


class SimOrchestratorNode : public rclcpp::Node {
    public:
        explicit SimOrchestratorNode() : Node("sim_orchestrator") {
            std_msgs::msg::Int64 calendar_start_time_message;
            std_msgs::msg::Float64 sim_time_message;
            std_msgs::msg::Bool startup_message;

            RCLCPP_INFO(get_logger(), "Initializing simulation orchestrator.");

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

            // ----------------------------------------------------------------- 
            // Create subscribers
            // -----------------------------------------------------------------
            odyssey_1_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
                "/odyssey_1/ready", startup_qos, [this](const std_msgs::msg::Bool::SharedPtr msg) {
                    odyssey_1_ready_ = msg->data;
                    check_all_vehicles_initialized();
                }
            );

            odyssey_2_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
                "/odyssey_2/ready", startup_qos, [this](const std_msgs::msg::Bool::SharedPtr msg) {
                    odyssey_2_ready_ = msg->data;
                    check_all_vehicles_initialized();
                }
            );

            odyssey_1_step_result_sub_ = create_subscription<manta_interfaces::msg::SimStepResult>(
                "/odyssey_1/step_result", step_qos, [this](const manta_interfaces::msg::SimStepResult::SharedPtr msg) {
                    handle_step_result(msg, "odyssey_1", odyssey_1_active_, odyssey_1_step_complete_);
            });

            odyssey_2_step_result_sub_ = create_subscription<manta_interfaces::msg::SimStepResult>(
                "/odyssey_2/step_result", step_qos, [this](const manta_interfaces::msg::SimStepResult::SharedPtr msg) {
                    handle_step_result(msg, "odyssey_2", odyssey_2_active_, odyssey_2_step_complete_);
            });

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

    private:
        void check_all_vehicles_initialized() {
            if (!odyssey_1_ready_ || !odyssey_2_ready_) return;
            RCLCPP_INFO(get_logger(), "All vehicles are initialized.");
            publish_current_step();
        }

        void publish_current_step() {
            manta_interfaces::msg::SimStep step;

            step.current_cycle = static_cast<uint64_t>(sim_state_.current_cycle);
            step.sim_time = sim_state_.sim_time;
            step.timestep = sim_config_.timestep;
            step.control_period = static_cast<uint64_t>(sim_config_.control_period);

            step.world.gravity = world_state_->gravity;
            step.world.magnetic_north = world_state_->magnetic_north;
            step.world.magnetic_east = world_state_->magnetic_east;
            step.world.magnetic_down = world_state_->magnetic_down;
            step.world.seawater_density = world_state_->seawater_density;

            sim_step_pub_->publish(step);
        }

        void check_all_step_results_received() {
            // Wait only for vehicles whose missions are still active.
            if ((odyssey_1_active_ && !odyssey_1_step_complete_) || (odyssey_2_active_ && !odyssey_2_step_complete_)) return;

            RCLCPP_DEBUG(get_logger(), "All active vehicles completed simulation step %d.", sim_state_.current_cycle);

            // The simulation is complete when no active vehicles remain.
            if (!odyssey_1_active_ && !odyssey_2_active_) {
                RCLCPP_INFO(get_logger(), "All vehicle missions have completed.");
                rclcpp::shutdown();
                return;
            }

            // Stop after completing the configured number of cycles.
            if (sim_state_.current_cycle + 1 >= sim_config_.num_cycles) {
                RCLCPP_INFO(get_logger(), "Simulation completed all %d configured cycles.", sim_config_.num_cycles);
                rclcpp::shutdown();
                return;
            }

            // Reset acknowledgements before publishing the next step.
            odyssey_1_step_complete_ = false;
            odyssey_2_step_complete_ = false;

            // Advance the shared simulation cycle and clock.
            if (!update_sim_state(&sim_state_, sim_config_.timestep)) {
                RCLCPP_ERROR(get_logger(), "Failed to advance the simulation state.");
                return;
            }

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

            if (msg->completed_cycle != static_cast<uint64_t>(sim_state_.current_cycle)) {
                RCLCPP_WARN(get_logger(), "Ignored result from %s for cycle %llu; current cycle is %d.", msg->vehicle_id.c_str(), static_cast<unsigned long long>(msg->completed_cycle), sim_state_.current_cycle);
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

        // Initialization state
        bool odyssey_1_ready_ = false;
        bool odyssey_2_ready_ = false;

        // Whether each vehicle is still participating in the simulation
        bool odyssey_1_active_ = true;
        bool odyssey_2_active_ = true;

        // Whether each active vehicle has returned a result for the current simulation cycle
        bool odyssey_1_step_complete_ = false;
        bool odyssey_2_step_complete_ = false;

        sim_config_t sim_config_;
        sim_state_t sim_state_;
        const world_state_t *world_state_ = nullptr;

        rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr calendar_start_time_pub_;
        rclcpp::Publisher<manta_interfaces::msg::SimStep>::SharedPtr sim_step_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr sim_time_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr startup_ready_pub_;
        
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr odyssey_1_ready_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr odyssey_2_ready_sub_;
        rclcpp::Subscription<manta_interfaces::msg::SimStepResult>::SharedPtr odyssey_1_step_result_sub_;
        rclcpp::Subscription<manta_interfaces::msg::SimStepResult>::SharedPtr odyssey_2_step_result_sub_;
};


int main(int argc, char *argv[]) {    
    rclcpp::init(argc, argv);   // Initialize ROS 2

    try {
        auto node = std::make_shared<SimOrchestratorNode>();  
        rclcpp::spin(node);                             
    }
    catch (const std::exception &error) {
        RCLCPP_FATAL(rclcpp::get_logger("sim_orchestrator_node"), "%s", error.what());   // Get error message
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}