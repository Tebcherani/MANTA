#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int64.hpp"

#include "manta_interfaces/msg/sim_step.hpp"
#include "manta_interfaces/msg/sim_step_result.hpp"
#include "manta_interfaces/msg/vehicle_state.hpp"

extern "C" {
    #include "odyssey.h"
    #include "odyssey_sim.h"
    #include "world_sim.h"
}


class OdysseyNode : public rclcpp::Node {
    public:
        OdysseyNode() : Node("odyssey_sim") {
            const char *vehicle_id_env;

            // ----------------------------------------------------------------- 
            // Initializations
            // -----------------------------------------------------------------
            vehicle_id_env = std::getenv("ODYSSEY_VEHICLE_ID");
            if (vehicle_id_env == nullptr || vehicle_id_env[0] == '\0') throw std::runtime_error("ODYSSEY_VEHICLE_ID is not set.");
            vehicle_id_ = vehicle_id_env;

            output_sample_period_ = declare_parameter<double>("output_sample_period", 2.0);
            if (output_sample_period_ <= 0.0) throw std::runtime_error("output_sample_period must be greater than zero.");

            // ----------------------------------------------------------------- 
            // Startup/ready QoS 
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
            ready_pub_ = create_publisher<std_msgs::msg::Bool>(vehicle_id_ + "/ready", startup_qos);
            step_result_pub_ = create_publisher<manta_interfaces::msg::SimStepResult>("/" + vehicle_id_ + "/step_result", step_qos);
            vehicle_state_pub_ = create_publisher<manta_interfaces::msg::VehicleState>("/" + vehicle_id_ + "/state", step_qos);

            // ----------------------------------------------------------------- 
            // Create subscribers
            // -----------------------------------------------------------------
            calendar_start_time_sub_ = create_subscription<std_msgs::msg::Int64>( "/sim/calendar_start_time", startup_qos, [this](const std_msgs::msg::Int64::SharedPtr msg) { 
                calendar_start_time_ = static_cast<time_t>(msg->data);
                calendar_start_time_received_ = true; 
                
                try_finish_init();  
            });
            
            startup_sub_ = create_subscription<std_msgs::msg::Bool>("/sim/startup_ready", startup_qos, [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (msg->data && !odyssey_initialized_) {
                    if (!init_odyssey_sim()) throw std::runtime_error("Failed to initialize " + vehicle_id_ + ".");
                    odyssey_initialized_ = true;
                    RCLCPP_INFO(get_logger(), "Initialized '%s' simulator.", vehicle_id_.c_str());

                    try_finish_init();
                }
            });

            sim_step_sub_ = create_subscription<manta_interfaces::msg::SimStep>("/sim/step", step_qos, [this](const manta_interfaces::msg::SimStep::SharedPtr msg) {
                manta_interfaces::msg::SimStepResult result;
                world_state_t world_state;

                // Prepare the result returned to the orchestrator and the native world structure required by the existing Odyssey C simulator.
                result.vehicle_id = vehicle_id_;
                result.completed_cycle = msg->current_cycle;
                result.status = manta_interfaces::msg::SimStepResult::RUNNING;
                result.detail = "";

                // A vehicle whose mission has finished no longer participates in subsequent simulation cycles.
                if (mission_finished_) return;

                // Convert the ROS world snapshot into the native world structure required by the existing Odyssey C simulator.                    
                world_state.gravity = msg->world.gravity;
                world_state.magnetic_north = msg->world.magnetic_north;
                world_state.magnetic_east = msg->world.magnetic_east;
                world_state.magnetic_down = msg->world.magnetic_down;
                world_state.seawater_density = msg->world.seawater_density;

                // Validate the control period before using it in the modulo operation.
                if (msg->control_period == 0) {
                    result.status = manta_interfaces::msg::SimStepResult::ERROR;
                    result.detail = "Simulation control period cannot be zero.";
                    step_result_pub_->publish(result);
                    return;
                }

                // Convert the requested output period in simulated seconds into a whole number of Odyssey control updates.
                if (!log_decimation_configured_) {
                    const double control_interval = static_cast<double>(msg->control_period) * msg->timestep;
                    const unsigned int log_decimation = static_cast<unsigned int>(std::max(1.0, std::ceil(output_sample_period_ / control_interval)));

                    set_odyssey_log_decimation(log_decimation);
                    log_decimation_configured_ = true;

                    RCLCPP_INFO(get_logger(), "Vehicle '%s' will record output every %.3f simulated seconds (%u control updates).",
                        vehicle_id_.c_str(), static_cast<double>(log_decimation) * control_interval, log_decimation
                    );
                }

                // Run every native Odyssey cycle contained in this ROS batch.
                for (uint32_t offset = 0; offset < msg->step_count; ++offset) {
                    const uint64_t cycle = msg->current_cycle + offset;
                    result.completed_cycle = cycle;

                    if (odyssey_abort_requested()) {
                        mission_finished_ = true;
                        result.status = manta_interfaces::msg::SimStepResult::ABORTED;
                        result.detail = "Odyssey requested mission abort.";
                        step_result_pub_->publish(result);
                        return;
                    }

                    // Dynamics still run at the original timestep.
                    if (!update_odyssey_sim_state(&world_state, msg->timestep)) {
                        result.status = manta_interfaces::msg::SimStepResult::ERROR;
                        result.detail = "Failed to update Odyssey simulation state.";
                        step_result_pub_->publish(result);
                        return;
                    }

                    // Sensors, navigation, control, and mission logic retain their original control-cycle timing.
                    if (cycle % msg->control_period == 0) {
                        const double control_timestep = static_cast<double>(msg->control_period) * msg->timestep;
                        const odyssey_update_status_t cycle_status = update_odyssey_sim_cycle(&world_state, control_timestep);

                        if (cycle_status == ODYSSEY_UPDATE_ERROR) {
                            result.status = manta_interfaces::msg::SimStepResult::ERROR;
                            result.detail = "Failed to update Odyssey control cycle.";
                            step_result_pub_->publish(result);
                            return;
                        }

                        if (cycle_status == ODYSSEY_UPDATE_MISSION_COMPLETE) {
                            mission_finished_ = true;
                            result.status = manta_interfaces::msg::SimStepResult::MISSION_COMPLETE;
                            result.detail = "Odyssey mission completed.";
                            step_result_pub_->publish(result);
                            return;
                        }
                    }

                    const double next_sim_time = msg->sim_time + static_cast<double>(offset + 1) * msg->timestep;

                    if (!update_odyssey_sim_time(next_sim_time)) {
                        result.status = manta_interfaces::msg::SimStepResult::ERROR;
                        result.detail = "Failed to update Odyssey simulation time.";
                        step_result_pub_->publish(result);
                        return;
                    }
                }

                // Report the vehicle state at the end of this completed batch.
                result.completed_cycle = msg->current_cycle + msg->step_count - 1;
                const double completed_sim_time = msg->sim_time + static_cast<double>(msg->step_count) * msg->timestep;
                publish_vehicle_state(result.completed_cycle, completed_sim_time);

                // Publish the state before acknowledging the completed batch.
                step_result_pub_->publish(result);
            });

            sim_time_sub_ = create_subscription<std_msgs::msg::Float64>(
                "/sim/time", startup_qos, [this](const std_msgs::msg::Float64::SharedPtr msg) {
                    sim_time_ = msg->data;
                    sim_time_received_ = true;

                    try_finish_init();
            });

            shutdown_sub_ = create_subscription<std_msgs::msg::Bool>(
                "/sim/shutdown", startup_qos, [this](const std_msgs::msg::Bool::SharedPtr msg) {
                    if (!msg->data) return;
                    RCLCPP_INFO(get_logger(), "Vehicle '%s' received simulation shutdown.", vehicle_id_.c_str());
                    rclcpp::shutdown();
            });

            RCLCPP_INFO(get_logger(), "%s waiting for simulation startup data.", vehicle_id_.c_str());
        }

        // Destructor
        ~OdysseyNode() {
            RCLCPP_INFO(get_logger(), "Shutting down vehicle '%s'.", vehicle_id_.c_str());
            
            // Flush and close Odyssey's mission data file if the simulator successfully reached its started state.
            if (odyssey_sim_started_ && !end_odyssey_sim(nullptr)) {
                RCLCPP_ERROR(get_logger(), "Failed to cleanly end simulator for vehicle '%s'.", vehicle_id_.c_str());
            }
        }

    private:
        void publish_vehicle_state(uint64_t completed_cycle, double completed_sim_time) {
            manta_interfaces::msg::VehicleState state;

            state.vehicle_id = vehicle_id_;
            state.cycle = completed_cycle;
            state.sim_time = completed_sim_time;

            state.north = get_odyssey_sim_north();
            state.east = get_odyssey_sim_east();
            state.depth = get_odyssey_sim_depth();

            vehicle_state_pub_->publish(state);
        }

        void try_finish_init() {
            if (!odyssey_initialized_) return;
            if (!sim_time_received_) return;

            // Init Odyssey sim time
            if (!sim_time_initialized_) {
                if (!update_odyssey_sim_time(sim_time_)) throw std::runtime_error("Failed to initialize " + vehicle_id_ + " simulation time.");
                sim_time_initialized_ = true;
                RCLCPP_INFO(get_logger(), "Initialized '%s' simulation time.", vehicle_id_.c_str());
            }

            if (!calendar_start_time_received_) return;

            // Start Odyssey sim
            if (!odyssey_sim_started_) {
                if (!start_odyssey_sim(calendar_start_time_)) throw std::runtime_error("Failed to start " + vehicle_id_ + " simulator.");
                odyssey_sim_started_ = true;
                RCLCPP_INFO(get_logger(), "Started '%s' simulator.", vehicle_id_.c_str());
            }

            // Notify orchestrator that Odyssey init is complete
            if (!ready_published_) {
                std_msgs::msg::Bool ready_message;        
                ready_message.data = true;
                ready_pub_->publish(ready_message);
                ready_published_ = true;
                RCLCPP_INFO(get_logger(), "Vehicle '%s' initialized and ready.", vehicle_id_.c_str()); 
            }
        }

        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
        rclcpp::Publisher<manta_interfaces::msg::SimStepResult>::SharedPtr step_result_pub_;
        rclcpp::Publisher<manta_interfaces::msg::VehicleState>::SharedPtr vehicle_state_pub_;

        rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr calendar_start_time_sub_;
        rclcpp::Subscription<manta_interfaces::msg::SimStep>::SharedPtr sim_step_sub_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sim_time_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr startup_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr shutdown_sub_;

        bool calendar_start_time_received_ = false;
        bool odyssey_initialized_ = false;
        bool ready_published_ = false;
        bool sim_time_initialized_ = false;
        bool sim_time_received_ = false;
        bool odyssey_sim_started_ = false;
        bool mission_finished_ = false;
        bool log_decimation_configured_ = false;

        time_t calendar_start_time_ = 0;
        double sim_time_ = 0.0;
        std::string vehicle_id_;
        double output_sample_period_ = 2.0;
}; 

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);   // Initialize ROS 2

    try {
        auto node = std::make_shared<OdysseyNode>();    // Create Odyssey node
        rclcpp::spin(node);                             // Node listens
    }
    catch (const std::exception &error) {
        RCLCPP_FATAL(rclcpp::get_logger("odyssey_node"), "%s", error.what());   // Get error message
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}