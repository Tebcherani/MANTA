#include "odyssey_sim/odyssey_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <string>

extern "C" {
    #include "odyssey.h"
    #include "odyssey_sim.h"
    #include "world_sim.h"
}


OdysseyNode::OdysseyNode() : rclcpp::Node("odyssey_sim") {
            const char *vehicle_id_env;
            const char *data_directory_env;

            // ----------------------------------------------------------------- 
            // Initializations
            // -----------------------------------------------------------------
            vehicle_id_env = std::getenv("ODYSSEY_VEHICLE_ID");
            if (vehicle_id_env == nullptr || vehicle_id_env[0] == '\0') throw std::runtime_error("ODYSSEY_VEHICLE_ID is not set.");
            vehicle_id_ = vehicle_id_env;

            data_directory_env = std::getenv("ODYSSEY_DATA_DIR");
            if (data_directory_env == nullptr || data_directory_env[0] == '\0') throw std::runtime_error("ODYSSEY_DATA_DIR is not set.");
            data_directory_ = data_directory_env;

            output_sample_period_ = declare_parameter<double>("output_sample_period", 2.0);
            if (output_sample_period_ <= 0.0) throw std::runtime_error("output_sample_period must be greater than zero.");

            // ----------------------------------------------------------------- 
            // Startup/ready QoS 
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
            ready_pub_ = create_publisher<std_msgs::msg::Bool>(vehicle_id_ + "/ready", startup_qos);
            step_result_pub_ = create_publisher<manta_interfaces::msg::SimStepResult>("/" + vehicle_id_ + "/step_result", step_qos);
            vehicle_state_pub_ = create_publisher<manta_interfaces::msg::VehicleState>("/" + vehicle_id_ + "/state", step_qos);

            // ----------------------------------------------------------------- 
            // Create subscribers
            // -----------------------------------------------------------------
            calendar_start_time_sub_ = create_subscription<std_msgs::msg::Int64>(
                "/sim/calendar_start_time", startup_qos, [this](const std_msgs::msg::Int64::SharedPtr msg) {handle_calendar_start_time(msg);}
            );
            
            startup_sub_ = create_subscription<std_msgs::msg::Bool>(
                "/sim/startup_ready", startup_qos, [this](const std_msgs::msg::Bool::SharedPtr msg) {handle_startup(msg);}
            );

            sim_step_sub_ = create_subscription<manta_interfaces::msg::SimStep>(
                "/sim/step", step_qos, [this](const manta_interfaces::msg::SimStep::SharedPtr msg) {handle_sim_step(msg);}
            );

            sim_time_sub_ = create_subscription<std_msgs::msg::Float64>(
                "/sim/time", startup_qos, [this](const std_msgs::msg::Float64::SharedPtr msg) {handle_sim_time(msg);}
            );

            shutdown_sub_ = create_subscription<std_msgs::msg::Bool>(
                "/sim/shutdown", startup_qos, [this](const std_msgs::msg::Bool::SharedPtr msg) {handle_shutdown(msg);}
            );

            RCLCPP_INFO(get_logger(), "%s waiting for simulation startup data.", vehicle_id_.c_str());
        }

        // Destructor
OdysseyNode::~OdysseyNode() {
            RCLCPP_INFO(get_logger(), "Shutting down vehicle '%s'.", vehicle_id_.c_str());
            
            // Flush and close Odyssey's mission data file if the simulator successfully reached its started state.
            if (odyssey_sim_started_) {
                if (!write_final_metrics()) RCLCPP_ERROR(get_logger(), "Failed to write final metrics for vehicle '%s'.", vehicle_id_.c_str());
                if (!end_odyssey_sim(nullptr)) RCLCPP_ERROR(get_logger(), "Failed to cleanly end simulator for vehicle '%s'.", vehicle_id_.c_str());
            }
        }

bool OdysseyNode::write_final_metrics() {
            odyssey_metrics_t metrics;

            if (metrics_written_) return true;
            if (!get_odyssey_metrics(&metrics)) return false;

            const std::filesystem::path directory(data_directory_);
            const std::filesystem::path final_path = directory / (vehicle_id_ + "_metrics.csv");
            const std::filesystem::path temporary_path = directory / (vehicle_id_ + "_metrics.csv.tmp");

            std::error_code error;
            std::filesystem::create_directories(directory, error);
            if (error) return false;

            std::ofstream file(temporary_path, std::ios::out | std::ios::trunc);
            if (!file.is_open()) return false;

            file
                << "vehicle_id,mass,"
                << "energy_consumed,mean_speed\n";

            file
                << vehicle_id_ << ","
                << std::setprecision(15)
                << metrics.mass << ","
                << metrics.energy_consumed << ","
                << metrics.mean_speed << "\n";

            file.close();
            if (!file) return false;

            std::filesystem::remove(final_path, error);
            error.clear();
            std::filesystem::rename(temporary_path, final_path, error);
            if (error) return false;

            metrics_written_ = true;
            RCLCPP_INFO(get_logger(), "Saved final vehicle metrics: %s", final_path.string().c_str());

            return true;
        }

void OdysseyNode::publish_vehicle_state(uint64_t completed_cycle, double completed_sim_time) {
            manta_interfaces::msg::VehicleState state;

            state.vehicle_id = vehicle_id_;
            state.cycle = completed_cycle;
            state.sim_time = completed_sim_time;

            state.north = get_odyssey_sim_north();
            state.east = get_odyssey_sim_east();
            state.depth = get_odyssey_sim_depth();

            vehicle_state_pub_->publish(state);
        }

void OdysseyNode::try_finish_init() {
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