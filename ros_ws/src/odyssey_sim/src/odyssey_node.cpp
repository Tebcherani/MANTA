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

extern "C" {
    #include "odyssey_sim.h"
}


class OdysseyNode : public rclcpp::Node {
    public:
        OdysseyNode() : Node("odyssey_sim") {
            const char *vehicle_id_env;

            // ----------------------------------------------------------------- 
            // Get vehicle ID 
            // -----------------------------------------------------------------
            vehicle_id_env = std::getenv("ODYSSEY_VEHICLE_ID");
            if (vehicle_id_env == nullptr || vehicle_id_env[0] == '\0') throw std::runtime_error("ODYSSEY_VEHICLE_ID is not set.");
            vehicle_id_ = vehicle_id_env;

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

            sim_step_sub_ = create_subscription<manta_interfaces::msg::SimStep>(
                "/sim/step", step_qos, [this](const manta_interfaces::msg::SimStep::SharedPtr msg) {
                    manta_interfaces::msg::SimStepResult result;

                    RCLCPP_INFO(get_logger(), "%s received cycle %lu at sim time %.2f", vehicle_id_.c_str(), static_cast<unsigned long>(msg->current_cycle), msg->sim_time);

                    result.vehicle_id = vehicle_id_;
                    result.completed_cycle = msg->current_cycle;
                    result.status = manta_interfaces::msg::SimStepResult::RUNNING;
                    result.detail = "";

                    step_result_pub_->publish(result);
            });

            sim_time_sub_ = create_subscription<std_msgs::msg::Float64>(
                "/sim/time", startup_qos, [this](const std_msgs::msg::Float64::SharedPtr msg) {
                    sim_time_ = msg->data;
                    sim_time_received_ = true;

                    try_finish_init();
                }
            );

            RCLCPP_INFO(get_logger(), "%s waiting for simulation startup data.", vehicle_id_.c_str());
        }

        // Destructor
        ~OdysseyNode() {
            RCLCPP_INFO(get_logger(), "Shutting down vehicle '%s'.", vehicle_id_.c_str());
        }

    private:
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

        rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr calendar_start_time_sub_;
        rclcpp::Subscription<manta_interfaces::msg::SimStep>::SharedPtr sim_step_sub_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sim_time_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr startup_sub_;

        bool calendar_start_time_received_ = false;
        bool odyssey_initialized_ = false;
        bool ready_published_ = false;
        bool sim_time_initialized_ = false;
        bool sim_time_received_ = false;
        bool odyssey_sim_started_ = false;

        time_t calendar_start_time_ = 0;
        double sim_time_ = 0.0;
        std::string vehicle_id_;
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