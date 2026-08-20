#include <cstdlib>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int64.hpp"

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
            // Startup/ready QoS 
            // -----------------------------------------------------------------
            rclcpp::QoS ready_qos(1);
            ready_qos.transient_local();
            ready_qos.reliable();

            // ----------------------------------------------------------------- 
            // Create publishers 
            // -----------------------------------------------------------------
            calendar_start_time_pub_ = create_publisher<std_msgs::msg::Int64>("/sim/calendar_start_time", ready_qos);
            startup_ready_pub_ = create_publisher<std_msgs::msg::Bool>("/sim/startup_ready", ready_qos);
            sim_time_pub_ = create_publisher<std_msgs::msg::Float64>("/sim/time", ready_qos);

            // ----------------------------------------------------------------- 
            // Create subscribers
            // -----------------------------------------------------------------
            odyssey_1_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
                "/odyssey_1/ready", ready_qos, [this](const std_msgs::msg::Bool::SharedPtr msg) {
                odyssey_1_ready_ = msg->data;
                check_all_ready();
            });

            odyssey_2_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
                "/odyssey_2/ready", ready_qos, [this](const std_msgs::msg::Bool::SharedPtr msg) {
                odyssey_2_ready_ = msg->data;
                check_all_ready();
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
        }

    private:
        void check_all_ready() {
            if (odyssey_1_ready_ && odyssey_2_ready_) {
                RCLCPP_INFO(get_logger(), "All Odyssey vehicles are ready.");
            }
        }

        bool odyssey_1_ready_ = false;
        bool odyssey_2_ready_ = false;

        sim_config_t sim_config_;
        sim_state_t sim_state_;
        const world_state_t *world_state_ = nullptr;

        rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr calendar_start_time_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr sim_time_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr startup_ready_pub_;
        
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr odyssey_1_ready_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr odyssey_2_ready_sub_;
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