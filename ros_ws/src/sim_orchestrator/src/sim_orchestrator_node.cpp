#include <cstdlib>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

extern "C" {
    #include "sim_log.h"
}


class SimOrchestratorNode : public rclcpp::Node {
    public:
        explicit SimOrchestratorNode() : Node("sim_orchestrator") {
            std_msgs::msg::Bool startup_message;
            
            RCLCPP_INFO(get_logger(), "Initializing simulation orchestrator.");

            if (open_sim_log()) RCLCPP_INFO(get_logger(), "Simulation log opened.");
            else throw std::runtime_error("Failed to open simulation log.");

            rclcpp::QoS ready_qos(1);
            ready_qos.transient_local();
            ready_qos.reliable();

            startup_ready_pub_ = create_publisher<std_msgs::msg::Bool>("/sim/startup_ready", ready_qos);
            startup_message.data = true;
            startup_ready_pub_->publish(startup_message);

            RCLCPP_INFO(get_logger(), "Ready to begin Odyssey initialization.");

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

            RCLCPP_INFO(get_logger(), "Simulation orchestrator initialized and ready.");
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