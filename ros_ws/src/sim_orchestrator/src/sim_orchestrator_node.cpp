#include <cstdlib>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"


class SimOrchestratorNode : public rclcpp::Node {
    public:
        SimOrchestratorNode() : Node("sim_orchestrator") {
            RCLCPP_INFO(get_logger(), "Initializing simulation orchestrator.");

            rclcpp::QoS ready_qos(1);
            ready_qos.transient_local();
            ready_qos.reliable();

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