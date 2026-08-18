#include <cstdlib>
#include <ctime>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

extern "C" {
    #include "odyssey_sim.h"
}


class OdysseyNode : public rclcpp::Node {
    public:
        // Create the underlying ROS node and name it odyssey_simulator, then execute my Odyssey-specific initialization.
        OdysseyNode() : Node("odyssey_simulator") {
            const char *vehicle_id_env;
            const char *mission_file_env;
            const char *fault_file_env;

            vehicle_id_env = std::getenv("ODYSSEY_VEHICLE_ID");
            if (vehicle_id_env == nullptr || vehicle_id_env[0] == '\0') throw std::runtime_error("ODYSSEY_VEHICLE_ID is not set.");
            vehicle_id_ = vehicle_id_env;

            mission_file_env = std::getenv("ODYSSEY_MISSION_FILE");
            if (mission_file_env == nullptr || mission_file_env[0] == '\0') throw std::runtime_error("ODYSSEY_MISSION_FILE is not set.");
            mission_file_ = mission_file_env;

            fault_file_env = std::getenv("ODYSSEY_FAULT_FILE");
            if (fault_file_env != nullptr && fault_file_env[0] != '\0') fault_file_ = fault_file_env;

            RCLCPP_INFO(get_logger(), "Initializing vehicle '%s'.", vehicle_id_.c_str());

            if (!init_odyssey_sim()) throw std::runtime_error("Failed to initialize Odyssey simulator.");

            const time_t calendar_start_time = std::time(nullptr);  // Temporarily use current time
            const char *fault_file = fault_file_.empty() ? nullptr : fault_file_.c_str();   // Convert C++ string to char *
            if (!start_odyssey_sim(mission_file_.c_str(), fault_file, calendar_start_time)) throw std::runtime_error("Failed to start Odyssey simulator.");

            RCLCPP_INFO(get_logger(), "Vehicle '%s' initialized and ready.", vehicle_id_.c_str());
        }

        // Destructor
        ~OdysseyNode() {
            RCLCPP_INFO(get_logger(), "Shutting down vehicle '%s'.", vehicle_id_.c_str());
        }

    private:
        std::string vehicle_id_;
        std::string mission_file_;
        std::string fault_file_;
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