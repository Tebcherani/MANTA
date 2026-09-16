#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "manta_interfaces/msg/vehicle_state.hpp"
#include "manta_interfaces/msg/whale_call.hpp"
#include "manta_interfaces/msg/whale_detection.hpp"
#include "manta_interfaces/msg/whale_call_processed.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

class WhaleDetectionNode : public rclcpp::Node {
    public:
        WhaleDetectionNode() : Node("whale_detection_node") {
            // ----------------------------------------------------------------- 
            // Initializations
            // -----------------------------------------------------------------
            vehicle_ids_ = declare_parameter<std::vector<std::string>>("vehicle_ids", std::vector<std::string>{});
            detection_range_ = declare_parameter<double>("detection_range", 500.0);

            const std::string data_directory = declare_parameter<std::string>("data_directory", "/odyssey/output/Data");
            std::filesystem::create_directories(data_directory);
            detection_log_.open(data_directory + "/whale_detections.csv", std::ios::out | std::ios::trunc);
            if (!detection_log_.is_open()) throw std::runtime_error("Failed to open whale detection results file.");
            detection_log_ << "cycle,sim_time,vehicle_id,whale_id,vehicle_north,vehicle_east,vehicle_depth,whale_north,whale_east,whale_depth,distance\n";

            if (vehicle_ids_.empty()) throw std::runtime_error("vehicle_ids must contain at least one vehicle.");
            if (detection_range_ <= 0.0) throw std::runtime_error("detection_range must be greater than zero.");

            // ----------------------------------------------------------------- 
            // Create publishers
            // -----------------------------------------------------------------
            detection_pub_ = create_publisher<manta_interfaces::msg::WhaleDetection>("/sim/whale_detections", 100);
            processed_pub_ = create_publisher<manta_interfaces::msg::WhaleCallProcessed>("/sim/whales/call_processed", 100);

            // ----------------------------------------------------------------- 
            // Create subscribers
            // -----------------------------------------------------------------
            // Subscribe to the generic state published by every configured vehicle.
            for (const std::string &vehicle_id : vehicle_ids_) {
                const std::string topic = "/" + vehicle_id + "/state";

                auto subscription = create_subscription<manta_interfaces::msg::VehicleState>(topic, 10, [this, vehicle_id](const manta_interfaces::msg::VehicleState::SharedPtr message) {
                    latest_vehicle_states_[vehicle_id] = *message;
                    try_process_pending_call();
                });

                vehicle_state_subs_.push_back(subscription);
            }

            whale_call_sub_ = create_subscription<manta_interfaces::msg::WhaleCall>("/sim/whales/calls", 100, [this](const manta_interfaces::msg::WhaleCall::SharedPtr call) {
                if (pending_call_available_) {
                    RCLCPP_ERROR(get_logger(), "Received whale call %s before the previous call was processed.", call->whale_id.c_str());
                    return;
                }

                pending_call_ = *call;
                pending_call_available_ = true;
                try_process_pending_call();
            });

            shutdown_sub_ = create_subscription<std_msgs::msg::Bool>("/sim/shutdown", 10, [this](const std_msgs::msg::Bool::SharedPtr message) {
                if (!message->data) return;
                RCLCPP_INFO(get_logger(), "Shutting down whale detector.");
                rclcpp::shutdown();
            });

            RCLCPP_INFO(get_logger(), "Whale detector initialized for %zu vehicles with a %.1f m detection range.", vehicle_ids_.size(), detection_range_);
        }

    private:
        void try_process_pending_call() {
            if (!pending_call_available_) return;

            // Wait until every vehicle has published its state for this call's cycle.
            for (const std::string &vehicle_id : vehicle_ids_) {
                const auto iterator = latest_vehicle_states_.find(vehicle_id);
                if (iterator == latest_vehicle_states_.end()) return;
                if (iterator->second.cycle < pending_call_.cycle) return;

                if (iterator->second.cycle > pending_call_.cycle) {
                    RCLCPP_ERROR(get_logger(), "Vehicle %s advanced to cycle %llu before call cycle %llu was processed.",
                        vehicle_id.c_str(), static_cast<unsigned long long>(iterator->second.cycle), static_cast<unsigned long long>(pending_call_.cycle)
                    );
                    return;
                }
            }

            // Every vehicle is now at the event cycle, so evaluate the call.
            for (const std::string &vehicle_id : vehicle_ids_) {
                const auto &vehicle = latest_vehicle_states_.at(vehicle_id);

                const double north_difference = pending_call_.north - vehicle.north;
                const double east_difference = pending_call_.east - vehicle.east;
                const double depth_difference = pending_call_.depth - vehicle.depth;

                const double distance = std::sqrt(north_difference * north_difference + east_difference * east_difference + depth_difference * depth_difference);

                if (distance > detection_range_) continue;

                manta_interfaces::msg::WhaleDetection detection;
                detection.vehicle_id = vehicle_id;
                detection.whale_id = pending_call_.whale_id;
                detection.cycle = pending_call_.cycle;
                detection.sim_time = pending_call_.sim_time;
                detection.vehicle_north = vehicle.north;
                detection.vehicle_east = vehicle.east;
                detection.vehicle_depth = vehicle.depth;
                detection.whale_north = pending_call_.north;
                detection.whale_east = pending_call_.east;
                detection.whale_depth = pending_call_.depth;
                detection.distance = distance;

                detection_pub_->publish(detection);

                detection_log_
                    << detection.cycle << ','
                    << std::setprecision(17) << detection.sim_time << ','
                    << detection.vehicle_id << ','
                    << detection.whale_id << ','
                    << detection.vehicle_north << ','
                    << detection.vehicle_east << ','
                    << detection.vehicle_depth << ','
                    << detection.whale_north << ','
                    << detection.whale_east << ','
                    << detection.whale_depth << ','
                    << detection.distance << '\n';
                detection_log_.flush();

                RCLCPP_INFO(get_logger(), "Vehicle %s detected %s at %.1f m.", vehicle_id.c_str(), pending_call_.whale_id.c_str(), distance);
            }

            manta_interfaces::msg::WhaleCallProcessed processed;
            processed.whale_id = pending_call_.whale_id;
            processed.cycle = pending_call_.cycle;
            processed.sim_time = pending_call_.sim_time;

            pending_call_available_ = false;
            processed_pub_->publish(processed);
        }

        std::vector<std::string> vehicle_ids_;
        double detection_range_ = 500.0;

        bool pending_call_available_ = false;
        manta_interfaces::msg::WhaleCall pending_call_;
        std::unordered_map<std::string, manta_interfaces::msg::VehicleState> latest_vehicle_states_;
        std::ofstream detection_log_;

        rclcpp::Publisher<manta_interfaces::msg::WhaleDetection>::SharedPtr detection_pub_;
        rclcpp::Publisher<manta_interfaces::msg::WhaleCallProcessed>::SharedPtr processed_pub_;

        rclcpp::Subscription<manta_interfaces::msg::WhaleCall>::SharedPtr whale_call_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr shutdown_sub_;
        std::vector<rclcpp::Subscription<manta_interfaces::msg::VehicleState>::SharedPtr> vehicle_state_subs_;
    };

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    try {
        rclcpp::spin(std::make_shared<WhaleDetectionNode>());
    } catch (const std::exception &error) {
        RCLCPP_FATAL(rclcpp::get_logger("whale_detection_node"), "%s", error.what());
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}