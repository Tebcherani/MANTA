#include <cmath>
#include <cstdint>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include "manta_interfaces/msg/sim_step.hpp"
#include "manta_interfaces/msg/whale_state.hpp"
#include "manta_interfaces/msg/whale_state_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

class WhaleNode : public rclcpp::Node {
public:
    WhaleNode() : Node("whale_node"), generator_(42) {
        whale_count_ = declare_parameter<int>("count", 5);
        line_start_north_ = declare_parameter<double>("line_start_north", 0.0);
        line_start_east_ = declare_parameter<double>("line_start_east", -1000.0);
        line_end_north_ = declare_parameter<double>("line_end_north", 0.0);
        line_end_east_ = declare_parameter<double>("line_end_east", 1000.0);
        speed_min_ = declare_parameter<double>("speed_min", 1.0);
        speed_max_ = declare_parameter<double>("speed_max", 2.0);
        start_distance_ = declare_parameter<double>("start_distance", 2000.0);
        whale_depth_ = declare_parameter<double>("depth", 20.0);
        mission_duration_ = declare_parameter<double>("mission_duration", 604800.0);
        call_rate_per_hour_ = declare_parameter<double>("call_rate_per_hour", 6.4);

        initialize_whales();

        whale_states_pub_ = create_publisher<manta_interfaces::msg::WhaleStateArray>("/sim/whales/ground_truth", 10);

        step_sub_ = create_subscription<manta_interfaces::msg::SimStep>("/sim/step", 10, [this](const manta_interfaces::msg::SimStep::SharedPtr step) {
            update_whales(*step);
        });
        
        shutdown_sub_ = create_subscription<std_msgs::msg::Bool>("/sim/shutdown", 10, [this](const std_msgs::msg::Bool::SharedPtr message) {
            if (!message->data) return;
            RCLCPP_INFO(get_logger(), "Shutting down whale node.");
            rclcpp::shutdown();
        });

        RCLCPP_INFO(get_logger(), "Initialized %d whales.", whale_count_);
    }

private:
    struct Whale {
        std::string id;
        double north;
        double east;
        double depth;
        double velocity_north;
        double velocity_east;
    };

    void initialize_whales() {
        const double line_north = line_end_north_ - line_start_north_;
        const double line_east = line_end_east_ - line_start_east_;
        const double line_length = std::hypot(line_north, line_east);

        if (line_length <= 0.0) {
            throw std::runtime_error("Picket line must have nonzero length.");
        }

        const double along_north = line_north / line_length;
        const double along_east = line_east / line_length;
        const double perpendicular_north = -along_east;
        const double perpendicular_east = along_north;

        std::uniform_real_distribution<double> along_line(0.0, line_length);
        std::uniform_real_distribution<double> speed(speed_min_, speed_max_);

        for (int index = 0; index < whale_count_; ++index) {
            const double crossing_distance = along_line(generator_);
            const double crossing_north =
                line_start_north_ + along_north * crossing_distance;
            const double crossing_east =
                line_start_east_ + along_east * crossing_distance;

            const double side = index % 2 == 0 ? 1.0 : -1.0;
            const double whale_speed = speed(generator_);

            whales_.push_back({
                "whale_" + std::to_string(index + 1),
                crossing_north + side * perpendicular_north * start_distance_,
                crossing_east + side * perpendicular_east * start_distance_,
                whale_depth_,
                -side * perpendicular_north * whale_speed,
                -side * perpendicular_east * whale_speed
            });
        }
    }

    void update_whales(const manta_interfaces::msg::SimStep &step) {
        if (step.step_count == 0) {
            RCLCPP_ERROR(get_logger(), "Received a simulation batch with zero steps.");
            return;
        }

        // Constant whale velocity makes this exactly equivalent to advancing one timestep at a time.
        const double batch_timestep = static_cast<double>(step.step_count) * step.timestep;

        for (auto &whale : whales_) {
            whale.north += whale.velocity_north * batch_timestep;
            whale.east += whale.velocity_east * batch_timestep;
        }

        const uint64_t completed_cycle = step.current_cycle + step.step_count - 1;

        manta_interfaces::msg::WhaleStateArray message;
        message.cycle = completed_cycle;

        for (const auto &whale : whales_) {
            manta_interfaces::msg::WhaleState state;
            state.whale_id = whale.id;
            state.cycle = completed_cycle;
            state.north = whale.north;
            state.east = whale.east;
            state.depth = whale.depth;
            state.velocity_north = whale.velocity_north;
            state.velocity_east = whale.velocity_east;
            state.velocity_down = 0.0;
            message.whales.push_back(state);
        }

        whale_states_pub_->publish(message);
    }

    int whale_count_;
    double line_start_north_;
    double line_start_east_;
    double line_end_north_;
    double line_end_east_;
    double speed_min_;
    double speed_max_;
    double start_distance_;
    double whale_depth_;
    double mission_duration_;
    double call_rate_per_hour_;

    std::mt19937 generator_;
    std::vector<Whale> whales_;

    rclcpp::Publisher<manta_interfaces::msg::WhaleStateArray>::SharedPtr whale_states_pub_;
    rclcpp::Subscription<manta_interfaces::msg::SimStep>::SharedPtr step_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr shutdown_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WhaleNode>());
    rclcpp::shutdown();
    return 0;
}