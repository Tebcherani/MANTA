#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "manta_interfaces/msg/sim_step.hpp"
#include "manta_interfaces/msg/whale_call.hpp"
#include "manta_interfaces/msg/whale_state.hpp"
#include "manta_interfaces/msg/whale_state_array.hpp"
#include "manta_interfaces/srv/get_next_whale_call.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

class WhaleNode : public rclcpp::Node {
public:
    WhaleNode() : Node("whale_node"), generator_(42) {
        // ----------------------------------------------------------------- 
        // Initializations
        // -----------------------------------------------------------------
        
        // Scenario geometry and whale behavior are supplied by MANTA's generated Compose file from config/simulation.yaml.
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
        call_rate_min_ = declare_parameter<double>("call_rate_min", 4.8);
        call_rate_max_ = declare_parameter<double>("call_rate_max", 8.0);
        turn_interval_ = declare_parameter<double>("turn_interval", 60.0);
        heading_correlation_ = declare_parameter<double>("heading_correlation", 0.95);
        heading_std_degrees_ = declare_parameter<double>("heading_std_degrees", 15.0);
        group_mean_size_ = declare_parameter<double>("group_mean_size", 3.0);
        group_position_std_ = declare_parameter<double>("group_position_std", 1000.0);
        group_start_time_std_ = declare_parameter<double>("group_start_time_std", 600.0);
        shared_heading_fraction_ = declare_parameter<double>("shared_heading_fraction", 0.75);
        const std::string data_directory = declare_parameter<std::string>("data_directory", "/odyssey/output/Data");
        whale_sample_period_ = declare_parameter<double>("sample_period", 60.0);
        
        if (whale_sample_period_ <= 0.0) throw std::runtime_error("Whale sample period must be greater than zero.");
        if (speed_min_ <= 0.0) throw std::runtime_error("speed_min must be greater than zero.");
        if (speed_max_ < speed_min_) throw std::runtime_error("speed_max must be greater than or equal to speed_min.");
        if (turn_interval_ <= 0.0) throw std::runtime_error("turn_interval must be greater than zero.");
        if (heading_correlation_ < 0.0 || heading_correlation_ >= 1.0) throw std::runtime_error("heading_correlation must be in [0, 1).");
        if (heading_std_degrees_ < 0.0) throw std::runtime_error("heading_std_degrees cannot be negative.");
        if (group_mean_size_ < 1.0) throw std::runtime_error("group_mean_size must be at least one.");
        if (group_position_std_ < 0.0) throw std::runtime_error("group_position_std cannot be negative.");
        if (group_start_time_std_ < 0.0) throw std::runtime_error("group_start_time_std cannot be negative.");
        if (shared_heading_fraction_ < 0.0 || shared_heading_fraction_ > 1.0) throw std::runtime_error("shared_heading_fraction must be in [0, 1].");

        std::filesystem::create_directories(data_directory);
        whale_log_.open(data_directory + "/whales.csv", std::ios::out | std::ios::trunc);
        if (!whale_log_.is_open()) throw std::runtime_error("Failed to open whale trajectory log.");
        whale_log_ << "cycle,sim_time,whale_id,group_id,"
            << "north,east,depth\n";

        const int64_t random_seed = declare_parameter<int64_t>("random_seed", 42);
        generator_.seed(static_cast<std::mt19937::result_type>(random_seed));

        initialize_whales();

        // ----------------------------------------------------------------- 
        // Create publishers 
        // -----------------------------------------------------------------
        
        // Ground truth is intended for simulation infrastructure and review, not as information directly available to vehicle autonomy.
        whale_states_pub_ = create_publisher<manta_interfaces::msg::WhaleStateArray>("/sim/whales/ground_truth", 10);

        // ----------------------------------------------------------------- 
        // Create subscribers 
        // -----------------------------------------------------------------

        // Advance whale positions whenever the orchestrator advances the shared simulation clock.
        step_sub_ = create_subscription<manta_interfaces::msg::SimStep>("/sim/step", 10, [this](const manta_interfaces::msg::SimStep::SharedPtr step) {
            update_whales(*step);
        });

        shutdown_sub_ = create_subscription<std_msgs::msg::Bool>("/sim/shutdown", 10, [this](const std_msgs::msg::Bool::SharedPtr message) {
            if (!message->data) return;
            RCLCPP_INFO(get_logger(), "Shutting down whale node.");
            rclcpp::shutdown();
        });

        // ----------------------------------------------------------------- 
        // Create services
        // -----------------------------------------------------------------

        // Return the first scheduled whale call after a requested simulation time. The orchestrator uses this time to shorten its next batch.
        next_call_service_ = create_service<manta_interfaces::srv::GetNextWhaleCall>("/sim/whales/get_next_call", [this](
            const std::shared_ptr<manta_interfaces::srv::GetNextWhaleCall::Request> request,
            std::shared_ptr<manta_interfaces::srv::GetNextWhaleCall::Response> response) {
            handle_get_next_call(*request, *response);
        });

        RCLCPP_INFO(get_logger(), "Initialized %d whales.", whale_count_);
    }

private:
    struct TrajectoryPoint {
        double time;
        double north;
        double east;
        double velocity_north;
        double velocity_east;
    };

    struct WhaleGroup {
        double crossing_distance;
        double side;
        double start_time;
        std::vector<double> heading_errors;
    };

    struct Whale {
        std::string id;
        int group_index;

        double depth;
        double start_time;
        double next_call_time;
        double call_rate_per_hour;

        std::vector<TrajectoryPoint> trajectory;

        // Current values published in WhaleState messages and written to CSV.
        double north;
        double east;
        double velocity_north;
        double velocity_east;
    };

    std::vector<TrajectoryPoint> generate_trajectory(
        double initial_north,
        double initial_east,
        double crossing_heading,
        double speed,
        double start_time,
        const WhaleGroup &group
    ) {
        std::vector<TrajectoryPoint> trajectory;

        const double heading_std_radians =
            heading_std_degrees_ * std::acos(-1.0) / 180.0;

        const double innovation_scale =
            heading_std_radians
            * std::sqrt(
                1.0
                - heading_correlation_ * heading_correlation_
            );

        // Weight the shared and individual processes so their combined
        // variance remains equal to the configured heading variance.
        const double group_weight =
            std::sqrt(shared_heading_fraction_);

        const double individual_weight =
            std::sqrt(1.0 - shared_heading_fraction_);

        std::normal_distribution<double> standard_normal(0.0, 1.0);

        double time = start_time;
        double north = initial_north;
        double east = initial_east;
        double individual_heading_error = 0.0;

        while (true) {
            const std::size_t group_heading_index = std::min(
                static_cast<std::size_t>(
                    std::floor(time / turn_interval_)
                ),
                group.heading_errors.size() - 1
            );

            const double combined_heading_error =
                group_weight
                    * group.heading_errors[group_heading_index]
                + individual_weight
                    * individual_heading_error;

            const double heading =
                crossing_heading + combined_heading_error;

            const double velocity_north =
                speed * std::cos(heading);

            const double velocity_east =
                speed * std::sin(heading);

            trajectory.push_back({
                time,
                north,
                east,
                velocity_north,
                velocity_east,
            });

            if (time >= mission_duration_) {
                break;
            }

            // End the current segment at the next global movement boundary.
            // All members therefore respond to group heading changes at the
            // same simulated time, even if their activation times differ.
            const double next_turn_time =
                (
                    std::floor(time / turn_interval_) + 1.0
                ) * turn_interval_;

            const double next_time = std::min(
                next_turn_time,
                mission_duration_
            );

            const double duration = next_time - time;

            north += velocity_north * duration;
            east += velocity_east * duration;
            time = next_time;

            if (time >= mission_duration_) {
                trajectory.push_back({
                    time,
                    north,
                    east,
                    velocity_north,
                    velocity_east,
                });
                break;
            }

            individual_heading_error =
                heading_correlation_ * individual_heading_error
                + innovation_scale * standard_normal(generator_);
        }

        return trajectory;
    }

    TrajectoryPoint trajectory_at_time(
        const Whale &whale,
        double time
    ) const {
        if (whale.trajectory.empty()) {
            throw std::runtime_error(
                "Whale trajectory contains no points."
            );
        }

        const TrajectoryPoint &first = whale.trajectory.front();
        const TrajectoryPoint &last = whale.trajectory.back();

        // Before its randomized activation time, the whale remains at its
        // starting location and has not begun moving.
        if (time < first.time) {
            return {
                time,
                first.north,
                first.east,
                0.0,
                0.0,
            };
        }

        // Do not extrapolate beyond the generated mission trajectory.
        if (time >= last.time) {
            return {
                time,
                last.north,
                last.east,
                0.0,
                0.0,
            };
        }

        // Find the first trajectory segment that starts after the requested
        // time. The preceding point defines the active segment.
        const auto next = std::upper_bound(
            whale.trajectory.begin(),
            whale.trajectory.end(),
            time,
            [](
                double requested_time,
                const TrajectoryPoint &point
            ) {
                return requested_time < point.time;
            }
        );

        const TrajectoryPoint &segment = *(next - 1);
        const double elapsed = time - segment.time;

        return {
            time,
            segment.north
                + segment.velocity_north * elapsed,
            segment.east
                + segment.velocity_east * elapsed,
            segment.velocity_north,
            segment.velocity_east,
        };
    }

    void initialize_whale_groups(double line_length) {
        const int group_count = std::max(
            1,
            static_cast<int>(
                std::ceil(
                    static_cast<double>(whale_count_)
                    / group_mean_size_
                )
            )
        );

        const std::size_t heading_count =
            static_cast<std::size_t>(
                std::ceil(mission_duration_ / turn_interval_)
            ) + 1;

        const double heading_std_radians =
            heading_std_degrees_ * std::acos(-1.0) / 180.0;

        const double innovation_scale =
            heading_std_radians
            * std::sqrt(
                1.0
                - heading_correlation_ * heading_correlation_
            );

        std::uniform_real_distribution<double> crossing_distance(
            0.0,
            line_length
        );

        std::uniform_real_distribution<double> start_time(
            0.0,
            mission_duration_
        );

        std::bernoulli_distribution starting_side(0.5);
        std::normal_distribution<double> standard_normal(0.0, 1.0);

        whale_groups_.clear();
        whale_groups_.reserve(group_count);

        for (int index = 0; index < group_count; ++index) {
            WhaleGroup group;

            group.crossing_distance = crossing_distance(generator_);
            group.side = starting_side(generator_) ? 1.0 : -1.0;
            group.start_time = start_time(generator_);
            group.heading_errors.reserve(heading_count);

            // Groups initially travel directly toward their crossing location.
            double heading_error = 0.0;
            group.heading_errors.push_back(heading_error);

            for (std::size_t heading_index = 1; heading_index < heading_count; ++heading_index) {
                heading_error = heading_correlation_ * heading_error + innovation_scale * standard_normal(generator_);
                group.heading_errors.push_back(heading_error);
            }

            whale_groups_.push_back(std::move(group));
        }

        RCLCPP_INFO(get_logger(), "Initialized %zu whale groups for %d whales.", whale_groups_.size(), whale_count_);
    }

    void initialize_whales() {
        const double line_north = line_end_north_ - line_start_north_;
        const double line_east = line_end_east_ - line_start_east_;
        const double line_length = std::hypot(line_north, line_east);

        if (line_length <= 0.0) {
            throw std::runtime_error(
                "Picket line must have nonzero length."
            );
        }
        if (mission_duration_ <= 0.0) {
            throw std::runtime_error(
                "Mission duration must be greater than zero."
            );
        }
        if (call_rate_min_ <= 0.0) {
            throw std::runtime_error(
                "call_rate_min must be greater than zero."
            );
        }
        if (call_rate_max_ < call_rate_min_) {
            throw std::runtime_error(
                "call_rate_max must be greater than or equal to call_rate_min."
            );
        }

        initialize_whale_groups(line_length);

        // Unit vectors parallel and perpendicular to the picket line.
        const double along_north = line_north / line_length;
        const double along_east = line_east / line_length;
        const double perpendicular_north = -along_east;
        const double perpendicular_east = along_north;

        std::uniform_real_distribution<double> speed(
            speed_min_,
            speed_max_
        );

        std::uniform_real_distribution<double> call_rate(
            call_rate_min_,
            call_rate_max_
        );

        std::normal_distribution<double> position_offset(
            0.0,
            group_position_std_
        );

        std::normal_distribution<double> start_time_offset(
            0.0,
            group_start_time_std_
        );

        whales_.clear();
        whales_.reserve(whale_count_);

        for (int index = 0; index < whale_count_; ++index) {
            // Round-robin assignment keeps group sizes balanced. With 46
            // whales and 16 groups, groups contain either two or three whales.
            const int group_index =
                index % static_cast<int>(whale_groups_.size());

            const WhaleGroup &group =
                whale_groups_[group_index];

            // Draw a member crossing location around the group center.
            // Resampling avoids creating artificial piles at the line ends.
            double crossing_distance;

            do {
                crossing_distance =
                    group.crossing_distance
                    + position_offset(generator_);
            } while (
                crossing_distance < 0.0
                || crossing_distance > line_length
            );

            // Draw an activation time around the group's nominal time.
            double whale_start_time;

            do {
                whale_start_time =
                    group.start_time
                    + start_time_offset(generator_);
            } while (
                whale_start_time < 0.0
                || whale_start_time > mission_duration_
            );

            const double crossing_north =
                line_start_north_
                + along_north * crossing_distance;

            const double crossing_east =
                line_start_east_
                + along_east * crossing_distance;

            const double initial_north =
                crossing_north
                + group.side
                    * perpendicular_north
                    * start_distance_;

            const double initial_east =
                crossing_east
                + group.side
                    * perpendicular_east
                    * start_distance_;

            const double whale_speed =
                speed(generator_);

            const double whale_call_rate =
                call_rate(generator_);

            std::exponential_distribution<double> call_interval(
                whale_call_rate / 3600.0
            );

            const double first_call_time =
                whale_start_time + call_interval(generator_);

            const double crossing_velocity_north =
                -group.side
                * perpendicular_north
                * whale_speed;

            const double crossing_velocity_east =
                -group.side
                * perpendicular_east
                * whale_speed;

            const double crossing_heading =
                std::atan2(
                    crossing_velocity_east,
                    crossing_velocity_north
                );

            Whale whale;

            whale.id =
                "whale_" + std::to_string(index + 1);

            whale.group_index = group_index;
            whale.depth = whale_depth_;
            whale.start_time = whale_start_time;
            whale.next_call_time = first_call_time;
            whale.call_rate_per_hour = whale_call_rate;

            whale.trajectory = generate_trajectory(
                initial_north,
                initial_east,
                crossing_heading,
                whale_speed,
                whale_start_time,
                group
            );

            const TrajectoryPoint &initial =
                whale.trajectory.front();

            whale.north = initial.north;
            whale.east = initial.east;
            whale.velocity_north = initial.velocity_north;
            whale.velocity_east = initial.velocity_east;

            whales_.push_back(std::move(whale));
        }
    }

    void handle_get_next_call(const manta_interfaces::srv::GetNextWhaleCall::Request &request, manta_interfaces::srv::GetNextWhaleCall::Response &response) {
        Whale *next_whale = nullptr;
        double next_time = std::numeric_limits<double>::infinity();

        for (auto &whale : whales_) {
            std::exponential_distribution<double> call_interval(whale.call_rate_per_hour / 3600.0);

            // Advance this whale's schedule past every call the orchestrator
            // has already handled.
            while (whale.next_call_time <= request.after_time) {
                whale.next_call_time += call_interval(generator_);
            }

            // Select the earliest remaining call across all whales.
            if (whale.next_call_time <= mission_duration_ && whale.next_call_time < next_time) {
                next_time = whale.next_call_time;
                next_whale = &whale;
            }
        }

        // No calls remain within the configured mission duration.
        if (next_whale == nullptr) {
            response.available = false;
            return;
        }

        response.available = true;
        response.call.whale_id = next_whale->id;
        response.call.cycle = 0;
        response.call.sim_time = next_time;

        // Obtain the position from the same precomputed correlated trajectory
        // used for ground-truth state publication.
        const TrajectoryPoint call_position = trajectory_at_time(*next_whale, next_time);

        response.call.north = call_position.north;
        response.call.east = call_position.east;
        response.call.depth = next_whale->depth;
    }

    void update_whales(const manta_interfaces::msg::SimStep &step) {
        if (step.step_count == 0) {
            RCLCPP_ERROR(get_logger(), "Received a simulation batch with zero steps.");
            return;
        }

        // Determine the simulated time covered by this batch.
        const double batch_start_time = step.sim_time;
        const double batch_end_time = batch_start_time + static_cast<double>(step.step_count) * step.timestep;

        // Trajectory files are only for review, so they can be sampled less often than live whale ground truth is published.
        const bool write_trajectory = batch_end_time >= next_whale_log_time_;

        // Calculates the number of the final cycle processed in the batch.
        const uint64_t completed_cycle = step.current_cycle + step.step_count - 1;

        // Query each whale's precomputed correlated trajectory at the end of
        // this batch. This produces the same position regardless of ROS batch size.
        for (auto &whale : whales_) {
            const TrajectoryPoint position = trajectory_at_time(whale, batch_end_time);

            whale.north = position.north;
            whale.east = position.east;
            whale.velocity_north = position.velocity_north;
            whale.velocity_east = position.velocity_east;
        }

        // Publish whale ground truth at the end of the completed batch.
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

            if (write_trajectory) {
                whale_log_
                    << completed_cycle << ','
                    << batch_end_time << ','
                    << whale.id << ','
                    << whale.group_index + 1 << ','
                    << whale.north << ','
                    << whale.east << ','
                    << whale.depth << '\n';
            }
        }

        if (write_trajectory) next_whale_log_time_ = batch_end_time + whale_sample_period_;

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
    double call_rate_min_;
    double call_rate_max_;
    double whale_sample_period_ = 60.0;
    double next_whale_log_time_ = 0.0;
    double turn_interval_;
    double heading_correlation_;
    double heading_std_degrees_;
    double group_mean_size_;
    double group_position_std_;
    double group_start_time_std_;
    double shared_heading_fraction_;

    std::mt19937 generator_;
    std::vector<WhaleGroup> whale_groups_;
    std::vector<Whale> whales_;
    std::ofstream whale_log_;

    rclcpp::Publisher<manta_interfaces::msg::WhaleStateArray>::SharedPtr whale_states_pub_;

    rclcpp::Subscription<manta_interfaces::msg::SimStep>::SharedPtr step_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr shutdown_sub_;

    rclcpp::Service<manta_interfaces::srv::GetNextWhaleCall>::SharedPtr next_call_service_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WhaleNode>());
    rclcpp::shutdown();
    return 0;
}