#include "whale_sim/whale_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>

WhaleNode::WhaleNode() : rclcpp::Node("whale_node"), generator_(42) {
    load_config();
    validate_config();
    open_whale_log();
    seed_generator();
    initialize_whales();
    create_ros_interfaces();

    RCLCPP_INFO(get_logger(), "Initialized %d whales.", whale_count_);
}

WhaleNode::TrajectoryPoint WhaleNode::trajectory_at_time(const Whale &whale, double time) const {
        if (whale.trajectory.empty()) throw std::runtime_error("Whale trajectory contains no points.");

        const TrajectoryPoint &first = whale.trajectory.front();
        const TrajectoryPoint &last = whale.trajectory.back();

        // Before its randomized activation time, the whale remains at its
        // starting location and has not begun moving.
        if (time < first.time) return {time, first.north, first.east, 0.0, 0.0};

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

void WhaleNode::update_whales(
        const manta_interfaces::msg::SimStep &step
    ) {
        if (step.step_count == 0) {
            RCLCPP_ERROR(
                get_logger(),
                "Received a simulation batch with zero steps."
            );
            return;
        }

        const double batch_end_time =
            step.sim_time
            + static_cast<double>(step.step_count)
                * step.timestep;

        const uint64_t completed_cycle =
            step.current_cycle + step.step_count - 1;

        // Update current whale states from the precomputed trajectories.
        // Looking up absolute simulated time makes motion independent of
        // the number of native cycles placed in each ROS batch.
        for (auto &whale : whales_) {
            const TrajectoryPoint position =
                trajectory_at_time(whale, batch_end_time);

            whale.north = position.north;
            whale.east = position.east;
            whale.velocity_north = position.velocity_north;
            whale.velocity_east = position.velocity_east;
        }

        // Publish ground truth at the completed batch time.
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

        // Write every scheduled trajectory sample crossed by this batch.
        // This preserves the requested output interval even when one ROS
        // message advances through many seconds of simulated time.
        while (
            next_whale_log_time_
            <= batch_end_time + 1.0e-9
        ) {
            // Ignore any stale logging boundary preceding this batch.
            if (next_whale_log_time_ < step.sim_time) {
                next_whale_log_time_ += whale_sample_period_;
                continue;
            }

            const double cycle_offset =
                (
                    next_whale_log_time_ - step.sim_time
                ) / step.timestep;

            const uint64_t sample_offset = std::min<uint64_t>(
                step.step_count - 1,
                static_cast<uint64_t>(
                    std::max(0.0, std::floor(cycle_offset))
                )
            );

            const uint64_t sample_cycle =
                step.current_cycle + sample_offset;

            for (const auto &whale : whales_) {
                const TrajectoryPoint sample =
                    trajectory_at_time(
                        whale,
                        next_whale_log_time_
                    );

                whale_log_
                    << sample_cycle << ','
                    << next_whale_log_time_ << ','
                    << whale.id << ','
                    << whale.group_index + 1 << ','
                    << sample.north << ','
                    << sample.east << ','
                    << whale.depth << '\n';
            }

            next_whale_log_time_ += whale_sample_period_;
        }
    }

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WhaleNode>());
    rclcpp::shutdown();
    return 0;
}