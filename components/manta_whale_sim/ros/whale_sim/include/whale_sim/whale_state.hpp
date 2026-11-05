#ifndef WHALE_SIM__WHALE_STATE_HPP_
#define WHALE_SIM__WHALE_STATE_HPP_

#include <cstddef>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include "whale_interfaces/msg/whale_call.hpp"
#include "whale_interfaces/msg/whale_step_result.hpp"
#include "whale_interfaces/msg/whale_state_array.hpp"
#include "manta_interfaces/msg/sim_step.hpp"
#include "std_msgs/msg/bool.hpp"
#include "whale_sim/whale_types.hpp"

// Configuration, population, and runtime state for the whale simulator.
struct WhaleState {
    // Output
    double next_whale_log_time_s = 0.0;
    double whale_sample_period_s = 60.0;
    std::string data_directory = "/odyssey/output/Data";

    // Scenario
    LineGeometry line_geometry{};
    double mission_duration_s = 0;

    // Simulation
    double sim_time_s = 0.0;
    std::uint64_t completed_cycle = 0;
    manta_interfaces::msg::SimStep received_sim_step;
    whale_interfaces::msg::WhaleStepResult step_result;
    std_msgs::msg::Bool ready;
    WhaleStepInput pending_sim_step{};
    WhaleTrajectoryQuery trajectory_query{};
    WhaleGenerationContext whale_generation{};
    WhaleCallScheduleWindow call_schedule_window{};

    // Whale initialization
    int whale_count = 0;
    bool whales_active_entire_mission_b = false;
    whale_interfaces::msg::WhaleCall next_whale_call;
    bool next_whale_call_available_b = false;
    std::size_t next_call_whale_index = 0;
    whale_interfaces::msg::WhaleStateArray current_whale_states;
    std::vector<WhaleGroup> whale_groups;
    std::vector<Whale> whales;
    std::string whale_initialization_mode;
    WhaleInitialState whale_initial_state{};
    WhaleGroupConfig whale_group_config{};
    WhaleMotionConfig whale_motion_config{};
    WhaleCallConfig whale_call_config{};

    // Seed, output destination, and lifecycle.
    std::int64_t random_seed = 42;
    std::mt19937 rng{static_cast<std::mt19937::result_type>(random_seed)};
    bool shutdown_requested_b = false;
};

#endif  // WHALE_SIM__WHALE_STATE_HPP_
