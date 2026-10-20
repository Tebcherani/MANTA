#ifndef WHALE_SIM__WHALE_STATE_HPP_
#define WHALE_SIM__WHALE_STATE_HPP_

#include <cstddef>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include "whale_interfaces/msg/whale_call.hpp"

// Configuration, population, and runtime state for the whale simulator.
struct WhaleState {
    // One timestamped point in a precomputed whale trajectory.
    struct WhaleWaypoint {
        double sim_time_s{};
        double whale_north_m{};
        double whale_east_m{};
        double whale_velocity_north_mps{};
        double whale_velocity_east_mps{};
    };

    // Shared movement and calling behavior for one group of whales.
    struct WhaleGroup {
        double whale_group_crossing_distance_m{};
        double whale_group_starting_side{};
        double whale_group_start_time_s{};
        std::vector<double> whale_group_heading_errors_rad;
    };

    // Simulation state and precomputed behavior for one whale.
    struct Whale {
        std::string id;
        int group_index{};
        double start_time_s{};
        std::vector<double> whale_call_times_s;
        std::size_t next_call_index = 0;
        std::vector<WhaleWaypoint> trajectory;
        double whale_north_m{};
        double whale_east_m{};
        double whale_velocity_north_mps{};
        double whale_velocity_east_mps{};
        bool whale_active_b{};
        bool whale_complete_b{};
        double whale_starting_side{};
    };

    // Geometric properties of the picket line expressed in local north-east coordinates.
    struct LineGeometry {
        double picket_line_length_m{};
        double picket_line_unit_vector_north{};
        double picket_line_unit_vector_east{};
        double perpendicular_to_picket_line_unit_vector_north{};
        double perpendicular_to_picket_line_unit_vector_east{};
    };

    // Initial whale position and direction of travel toward the picket line.
    struct WhaleInitialState {
        double whale_north_m{};
        double whale_east_m{};
        double whale_heading_rad{};
    };

    // Output
    double next_whale_log_time_s = 0.0;
    double whale_sample_period_s = 60.0;

    // Scenario
    double picket_line_start_north_m = 0;
    double picket_line_start_east_m = 0;
    double picket_line_end_north_m = 0;
    double picket_line_end_east_m = 0;

    double mission_duration_s = 0;

    // Simulation
    double sim_time_s = 0.0;
    std::uint64_t completed_cycle = 0;
    whale_interfaces::msg::WhaleCall next_whale_call;
    bool next_whale_call_available_b = false;
    std::mt19937 rng{42};
    std::vector<WhaleGroup> whale_groups;
    std::vector<Whale> whales;

    // Whale initialization
    int whale_count = 0;
    bool whales_active_entire_mission_b = false;
    std::string whale_initialization_mode;

    double whale_start_distance_from_picket_line_m = 0;
    double whale_initial_north_m = 0;
    double whale_initial_east_m = 0;
    double whale_initial_heading_deg = 0;

    double whale_call_bout_mean_duration_s = 0;
    double whale_call_bout_mean_silent_duration_s = 0;
    double whale_active_b_call_bout_rate_callph = 0;

    double whale_group_mean_size = 0;
    double whale_group_position_std_m = 0;
    double whale_group_start_time_std_s = 0;
    double whale_group_heading_fraction = 0;

    double whale_heading_correlation = 0;
    double whale_heading_std_deg = 0;
    double whale_speed_mean_mps = 0;
    double whale_speed_std_mps = 0;
    double whale_turn_interval_s = 0;

    // Seed, output destination, and lifecycle.
    std::int64_t random_seed = 42;
    std::string data_directory = "/odyssey/output/Data";
    bool shutdown_requested_b = false;
};

#endif  // WHALE_SIM__WHALE_STATE_HPP_
