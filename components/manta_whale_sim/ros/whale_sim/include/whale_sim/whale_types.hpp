#ifndef WHALE_SIM__WHALE_TYPES_HPP_
#define WHALE_SIM__WHALE_TYPES_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

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
    double picket_line_start_north_m{};
    double picket_line_start_east_m{};
    double picket_line_end_north_m{};
    double picket_line_end_east_m{};
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
    double whale_start_distance_from_picket_line_m{};
};

// Parameters shared by all generated whale groups.
struct WhaleGroupConfig {
    double whale_group_mean_size{};
    double whale_group_position_std_m{};
    double whale_group_start_time_std_s{};
    double whale_group_heading_fraction{};
};

// Parameters shared by all whale trajectories.
struct WhaleMotionConfig {
    double whale_heading_correlation{};
    double whale_heading_std_deg{};
    double whale_speed_mean_mps{};
    double whale_speed_std_mps{};
    double whale_turn_interval_s{};
};

// Parameters for the population-wide whale calling schedule.
struct WhaleCallConfig {
    double whale_call_bout_mean_duration_s{};
    double whale_call_bout_mean_silent_duration_s{};
    double whale_active_b_call_bout_rate_callph{};
};

// Fields from a simulation step needed to advance and sample whale state.
struct WhaleStepInput {
    std::uint64_t current_cycle{};
    double sim_time_s{};
    double timestep{};
    std::uint32_t step_count{};
};

// Temporary values used while constructing one whale and its trajectory.
struct WhaleGenerationContext {
    std::size_t whale_index{};
    std::size_t group_index{};
    double start_time_s{};
    double speed_mps{};
    double group_crossing_distance_m{};
    WhaleInitialState initial_state{};
};

// Selects one whale trajectory and the time to evaluate on it.
struct WhaleTrajectoryQuery {
    std::size_t whale_index{};
    double sim_time_s{};
};

// Time span over which population-wide whale calls are generated.
struct WhaleCallScheduleWindow {
    double start_time_s{};
    double end_time_s{};
};

#endif  // WHALE_SIM__WHALE_TYPES_HPP_
