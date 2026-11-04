#ifndef WHALE_SIM__STATE_JSON_HPP_
#define WHALE_SIM__STATE_JSON_HPP_

#include "manta_interfaces/state_log.hpp"
#include "whale_sim/whale_state.hpp"

inline void write_json(std::ostream &out, const WhaleStepInput &state) {
    using manta::write_json;
    bool first = true;
    out << '{';
    manta::json_field(out, "current_cycle", state.current_cycle, first);
    manta::json_field(out, "sim_time_s", state.sim_time_s, first);
    manta::json_field(out, "timestep", state.timestep, first);
    manta::json_field(out, "step_count", state.step_count, first);
    out << '}';
}

namespace whale_interfaces { namespace msg {
#ifndef MANTA_WHALE_CALL_JSON_DEFINED
#define MANTA_WHALE_CALL_JSON_DEFINED
inline void write_json(std::ostream &out, const WhaleCall &state) {
    using manta::write_json;
    bool first = true;
    out << '{';
    manta::json_field(out, "whale_id", state.whale_id, first);
    manta::json_field(out, "cycle", state.cycle, first);
    manta::json_field(out, "sim_time_s", state.sim_time_s, first);
    manta::json_field(out, "whale_call_north_m", state.whale_call_north_m, first);
    manta::json_field(out, "whale_call_east_m", state.whale_call_east_m, first);
    out << '}';
}
#endif
inline void write_json(std::ostream &out, const WhaleStepResult &state) {
    bool first = true;
    out << '{';
    manta::json_field(out, "completed_cycle", state.completed_cycle, first);
    manta::json_field(out, "completed_step_count", state.completed_step_count, first);
    manta::json_field(out, "succeeded_b", state.succeeded_b, first);
    manta::json_field(out, "whale_call_published_b", state.whale_call_published_b, first);
    manta::json_field(out, "whale_call", state.whale_call, first);
    manta::json_field(out, "detail", state.detail, first);
    out << '}';
}
inline void write_json(std::ostream &out, const WhaleState &state) {
    using manta::write_json;
    bool first = true;
    out << '{';
    manta::json_field(out, "whale_active_b_b", state.whale_active_b_b, first);
    manta::json_field(out, "whale_id", state.whale_id, first);
    manta::json_field(out, "cycle", state.cycle, first);
    manta::json_field(out, "whale_north_m", state.whale_north_m, first);
    manta::json_field(out, "whale_east_m", state.whale_east_m, first);
    manta::json_field(out, "whale_velocity_north_mps", state.whale_velocity_north_mps, first);
    manta::json_field(out, "whale_velocity_east_mps", state.whale_velocity_east_mps, first);
    out << '}';
}
inline void write_json(std::ostream &out, const WhaleStateArray &state) {
    using manta::write_json;
    bool first = true;
    out << '{';
    manta::json_field(out, "cycle", state.cycle, first);
    manta::json_field(out, "whales", state.whales, first);
    out << '}';
}
}}

namespace manta_interfaces { namespace msg {
inline void write_json(std::ostream &out, const SimStep &state) {
    bool first = true;
    out << '{';
    manta::json_field(out, "current_cycle", state.current_cycle, first);
    manta::json_field(out, "sim_time_s", state.sim_time_s, first);
    manta::json_field(out, "timestep_s", state.timestep_s, first);
    manta::json_field(out, "step_count", state.step_count, first);
    manta::json_field(out, "control_period", state.control_period, first);
    manta::json_field(out, "whale_call_expected_b", state.whale_call_expected_b, first);
    out << '}';
}
}}

inline void write_json(std::ostream &out, const WhaleWaypoint &state) {
    using manta::write_json;
    bool first = true;
    out << '{';
    manta::json_field(out, "sim_time_s", state.sim_time_s, first);
    manta::json_field(out, "whale_north_m", state.whale_north_m, first);
    manta::json_field(out, "whale_east_m", state.whale_east_m, first);
    manta::json_field(out, "whale_velocity_north_mps", state.whale_velocity_north_mps, first);
    manta::json_field(out, "whale_velocity_east_mps", state.whale_velocity_east_mps, first);
    out << '}';
}

inline void write_whale_group_json(std::ostream &out, const WhaleGroup &state, bool include_precomputed) {
    using manta::write_json;
    bool first = true;
    out << '{';
    manta::json_field(out, "whale_group_crossing_distance_m", state.whale_group_crossing_distance_m, first);
    manta::json_field(out, "whale_group_starting_side", state.whale_group_starting_side, first);
    manta::json_field(out, "whale_group_start_time_s", state.whale_group_start_time_s, first);
    if (include_precomputed) {
        manta::json_field(out, "whale_group_heading_errors_rad", state.whale_group_heading_errors_rad, first);
    }
    out << '}';
}

inline void write_json(std::ostream &out, const WhaleGroup &state) {
    write_whale_group_json(out, state, true);
}

inline void write_whale_json(std::ostream &out, const Whale &state, bool include_precomputed) {
    using manta::write_json;
    bool first = true;
    out << '{';
    manta::json_field(out, "id", state.id, first);
    manta::json_field(out, "group_index", state.group_index, first);
    manta::json_field(out, "start_time_s", state.start_time_s, first);
    if (include_precomputed) {
        manta::json_field(out, "whale_call_times_s", state.whale_call_times_s, first);
    }
    manta::json_field(out, "next_call_index", state.next_call_index, first);
    if (include_precomputed) {
        manta::json_field(out, "trajectory", state.trajectory, first);
    }
    manta::json_field(out, "whale_north_m", state.whale_north_m, first);
    manta::json_field(out, "whale_east_m", state.whale_east_m, first);
    manta::json_field(out, "whale_velocity_north_mps", state.whale_velocity_north_mps, first);
    manta::json_field(out, "whale_velocity_east_mps", state.whale_velocity_east_mps, first);
    manta::json_field(out, "whale_active_b", state.whale_active_b, first);
    manta::json_field(out, "whale_complete_b", state.whale_complete_b, first);
    manta::json_field(out, "whale_starting_side", state.whale_starting_side, first);
    out << '}';
}

inline void write_json(std::ostream &out, const Whale &state) {
    write_whale_json(out, state, true);
}

inline void write_json(std::ostream &out, const LineGeometry &state) {
    using manta::write_json;
    bool first = true;
    out << '{';
    manta::json_field(out, "picket_line_start_north_m", state.picket_line_start_north_m, first);
    manta::json_field(out, "picket_line_start_east_m", state.picket_line_start_east_m, first);
    manta::json_field(out, "picket_line_end_north_m", state.picket_line_end_north_m, first);
    manta::json_field(out, "picket_line_end_east_m", state.picket_line_end_east_m, first);
    manta::json_field(out, "picket_line_length_m", state.picket_line_length_m, first);
    manta::json_field(out, "picket_line_unit_vector_north", state.picket_line_unit_vector_north, first);
    manta::json_field(out, "picket_line_unit_vector_east", state.picket_line_unit_vector_east, first);
    manta::json_field(out, "perpendicular_to_picket_line_unit_vector_north", state.perpendicular_to_picket_line_unit_vector_north, first);
    manta::json_field(out, "perpendicular_to_picket_line_unit_vector_east", state.perpendicular_to_picket_line_unit_vector_east, first);
    out << '}';
}

inline void write_json(std::ostream &out, const WhaleInitialState &state) {
    using manta::write_json;
    bool first = true;
    out << '{';
    manta::json_field(out, "whale_north_m", state.whale_north_m, first);
    manta::json_field(out, "whale_east_m", state.whale_east_m, first);
    manta::json_field(out, "whale_heading_rad", state.whale_heading_rad, first);
    manta::json_field(out, "whale_start_distance_from_picket_line_m", state.whale_start_distance_from_picket_line_m, first);
    out << '}';
}

inline void write_json(std::ostream &out, const WhaleGroupConfig &state) {
    using manta::write_json;
    bool first = true;
    out << '{';
    manta::json_field(out, "whale_group_mean_size", state.whale_group_mean_size, first);
    manta::json_field(out, "whale_group_position_std_m", state.whale_group_position_std_m, first);
    manta::json_field(out, "whale_group_start_time_std_s", state.whale_group_start_time_std_s, first);
    manta::json_field(out, "whale_group_heading_fraction", state.whale_group_heading_fraction, first);
    out << '}';
}

inline void write_json(std::ostream &out, const WhaleMotionConfig &state) {
    using manta::write_json;
    bool first = true;
    out << '{';
    manta::json_field(out, "whale_heading_correlation", state.whale_heading_correlation, first);
    manta::json_field(out, "whale_heading_std_deg", state.whale_heading_std_deg, first);
    manta::json_field(out, "whale_speed_mean_mps", state.whale_speed_mean_mps, first);
    manta::json_field(out, "whale_speed_std_mps", state.whale_speed_std_mps, first);
    manta::json_field(out, "whale_turn_interval_s", state.whale_turn_interval_s, first);
    out << '}';
}

inline void write_json(std::ostream &out, const WhaleCallConfig &state) {
    using manta::write_json;
    bool first = true;
    out << '{';
    manta::json_field(out, "whale_call_bout_mean_duration_s", state.whale_call_bout_mean_duration_s, first);
    manta::json_field(out, "whale_call_bout_mean_silent_duration_s", state.whale_call_bout_mean_silent_duration_s, first);
    manta::json_field(out, "whale_active_b_call_bout_rate_callph", state.whale_active_b_call_bout_rate_callph, first);
    out << '}';
}

inline void write_json(std::ostream &out, const WhaleGenerationContext &state) {
    using manta::write_json;
    bool first = true;
    out << '{';
    manta::json_field(out, "whale_index", state.whale_index, first);
    manta::json_field(out, "group_index", state.group_index, first);
    manta::json_field(out, "start_time_s", state.start_time_s, first);
    manta::json_field(out, "speed_mps", state.speed_mps, first);
    manta::json_field(out, "group_crossing_distance_m", state.group_crossing_distance_m, first);
    manta::json_field(out, "initial_state", state.initial_state, first);
    out << '}';
}

inline void write_json(std::ostream &out, const WhaleTrajectoryQuery &state) {
    using manta::write_json;
    bool first = true;
    out << '{';
    manta::json_field(out, "whale_index", state.whale_index, first);
    manta::json_field(out, "sim_time_s", state.sim_time_s, first);
    out << '}';
}

inline void write_json(std::ostream &out, const WhaleCallScheduleWindow &state) {
    using manta::write_json;
    bool first = true;
    out << '{';
    manta::json_field(out, "start_time_s", state.start_time_s, first);
    manta::json_field(out, "end_time_s", state.end_time_s, first);
    out << '}';
}

inline void write_whale_state_json(std::ostream &out, const WhaleState &state, bool include_precomputed) {
    using manta::write_json;
    bool first = true;
    out << '{';
    manta::json_field(out, "next_whale_log_time_s", state.next_whale_log_time_s, first);
    manta::json_field(out, "whale_sample_period_s", state.whale_sample_period_s, first);
    manta::json_field(out, "line_geometry", state.line_geometry, first);
    manta::json_field(out, "mission_duration_s", state.mission_duration_s, first);
    manta::json_field(out, "sim_time_s", state.sim_time_s, first);
    manta::json_field(out, "completed_cycle", state.completed_cycle, first);
    manta::json_field(out, "received_sim_step", state.received_sim_step, first);
    manta::json_field(out, "step_result", state.step_result, first);
    manta::json_field(out, "ready", state.ready.data, first);
    manta::json_field(out, "pending_sim_step", state.pending_sim_step, first);
    manta::json_field(out, "trajectory_query", state.trajectory_query, first);
    manta::json_field(out, "whale_generation", state.whale_generation, first);
    manta::json_field(out, "call_schedule_window", state.call_schedule_window, first);
    manta::json_field(out, "next_whale_call", state.next_whale_call, first);
    manta::json_field(out, "next_whale_call_available_b", state.next_whale_call_available_b, first);
    manta::json_field(out, "next_call_whale_index", state.next_call_whale_index, first);
    manta::json_field(out, "current_whale_states", state.current_whale_states, first);
    if (!first) out << ',';
    first = false;
    out << "\"whale_groups\":[";
    for (std::size_t i = 0; i < state.whale_groups.size(); ++i) {
        if (i > 0) out << ',';
        write_whale_group_json(out, state.whale_groups[i], include_precomputed);
    }
    out << ']';
    if (!first) out << ',';
    first = false;
    out << "\"whales\":[";
    for (std::size_t i = 0; i < state.whales.size(); ++i) {
        if (i > 0) out << ',';
        write_whale_json(out, state.whales[i], include_precomputed);
    }
    out << ']';
    manta::json_field(out, "whale_count", state.whale_count, first);
    manta::json_field(out, "whales_active_entire_mission_b", state.whales_active_entire_mission_b, first);
    manta::json_field(out, "whale_initialization_mode", state.whale_initialization_mode, first);
    manta::json_field(out, "whale_initial_state", state.whale_initial_state, first);
    manta::json_field(out, "whale_group_config", state.whale_group_config, first);
    manta::json_field(out, "whale_motion_config", state.whale_motion_config, first);
    manta::json_field(out, "whale_call_config", state.whale_call_config, first);
    manta::json_field(out, "random_seed", state.random_seed, first);
    manta::json_field(out, "data_directory", state.data_directory, first);
    manta::json_field(out, "shutdown_requested_b", state.shutdown_requested_b, first);
    out << '}';
}

inline void write_json(std::ostream &out, const WhaleState &state) {
    write_whale_state_json(out, state, true);
}

inline void write_whale_runtime_json(std::ostream &out, const WhaleState &state) {
    write_whale_state_json(out, state, false);
}

#endif
