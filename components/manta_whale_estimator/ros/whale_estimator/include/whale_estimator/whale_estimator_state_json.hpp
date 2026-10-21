#ifndef MANTA_WHALE_ESTIMATOR_STATE_JSON_HPP
#define MANTA_WHALE_ESTIMATOR_STATE_JSON_HPP

#include "manta_interfaces/state_log.hpp"

inline void write_json(std::ostream &out, const whale_estimate_one_axis_t &axis) {
    bool first = true;
    out << '{';
    manta::json_field(out, "position", axis.position, first);
    manta::json_field(out, "velocity", axis.velocity, first);
    manta::json_field(out, "position_variance", axis.position_variance, first);
    manta::json_field(out, "position_velocity_covariance", axis.position_velocity_covariance, first);
    manta::json_field(out, "velocity_variance", axis.velocity_variance, first);
    out << '}';
}

inline void write_json(std::ostream &out, const whale_bearing_position_t &fix) {
    bool first = true;
    out << '{';
    manta::json_field(out, "time", fix.time, first);
    manta::json_field(out, "north", fix.north, first);
    manta::json_field(out, "east", fix.east, first);
    manta::json_field(out, "north_variance", fix.north_variance, first);
    manta::json_field(out, "east_variance", fix.east_variance, first);
    out << '}';
}

inline void write_json(std::ostream &out, const whale_estimator_config_t &config) {
    bool first = true;
    out << '{';
    manta::json_field(out, "velocity_window_max_age_s", config.velocity_window_max_age_s, first);
    manta::json_field(out, "acceleration_psd_m2_s3", config.acceleration_psd_m2_s3, first);
    manta::json_field(out, "velocity_return_time_s", config.velocity_return_time_s, first);
    manta::json_field(out, "nominal_north_velocity_mps", config.nominal_north_velocity_mps, first);
    manta::json_field(out, "nominal_east_velocity_mps", config.nominal_east_velocity_mps, first);
    manta::json_field(out, "initial_velocity_variance_m2ps2", config.initial_velocity_variance_m2ps2, first);
    out << '}';
}

inline void write_json(std::ostream &out, const whale_estimator_state_t &state) {
    bool first = true;
    out << '{';
    manta::json_field(out, "config", state.config, first);
    manta::json_field(out, "estimator_configured_b", state.estimator_configured_b != 0, first);
    manta::json_field(out, "shutdown_requested_b", state.shutdown_requested_b != 0, first);
    manta::json_field(out, "received_shutdown_b", state.received_shutdown_b != 0, first);
    manta::json_field(out, "estimator_initialized_b", state.estimator_initialized_b != 0, first);
    manta::json_field(out, "measurement_time_s", state.measurement_time_s, first);
    manta::json_field(out, "north", state.north, first);
    manta::json_field(out, "east", state.east, first);
    out << ",\"recent_fixes\":[";
    for (int i = 0; i < state.recent_fix_count; ++i) {
        if (i) out << ',';
        write_json(out, state.recent_fixes[i]);
    }
    out << ']';
    manta::json_field(out, "recent_fix_count", state.recent_fix_count, first);
    manta::json_field(out, "received_bearing_whale_position", state.received_bearing_whale_position, first);
    manta::json_field(out, "received_vehicle_id", state.received_vehicle_id, first);
    manta::json_field(out, "received_call_cycle", state.received_call_cycle, first);
    manta::json_field(out, "bearing_whale_position_accepted_b", state.bearing_whale_position_accepted_b != 0, first);
    manta::json_field(out, "axis_update", state.axis_update, first);
    manta::json_field(out, "axis_update_elapsed_s", state.axis_update_elapsed_s, first);
    manta::json_field(out, "axis_update_is_north_b", state.axis_update_is_north_b != 0, first);
    out << '}';
}

#endif
