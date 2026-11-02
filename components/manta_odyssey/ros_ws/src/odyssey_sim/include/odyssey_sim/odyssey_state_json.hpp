#ifndef ODYSSEY_SIM__ODYSSEY_STATE_JSON_HPP_
#define ODYSSEY_SIM__ODYSSEY_STATE_JSON_HPP_

#include <ostream>
#include <string>

#include "manta_interfaces/state_log.hpp"
#include "odyssey_sim/odyssey_state.hpp"

inline void write_json(std::ostream &out, const OdysseyState::BearingPosition &position) {
    bool first = true;
    out << '{';
    manta::json_field(out, "cycle", position.cycle, first);
    manta::json_field(out, "sim_time_s", position.sim_time_s, first);
    manta::json_field(out, "north_m", position.north_m, first);
    manta::json_field(out, "east_m", position.east_m, first);
    out << '}';
}

namespace acoustics_interfaces { namespace msg {
inline void write_json(std::ostream &out, const BearingObservation &message) {
    bool first = true;
    out << '{';
    manta::json_field(out, "receiver_id", message.receiver_id, first);
    manta::json_field(out, "call_cycle", message.call_cycle, first);
    manta::json_field(out, "sim_time_s", message.sim_time_s, first);
    manta::json_field(out, "bearing_rad", message.bearing_rad, first);
    manta::json_field(out, "bearing_std_rad", message.bearing_std_rad, first);
    out << '}';
}

inline void write_json(std::ostream &out, const BearingTransmission &message) {
    bool first = true;
    out << '{';
    manta::json_field(out, "receiver_id", message.receiver_id, first);
    manta::json_field(out, "call_cycle", message.call_cycle, first);
    manta::json_field(out, "sim_time_s", message.sim_time_s, first);
    manta::json_field(out, "bearing_rad", message.bearing_rad, first);
    manta::json_field(out, "bearing_std_rad", message.bearing_std_rad, first);
    manta::json_field(out, "receiver_north_m", message.receiver_north_m, first);
    manta::json_field(out, "receiver_east_m", message.receiver_east_m, first);
    out << '}';
}
}}

namespace whale_estimator_interfaces { namespace msg {
inline void write_json(std::ostream &out, const WhaleKalmanEstimate &message) {
    bool first = true;
    out << '{';
    manta::json_field(out, "vehicle_id", message.vehicle_id, first);
    manta::json_field(out, "call_cycle", message.call_cycle, first);
    manta::json_field(out, "measurement_time_s", message.measurement_time_s, first);
    manta::json_field(out, "whale_kalman_north_m", message.whale_kalman_north_m, first);
    manta::json_field(out, "whale_kalman_east_m", message.whale_kalman_east_m, first);
    manta::json_field(out, "whale_kalman_position_variance_north_m2", message.whale_kalman_position_variance_north_m2, first);
    manta::json_field(out, "whale_kalman_position_variance_east_m2", message.whale_kalman_position_variance_east_m2, first);
    manta::json_field(out, "whale_kalman_velocity_north_mps", message.whale_kalman_velocity_north_mps, first);
    manta::json_field(out, "whale_kalman_velocity_east_mps", message.whale_kalman_velocity_east_mps, first);
    manta::json_field(out, "whale_kalman_velocity_variance_north_m2ps2", message.whale_kalman_velocity_variance_north_m2ps2, first);
    manta::json_field(out, "whale_kalman_velocity_variance_east_m2ps2", message.whale_kalman_velocity_variance_east_m2ps2, first);
    manta::json_field(out, "whale_baseline_velocity_north_mps", message.whale_baseline_velocity_north_mps, first);
    manta::json_field(out, "whale_baseline_velocity_east_mps", message.whale_baseline_velocity_east_mps, first);
    manta::json_field(out, "whale_velocity_return_time_s", message.whale_velocity_return_time_s, first);
    out << '}';
}
}}

namespace manta_interfaces { namespace msg {
inline void write_json(std::ostream &out, const WorldState &world) {
    bool first = true;
    out << '{';
    manta::json_field(out, "gravity_mps2", world.gravity_mps2, first);
    manta::json_field(out, "magnetic_north_gauss", world.magnetic_north_gauss, first);
    manta::json_field(out, "magnetic_east_gauss", world.magnetic_east_gauss, first);
    manta::json_field(out, "magnetic_down_gauss", world.magnetic_down_gauss, first);
    manta::json_field(out, "seawater_density_kgpm3", world.seawater_density_kgpm3, first);
    out << '}';
}

inline void write_json(std::ostream &out, const SimStep &step) {
    bool first = true;
    out << '{';
    manta::json_field(out, "current_cycle", step.current_cycle, first);
    manta::json_field(out, "sim_time_s", step.sim_time_s, first);
    manta::json_field(out, "timestep_s", step.timestep_s, first);
    manta::json_field(out, "step_count", step.step_count, first);
    manta::json_field(out, "control_period", step.control_period, first);
    manta::json_field(out, "world", step.world, first);
    out << '}';
}
}}

template<class T>
inline void write_nullable_field(std::ostream &out, const char *name, const T *value, bool &first) {
    if (!first) out << ',';
    first = false;
    manta::write_json(out, std::string(name));
    out << ':';
    if (value != nullptr) {
        using manta::write_json;
        write_json(out, *value);
    } else out << "null";
}

inline void write_json(std::ostream &out, const OdysseyState &state) {
    bool first = true;
    out << '{';
    manta::json_field(out, "bearing_positions_count", state.bearing_positions.size(), first);
    write_nullable_field(out, "latest_bearing_position",
        state.bearing_positions.empty() ? nullptr : &state.bearing_positions.back(), first);
    manta::json_field(out, "latest_bearing_available_b", state.latest_bearing_available_b, first);
    manta::json_field(out, "latest_bearing_unsent_b", state.latest_bearing_unsent_b, first);
    manta::json_field(out, "latest_bearing", state.latest_bearing, first);
    manta::json_field(out, "last_received_local_bearing_available_b", state.last_received_local_bearing_available_b, first);
    manta::json_field(out, "last_received_local_bearing", state.last_received_local_bearing, first);
    manta::json_field(out, "last_received_shared_bearing_available_b", state.last_received_shared_bearing_available_b, first);
    manta::json_field(out, "last_received_shared_bearing", state.last_received_shared_bearing, first);
    manta::json_field(out, "last_received_whale_estimate_available_b", state.last_received_whale_estimate_available_b, first);
    manta::json_field(out, "last_received_whale_estimate", state.last_received_whale_estimate, first);
    manta::json_field(out, "local_bearings_by_cycle_count", state.local_bearings_by_cycle.size(), first);
    write_nullable_field(out, "highest_cycle_local_bearing",
        state.local_bearings_by_cycle.empty() ? nullptr : &state.local_bearings_by_cycle.rbegin()->second, first);
    manta::json_field(out, "pending_shared_bearings_count", state.pending_shared_bearings.size(), first);
    write_nullable_field(out, "highest_cycle_pending_shared_bearing",
        state.pending_shared_bearings.empty() ? nullptr : state.pending_shared_bearings.rbegin()->second.get(), first);
    manta::json_field(out, "processed_bearing_cycles_count", state.processed_bearing_cycles.size(), first);
    write_nullable_field(out, "highest_processed_bearing_cycle",
        state.processed_bearing_cycles.empty() ? nullptr : &*state.processed_bearing_cycles.rbegin(), first);
    manta::json_field(out, "communication_interval_s", state.communication_interval_s, first);
    manta::json_field(out, "next_communication_index", state.next_communication_index, first);
    manta::json_field(out, "log_decimation_configured_b", state.log_decimation_configured_b, first);
    manta::json_field(out, "metrics_written_b", state.metrics_written_b, first);
    manta::json_field(out, "mission_finished_b", state.mission_finished_b, first);
    manta::json_field(out, "tracking_enabled_b", state.tracking_enabled_b, first);
    manta::json_field(out, "calendar_start_time_received_b", state.calendar_start_time_received_b, first);
    manta::json_field(out, "odyssey_initialized_b", state.odyssey_initialized_b, first);
    manta::json_field(out, "odyssey_sim_started_b", state.odyssey_sim_started_b, first);
    manta::json_field(out, "ready_published_b", state.ready_published_b, first);
    manta::json_field(out, "sim_time_initialized_b", state.sim_time_initialized_b, first);
    manta::json_field(out, "sim_time_received_b", state.sim_time_received_b, first);
    manta::json_field(out, "startup_ready_b", state.startup_ready_b, first);
    manta::json_field(out, "pending_sim_step", state.pending_sim_step, first);
    manta::json_field(out, "completed_cycle", state.completed_cycle, first);
    manta::json_field(out, "calendar_start_time", state.calendar_start_time, first);
    manta::json_field(out, "sim_time_s", state.sim_time_s, first);
    manta::json_field(out, "vehicle_id", state.vehicle_id, first);
    manta::json_field(out, "data_directory", state.data_directory, first);
    manta::json_field(out, "output_sample_period_s", state.output_sample_period_s, first);
    manta::json_field(out, "next_prediction_log_time_s", state.next_prediction_log_time_s, first);
    out << '}';
}

#endif
