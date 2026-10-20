#ifndef WHALE_DETECTION_SIM__STATE_JSON_HPP_
#define WHALE_DETECTION_SIM__STATE_JSON_HPP_

#include "manta_interfaces/state_log.hpp"
#include "whale_detection_sim/whale_detection_state.hpp"

#ifndef MANTA_WHALE_CALL_JSON_DEFINED
#define MANTA_WHALE_CALL_JSON_DEFINED
namespace whale_interfaces { namespace msg {
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
}}
#endif

namespace manta_interfaces { namespace msg {
inline void write_json(std::ostream &out, const VehicleState &state) {
    using manta::write_json;
    bool first = true;
    out << '{';
    manta::json_field(out, "vehicle_id", state.vehicle_id, first);
    manta::json_field(out, "cycle", state.cycle, first);
    manta::json_field(out, "sim_time_s", state.sim_time_s, first);
    manta::json_field(out, "vehicle_north_m", state.vehicle_north_m, first);
    manta::json_field(out, "vehicle_east_m", state.vehicle_east_m, first);
    manta::json_field(out, "depth", state.depth, first);
    out << '}';
}
}}

namespace acoustics_interfaces { namespace msg {
inline void write_json(std::ostream &out, const WhaleDetection &state) {
    using manta::write_json;
    bool first = true;
    out << '{';
    manta::json_field(out, "vehicle_id", state.vehicle_id, first);
    manta::json_field(out, "whale_id", state.whale_id, first);
    manta::json_field(out, "cycle", state.cycle, first);
    manta::json_field(out, "sim_time_s", state.sim_time_s, first);
    manta::json_field(out, "vehicle_north_m", state.vehicle_north_m, first);
    manta::json_field(out, "vehicle_east_m", state.vehicle_east_m, first);
    manta::json_field(out, "whale_north_m", state.whale_north_m, first);
    manta::json_field(out, "whale_east_m", state.whale_east_m, first);
    manta::json_field(out, "vehicle_to_whale_range_m", state.vehicle_to_whale_range_m, first);
    out << '}';
}
}}

inline void write_json(std::ostream &out, const WhaleDetectionState &state) {
    using manta::write_json;
    bool first = true;
    out << '{';
    manta::json_field(out, "vehicle_ids", state.vehicle_ids, first);
    manta::json_field(out, "detection_range_m", state.detection_range_m, first);
    manta::json_field(out, "bearing_std_rad", state.bearing_std_rad, first);
    manta::json_field(out, "bearing_noise_seed", state.bearing_noise_seed, first);
    manta::json_field(out, "data_directory", state.data_directory, first);
    manta::json_field(out, "shutdown_requested_b", state.shutdown_requested_b, first);
    manta::json_field(out, "pending_whale_call_available_b", state.pending_whale_call_available_b, first);
    manta::json_field(out, "latest_vehicle_states", state.latest_vehicle_states, first);
    manta::json_field(out, "pending_whale_call", state.pending_whale_call, first);
    manta::json_field(out, "whale_detection_available_b", state.whale_detection_available_b, first);
    manta::json_field(out, "whale_detection", state.whale_detection, first);
    out << '}';
}

#endif
