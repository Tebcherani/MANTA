#ifndef SIM_ORCHESTRATOR__SIM_ORCHESTRATOR_STATE_JSON_HPP_
#define SIM_ORCHESTRATOR__SIM_ORCHESTRATOR_STATE_JSON_HPP_

#include <algorithm>
#include <string>
#include <unordered_set>
#include <vector>

#include "manta_interfaces/state_log.hpp"
#include "sim_orchestrator/sim_orchestrator_state.hpp"

inline void write_json(std::ostream &out, const VehicleState &state) {
    bool first = true;
    out << '{';
    manta::json_field(out, "ready_b", state.ready_b, first);
    manta::json_field(out, "active_b", state.active_b, first);
    manta::json_field(out, "step_complete_b", state.step_complete_b, first);
    out << '}';
}

inline void write_json(std::ostream &out, const sim_config_t &state) {
    bool first = true;
    out << '{';
    manta::json_field(out, "timestep_s", state.timestep_s, first);
    manta::json_field(out, "num_cycles", state.num_cycles, first);
    manta::json_field(out, "control_period", state.control_period, first);
    out << '}';
}

inline void write_json(std::ostream &out, const sim_state_t &state) {
    bool first = true;
    out << '{';
    manta::json_field(out, "current_cycle", state.current_cycle, first);
    manta::json_field(out, "sim_time_s", state.sim_time_s, first);
    manta::json_field(out, "calendar_start_time", state.calendar_start_time, first);
    out << '}';
}

inline void write_json(std::ostream &out, const world_state_t &state) {
    bool first = true;
    out << '{';
    manta::json_field(out, "gravity_mps2", state.gravity_mps2, first);
    manta::json_field(out, "magnetic_north_gauss", state.magnetic_north_gauss, first);
    manta::json_field(out, "magnetic_east_gauss", state.magnetic_east_gauss, first);
    manta::json_field(out, "magnetic_down_gauss", state.magnetic_down_gauss, first);
    manta::json_field(out, "seawater_density_kgpm3", state.seawater_density_kgpm3, first);
    out << '}';
}

namespace manta_interfaces { namespace msg {
inline void write_json(std::ostream &out, const SimStepResult &state) {
    bool first = true;
    out << '{';
    manta::json_field(out, "vehicle_id", state.vehicle_id, first);
    manta::json_field(out, "completed_cycle", state.completed_cycle, first);
    manta::json_field(out, "status", static_cast<unsigned int>(state.status), first);
    manta::json_field(out, "detail", state.detail, first);
    out << '}';
}
}}

namespace acoustics_interfaces { namespace msg {
inline void write_json(std::ostream &out, const BearingProcessed &state) {
    bool first = true;
    out << '{';
    manta::json_field(out, "vehicle_id", state.vehicle_id, first);
    manta::json_field(out, "call_cycle", state.call_cycle, first);
    manta::json_field(out, "sim_time_s", state.sim_time_s, first);
    manta::json_field(out, "estimate_valid_b", state.estimate_valid_b, first);
    out << '}';
}

inline void write_json(std::ostream &out, const WhaleCallProcessed &state) {
    bool first = true;
    out << '{';
    manta::json_field(out, "whale_id", state.whale_id, first);
    manta::json_field(out, "cycle", state.cycle, first);
    manta::json_field(out, "sim_time_s", state.sim_time_s, first);
    manta::json_field(out, "bearing_receiver_ids", state.bearing_receiver_ids, first);
    out << '}';
}
}}

#ifndef MANTA_WHALE_CALL_JSON_DEFINED
#define MANTA_WHALE_CALL_JSON_DEFINED
namespace whale_interfaces { namespace msg {
inline void write_json(std::ostream &out, const WhaleCall &state) {
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

inline void write_sorted_string_set_field(
    std::ostream &out, const char *name, const std::unordered_set<std::string> &values, bool &first) {
    std::vector<std::string> sorted(values.begin(), values.end());
    std::sort(sorted.begin(), sorted.end());
    manta::json_field(out, name, sorted, first);
}

inline void write_json(std::ostream &out, const SimOrchestratorState &state) {
    bool first = true;
    out << '{';
    manta::json_field(out, "simulation_started_b", state.simulation_started_b, first);
    manta::json_field(out, "simulation_failed_b", state.simulation_failed_b, first);
    manta::json_field(out, "steps_per_message", state.steps_per_message, first);
    manta::json_field(out, "communication_steps", state.communication_steps, first);
    manta::json_field(out, "current_batch_steps", state.current_batch_steps, first);
    manta::json_field(out, "sim_config", state.sim_config, first);
    manta::json_field(out, "sim_state", state.sim_state, first);
    if (!first) out << ',';
    first = false;
    manta::write_json(out, std::string("world_state"));
    out << ':';
    if (state.world_state != nullptr) write_json(out, *state.world_state);
    else out << "null";
    manta::json_field(out, "data_directory", state.data_directory, first);
    manta::json_field(out, "vehicle_ids", state.vehicle_ids, first);
    manta::json_field(out, "vehicle_states", state.vehicle_states, first);
    manta::json_field(out, "wait_for_whale_estimators_b", state.wait_for_whale_estimators_b, first);
    write_sorted_string_set_field(out, "ready_whale_estimators", state.ready_whale_estimators, first);
    manta::json_field(out, "received_step_result", state.received_step_result, first);
    manta::json_field(out, "expected_step_vehicle_id", state.expected_step_vehicle_id, first);
    manta::json_field(out, "next_whale_call_available_b", state.next_whale_call_available_b, first);
    manta::json_field(out, "current_batch_reaches_whale_call_b", state.current_batch_reaches_whale_call_b, first);
    manta::json_field(out, "waiting_for_whale_call_processing_b", state.waiting_for_whale_call_processing_b, first);
    manta::json_field(out, "whale_detection_complete_b", state.whale_detection_complete_b, first);
    manta::json_field(out, "wait_for_bearing_processing_b", state.wait_for_bearing_processing_b, first);
    write_sorted_string_set_field(out, "expected_bearing_processors", state.expected_bearing_processors, first);
    write_sorted_string_set_field(out, "completed_bearing_processors", state.completed_bearing_processors, first);
    manta::json_field(out, "received_bearing_processed", state.received_bearing_processed, first);
    manta::json_field(out, "received_whale_call_processed", state.received_whale_call_processed, first);
    manta::json_field(out, "next_whale_call", state.next_whale_call, first);
    out << '}';
}

#endif
