#ifndef SIM_ORCHESTRATOR__SIM_ORCHESTRATOR_STATE_HPP_
#define SIM_ORCHESTRATOR__SIM_ORCHESTRATOR_STATE_HPP_

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "acoustics_interfaces/msg/bearing_processed.hpp"
#include "acoustics_interfaces/msg/whale_call_processed.hpp"
#include "whale_interfaces/msg/whale_call.hpp"
#include "manta_interfaces/msg/sim_step_result.hpp"

extern "C" {
    #include "sim_config.h"
    #include "sim_state.h"
    #include "world_sim.h"
}

// ---------------------------------------------------------------------
// Structures
// ---------------------------------------------------------------------

struct VehicleState {
    bool ready_b = false;
    bool active_b = true;
    bool step_complete_b = false;
};

struct SimOrchestratorState {
    // Simulation
    bool simulation_started_b = false;
    bool simulation_failed_b = false;

    int steps_per_message = 1;
    int communication_steps = 0;
    int current_batch_steps = 0;

    sim_config_t sim_config;
    sim_state_t sim_state;
    const world_state_t *world_state = nullptr;
    std::string data_directory = "/odyssey/output/Data";

    // Vehicle
    std::vector<std::string> vehicle_ids;
    std::unordered_map<std::string, VehicleState> vehicle_states;
    bool wait_for_whale_estimators_b = false;
    std::unordered_set<std::string> ready_whale_estimators;
    manta_interfaces::msg::SimStepResult received_step_result;
    std::string expected_step_vehicle_id;

    // Whale calls
    bool next_whale_call_available_b = false;
    bool current_batch_reaches_whale_call_b = false;
    bool waiting_for_whale_call_processing_b = false;
    bool whale_detection_complete_b = false;
    bool wait_for_bearing_processing_b = false;

    std::unordered_set<std::string> expected_bearing_processors;
    std::unordered_set<std::string> completed_bearing_processors;

    acoustics_interfaces::msg::BearingProcessed received_bearing_processed;
    acoustics_interfaces::msg::WhaleCallProcessed received_whale_call_processed;
    whale_interfaces::msg::WhaleCall next_whale_call;
};

#endif
