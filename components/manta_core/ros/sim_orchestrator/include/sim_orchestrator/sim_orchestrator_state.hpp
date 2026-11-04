#ifndef SIM_ORCHESTRATOR__SIM_ORCHESTRATOR_STATE_HPP_
#define SIM_ORCHESTRATOR__SIM_ORCHESTRATOR_STATE_HPP_

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "whale_interfaces/msg/whale_call.hpp"
#include "whale_interfaces/msg/whale_step_result.hpp"
#include "manta_interfaces/msg/sim_step.hpp"
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
    manta_interfaces::msg::SimStep current_step;
    bool waiting_for_whale_step_b = false;

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
    whale_interfaces::msg::WhaleStepResult received_whale_step_result;
    bool whale_ready_b = false;
    bool detection_ready_b = false;

    // Whale calls
    bool current_batch_has_whale_call_b = false;
    whale_interfaces::msg::WhaleCall current_whale_call;
};

#endif
