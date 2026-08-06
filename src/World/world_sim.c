#include "world_sim.h"

int init_world_sim(world_sim_t *simulator) {
    return init_world_state(&simulator->state);
}

world_result_t step_world_sim(world_sim_t *simulator, const world_step_request_t *request) {
    return update_world_state(&simulator->state, request);
}