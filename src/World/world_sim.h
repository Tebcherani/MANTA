#ifndef WORLD_SIM_H
#define WORLD_SIM_H

#include "world_protocol.h"
#include "world_state.h"

typedef struct {
    world_state_t state;
} world_sim_t;

int init_world_sim(world_sim_t *simulator);

world_result_t step_world_sim(world_sim_t *simulator, const world_step_request_t *request);

#endif