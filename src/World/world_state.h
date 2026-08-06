#ifndef WORLD_STATE_H
#define WORLD_STATE_H

#include "world_protocol.h"

typedef struct {
    int temp;
} world_state_t;

int init_world_state(world_state_t *state);
world_step_result_t update_world_state(world_state_t *state, const world_step_request_t *request);

#endif