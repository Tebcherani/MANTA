#ifndef WORLD_STATE_H
#define WORLD_STATE_H

typedef struct {
    int temp;
} world_state_t;

int init_world_state(world_state_t *state);
int update_world_state(world_state_t *state, double timestep);

#endif