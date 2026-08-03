#ifndef ODYSSEY_STATE_H
#define ODYSSEY_STATE_H

typedef struct {
    double force_moment[7];
    double position_orientation[7];
} odyssey_state_t;

int init_odyssey_state(odyssey_state_t *state);
int update_odyssey_state(odyssey_state_t *state, double timestep);

#endif