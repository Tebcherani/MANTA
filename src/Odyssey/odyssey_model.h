#ifndef ODYSSEY_MODEL_H
#define ODYSSEY_MODEL_H

typedef struct {
    double force_moment[7];
    double position_orientation[7];
} odyssey_state_t;

int init_odyssey_model(const char *path);
int init_odyssey_state(void);
int init_odyssey_dynamics(void);
int update_odyssey_state(double timestep);

#endif