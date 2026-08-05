#ifndef SIM_STATE_H
#define SIM_STATE_H

#include <time.h>

typedef struct {
    int initialized;
    int current_cycle;
    double sim_time;
    time_t calendar_start_time;
} sim_state_t;

int init_mission_time(sim_state_t *state);
int init_sim_state(sim_state_t *state);
int update_sim_state(sim_state_t *state, double timestep);

#endif