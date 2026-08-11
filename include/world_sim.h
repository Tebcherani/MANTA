#ifndef WORLD_SIM_H
#define WORLD_SIM_H

typedef struct {
    double gravity;
    double magnetic_north;
    double magnetic_east;
    double magnetic_down;
    double seawater_density;
} world_state_t;

int init_world_sim(void);
const world_state_t *get_world_state(void);

#endif