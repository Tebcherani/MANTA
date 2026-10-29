#ifndef WORLD_SIM_H
#define WORLD_SIM_H

typedef struct {
    double gravity_mps2;
    double magnetic_north_gauss;
    double magnetic_east_gauss;
    double magnetic_down_gauss;
    double seawater_density_kgpm3;
} world_state_t;

int init_world_sim(void);
const world_state_t *get_world_state(void);

#endif