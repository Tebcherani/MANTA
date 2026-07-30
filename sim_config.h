#ifndef SIM_CONFIG_H
#define SIM_CONFIG_H

typedef struct {
    double timestep;
    int num_cycles;
} sim_config_t;

int init_sim(sim_config_t *config);

#endif