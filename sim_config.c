#include <stdlib.h>

#include "sim_config.h"

int init_sim(sim_config_t *config) {
    if (config == NULL) return -1;

    // Simulation defaults
    config->num_cycles = 1000000;
    config->timestep = 0.02;

    return 0;
}