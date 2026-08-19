#include <stdlib.h>

#include "sim_config.h"
#include "sim_log.h"

int init_sim_config(sim_config_t *config) {
    if (config == NULL) return 0;

    config->num_cycles = 1000000;
    config->timestep = 0.02;
    config->control_period = 10;

    return 1;
}