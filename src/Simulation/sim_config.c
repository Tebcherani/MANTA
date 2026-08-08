#include <stdlib.h>

#include "sim_config.h"
#include "sim_log.h"

int init_sim_config(sim_config_t *config) {
    if (config == NULL) {
        write_to_sim_log("[sim_config] Sim config failed: config is NULL.\n");
        return 0;
    }

    config->num_cycles = 1000000;
    config->timestep = 0.02;
    config->control_period = 10;

    write_to_sim_log("[sim_config] Sim configured.\n");
    return 1;
}