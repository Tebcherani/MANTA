#include <stdlib.h>

#include "sim_config.h"
#include "sim_log.h"

int init_sim(sim_config_t *config) {
    if (config == NULL) {
        write_to_sim_log(
            "Failed to initialize simulation configuration: "
            "config pointer is NULL.\n"
        );
        return -1;
    }

    // Simulation defaults
    config->num_cycles = 1000000;
    config->timestep = 0.02;
    config->control_period = 10;

    write_to_sim_log(
        "Simulation configuration initialized.\n"
        "  Number of cycles: %d\n"
        "  Timestep: %.6f seconds\n"
        "  Control period: %d cycles\n\n",
        config->num_cycles,
        config->timestep,
        config->control_period
    );

    return 0;
}