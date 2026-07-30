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

    if (config->num_cycles <= 0) {
        write_to_sim_log(
            "Failed to initialize simulation configuration: "
            "num_cycles must be greater than zero.\n"
        );
        return -1;
    }

    if (config->timestep <= 0.0) {
        write_to_sim_log(
            "Failed to initialize simulation configuration: "
            "timestep must be greater than zero.\n"
        );
        return -1;
    }

    write_to_sim_log(
        "Simulation configuration initialized.\n"
        "  Number of cycles: %d\n"
        "  Timestep: %.6f seconds\n",
        config->num_cycles,
        config->timestep
    );

    return 0;
}