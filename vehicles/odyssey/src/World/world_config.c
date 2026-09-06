#include <stddef.h>

#include "world_config.h"

int init_world_config(world_state_t *config) {
    if (config == NULL) return 0;

    config->gravity = 9.81;
    config->magnetic_north = 0.275;
    config->magnetic_east = 0.0;
    config->magnetic_down = 0.476;
    config->seawater_density = 1.025e3;

    return 1;
}