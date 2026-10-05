#include "world_sim.h"
#include "world_config.h"

static world_state_t world;

int init_world_sim(void) {
    if (!init_world_config(&world)) return 0;

    return 1;
}

const world_state_t *get_world_state(void) {
    return &world;
}