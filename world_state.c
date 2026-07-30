#include "world_state.h"
#include "sim_log.h"

int init_world_state(world_state_t *world_state) {
    if (world_state == NULL) {
        write_to_sim_log(
            "Failed to initialize world state: "
            "world_state pointer is NULL.\n");
        return -1;
    }

    world_state->current_north = 0.0;
    world_state->current_east = 0.0;
    world_state->current_down = 0.0;

    write_to_sim_log(
        "World state initialized.\n"
        "  Current north: %.2f m/s\n"
        "  Current east : %.2f m/s\n"
        "  Current down : %.2f m/s\n\n",
        world_state->current_north,
        world_state->current_east,
        world_state->current_down
    );

    return 0;
}