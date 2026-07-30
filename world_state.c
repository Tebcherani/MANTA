#include "world_state.h"
#include "sim_log.h"

int init_world_state(world_state_t *state) {
    if (state == NULL) {
        write_to_sim_log(
            "Failed to initialize world state: "
            "world_state pointer is NULL.\n");
        return -1;
    }

    state->temp = 0;
    write_to_sim_log("World state initialized.\n");

    return 0;
}