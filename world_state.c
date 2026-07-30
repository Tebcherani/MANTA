#include "world_state.h"
#include "sim_log.h"

int init_world_state(world_state_t *state) {
    if (state == NULL) {
        write_to_sim_log(
            "Failed to initialize world state: "
            "state pointer is NULL.\n");
        return -1;
    }

    state->temp = 0;
    write_to_sim_log("World state initialized.\n\n");

    return 0;
}

int update_world_state(world_state_t *state, double sim_time) {
    if (state == NULL) {
        write_to_sim_log(
            "Failed to update world state: "
            "state pointer is NULL.\n"
        );
        return -1;
    }

    /*
     * World models will eventually be updated here.
     */
    (void)sim_time;

    return 0;
}