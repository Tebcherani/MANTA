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

world_result_t update_world_state(world_state_t *state, const world_step_request_t *request) {
    world_result_t result;

    /*
     * World models will eventually be updated here.
     */
    (void)state;
    (void)request;

    result.status = 0;

    return result;
}