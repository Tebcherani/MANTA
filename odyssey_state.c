#include "odyssey_state.h"
#include "sim_log.h"

int init_odyssey_state(odyssey_state_t *state) {
    int i;
    
    if (state == NULL) {
        write_to_sim_log(
            "Failed to initialize Odyssey state: "
            "state pointer is NULL.\n");
        return -1;
    }

    for (i=0; i<7; i++) {
        state->force_moment[i] = 0.0;
        state->position_orientation[i] = 0.0;
    }
    
    write_to_sim_log("Odysssey state initialized.\n");

    return 0;
}

int update_odyssey_state(odyssey_state_t *state, double sim_time) {
    if (state == NULL) {
        write_to_sim_log(
            "Failed to update Odyssey state: "
            "state pointer is NULL.\n"
        );
        return -1;
    }

    /*
     * Odyssey models will eventually be updated here.
     */
    (void)sim_time;

    return 0;
}