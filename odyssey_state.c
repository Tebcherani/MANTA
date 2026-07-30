#include "odyssey_state.h"
#include "sim_log.h"

#include "vehicle.h"

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

    state->position_orientation[1] = sensor(REAL_X);
    state->position_orientation[2] = sensor(REAL_Y);
    state->position_orientation[3] = sensor(REAL_Z);
    state->position_orientation[6] = sensor(REAL_PSI); 
    
    state->force_moment[1] = sensor(REAL_U);
    state->force_moment[2] = sensor(REAL_V);
    state->force_moment[3] = sensor(REAL_W);

    write_to_sim_log(
        "Odyssey state initialized.\n"
        "  North   : %.2lf\n"
        "  East    : %.2lf\n"
        "  Depth   : %.2lf\n"
        "  Roll    : %.2lf\n"
        "  Pitch   : %.2lf\n"
        "  Heading : %.2lf\n",
        state->position_orientation[1],
        state->position_orientation[2],
        state->position_orientation[3],
        state->position_orientation[4],
        state->position_orientation[5],
        state->position_orientation[6]
    );
    
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