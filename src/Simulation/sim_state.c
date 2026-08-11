#include <time.h>

#include "sim_log.h"
#include "sim_state.h"

int init_sim_state(sim_state_t *state) {
    if (state == NULL) {
        write_to_sim_log("[sim_state] Sim state init failed: state is NULL.\n");
        return 0;
    }

    state->current_cycle = 0;
    state->sim_time = 0.0;
    state->calendar_start_time = time(NULL);

    if (state->calendar_start_time == (time_t)-1) {
        write_to_sim_log("[sim_state] Failed to get calendar start time.\n");
        return 0;
    }

    write_to_sim_log("[sim_state] Sim state initialized.\n");
    return 1;
}

int update_sim_state(sim_state_t *state, double timestep)
{
    if (state == NULL) return 0;

    state->current_cycle++;
    state->sim_time += timestep;

    return 1;
}