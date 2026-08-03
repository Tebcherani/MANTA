#include <time.h>

#include "sim_log.h"
#include "sim_state.h"

int init_sim_state(sim_state_t *state) {
    char time_string[32];
    struct tm *time_ptr;

    if (state == NULL) {
        write_to_sim_log("Failed to initialize simulation state: state pointer is NULL.\n");
        return -1;
    }

    state->current_cycle = 0;
    state->sim_time = 0.0;
    state->calendar_start_time = time(NULL);

    if (state->calendar_start_time == (time_t)-1) {
        write_to_sim_log("Failed to initialize simulation state: could not obtain calendar start time.\n");
        return -1;
    }

    // Get time in UTC
    time_ptr = gmtime(&state->calendar_start_time);
    
    if (time_ptr == NULL) {
        write_to_sim_log("Failed to initialize simulation state: could not convert calendar start time.\n");
        return -1;
    }

    strftime(time_string, sizeof(time_string), "%b %d, %Y %H:%M:%S UTC", time_ptr);
    
    write_to_sim_log(
        "Simulation state initialized.\n"
        "  Simulation time: %.2f seconds\n"
        "  Current cycle: %d\n"
        "  Calendar start time: %s\n\n",
        state->sim_time,
        state->current_cycle,
        time_string
    );

    return 0;
}

int update_sim_state(sim_state_t *state, double time)
{
    if (state == NULL) {
        write_to_sim_log("Failed to update simulation state: state pointer is NULL.\n");
        return -1;
    }

    state->current_cycle++;
    state->sim_time += time;

    return 0;
}