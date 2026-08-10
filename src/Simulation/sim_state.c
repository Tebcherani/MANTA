#include <time.h>

#include "sim_log.h"
#include "sim_state.h"
#include "vehicle.h"
#include "odyssey_data_struct.h"

int init_sim_state(sim_state_t *state) {
    if (state == NULL) {
        write_to_sim_log("[sim_state] Sim state init failed: state is NULL.\n");
        return 0;
    }

    state->current_cycle = 0;
    state->sim_time = 0.0;

    update_sensor(M_PRESENT_TIME, state->sim_time);
    sensor_time(M_PRESENT_TIME) = (long)(1000.0 * state->sim_time);

    state->calendar_start_time = time(NULL);

    if (state->calendar_start_time == (time_t)-1) {
        write_to_sim_log("[sim_state] Failed to get calendar start time.\n");
        return 0;
    }

    write_to_sim_log("[sim_state] Sim state initialized.\n");
    return 1;
}

int update_sim_state(sim_state_t *state, double time)
{
    if (state == NULL) return -1;

    state->current_cycle++;
    state->sim_time += time;

    update_sensor(M_PRESENT_TIME, state->sim_time);
    sensor_time(M_PRESENT_TIME) = (long)(1000.0 * state->sim_time);

    return 0;
}