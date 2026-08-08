#include <time.h>

#include "sim_log.h"
#include "sim_state.h"
#include "vehicle.h"
#include "odyssey_data_struct.h"

int hirestime(double *time_value);

int init_mission_time(sim_state_t *state) {
    struct tm *time_info;

    time_info = gmtime(&state->calendar_start_time);
    if (time_info == NULL) return -1;

    if (strftime(vvp->mission_date, sizeof(vvp->mission_date), "%b %d, %Y %H:%M:%S", time_info) == 0) return -1;
    if (hirestime(&vvp->mission_start_time) != 0) return -1;

    return 0;
}

int init_sim_state(sim_state_t *state) {
    char time_string[32];
    struct tm *time_ptr;

    if (state == NULL) return -1;

    state->current_cycle = 0;

    state->sim_time = 0.0;
    update_sensor(M_PRESENT_TIME, state->sim_time);
    sensor_time(M_PRESENT_TIME) = (long)(1000.0 * state->sim_time);

    state->calendar_start_time = time(NULL);
    if (state->calendar_start_time == (time_t)-1) return -1;

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

    update_sensor(M_PRESENT_TIME, state->sim_time);
    sensor_time(M_PRESENT_TIME) = (long)(1000.0 * state->sim_time);

    return 0;
}