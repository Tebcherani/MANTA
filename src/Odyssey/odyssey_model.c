#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ody_defs.h"
#include "odyssey_data_struct.h"
#include "ody_func.h"
#include "odyssey_model.h"
#include "ody_var.h"
#include "sim_log.h"
#include "vehicle.h"

static odyssey_state_t state;

int convert_units(const char *expected, const char *given, const double inval, double *outval);
int match_sensor(const char *name);
int load_vehicle_def(const char *fname);

int init_odyssey_model(void) {
    const char *model_path;

    model_path = getenv("VEHICLE_PATH");
    if (model_path == NULL || model_path[0] == '\0') {
        write_to_sim_log("[odyssey_model] VEHICLE_PATH is not set.\n");
        return 0;
    }

    if (load_vehicle_def(model_path) != 0) {
        write_to_sim_log("[odyssey_model] Failed to load vehicle definition.\n");
        return 0;
    }

    write_to_sim_log("[odyssey_model] Ody model initialized.\n");
    return 1;
}

int init_odyssey_state(void) {
    int i;
    
    for (i=0; i<7; i++) {
        state.force_moment[i] = 0.0;
        state.position_orientation[i] = 0.0;
    }

    update_sensor(REAL_X, read_sensor(M_POS_N));
    update_sensor(REAL_Y, read_sensor(M_POS_E));
    update_sensor(REAL_Z, read_sensor(M_DEPTH));

    state.position_orientation[1] = read_sensor(REAL_X);
    state.position_orientation[2] = read_sensor(REAL_Y);
    state.position_orientation[3] = read_sensor(REAL_Z);
    state.position_orientation[6] = read_sensor(REAL_PSI); 
    
    state.force_moment[1] = read_sensor(REAL_U);
    state.force_moment[2] = read_sensor(REAL_V);
    state.force_moment[3] = read_sensor(REAL_W);
    
    write_to_sim_log("[odyssey_model] Ody state initialized.\n");
    return 1;
}

int update_odyssey_state(double timestep) {
    double thruster_current;
    double rudder_angle;
    double elevator_angle;
    
    thruster_current = read_sensor(C_THRUSTER_CURRENT);
    rudder_angle = read_sensor(C_RUDDER_ANGLE);
    elevator_angle = read_sensor(C_ELEVATOR_ANGLE);

    motion(
        state.force_moment,
        state.position_orientation,
        thruster_current,
        rudder_angle,
        elevator_angle,
        timestep
    );

    if (fabs(state.force_moment[1]) +
        fabs(state.force_moment[2]) +
        fabs(state.force_moment[3]) +
        fabs(state.force_moment[4]) +
        fabs(state.force_moment[5]) +
        fabs(state.force_moment[6]) > 1.e2
    ) return -1;

    return 0;
}