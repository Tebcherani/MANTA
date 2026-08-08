#include <math.h>

#include "odyssey_control.h"
#include "odyssey_data_struct.h"
#include "odyssey_model.h"
#include "odyssey_mission.h"
#include "odyssey_nav.h"
#include "odyssey_sensors.h"
#include "odyssey_sim.h"
#include "vehicle.h"

static odyssey_state_t state;

int init_odyssey_sim(void) {
    int i;
    
    for (i=0; i<7; i++) {
        state.force_moment[i] = 0.0;
        state.position_orientation[i] = 0.0;
    }

    // Use data structure initial values to set initial position
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
    
    return 0;
}

int update_odyssey_cycle(void) {
    if (update_odyssey_sensors() != 0) return -1;
    if (update_odyssey_nav() != 0) return -1;
    if (update_odyssey_control() != 0) return -1;
    return update_odyssey_mission();
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