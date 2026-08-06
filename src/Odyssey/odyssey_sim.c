#include <math.h>

#include "odyssey_control.h"
#include "odyssey_mission.h"
#include "odyssey_nav.h"
#include "odyssey_sensors.h"
#include "odyssey_sim.h"
#include "vehicle.h"

typedef struct {
    double force_moment[7];
    double position_orientation[7];
} odyssey_state_t;

static odyssey_state_t state;

int init_odyssey_sim(void) {
    int i;
    
    for (i=0; i<7; i++) {
        state.force_moment[i] = 0.0;
        state.position_orientation[i] = 0.0;
    }

    // Use data structure initial values to set initial position
    update(REAL_X, sensor(M_POS_N));
    update(REAL_Y, sensor(M_POS_E));
    update(REAL_Z, sensor(M_DEPTH));

    state.position_orientation[1] = sensor(REAL_X);
    state.position_orientation[2] = sensor(REAL_Y);
    state.position_orientation[3] = sensor(REAL_Z);
    state.position_orientation[6] = sensor(REAL_PSI); 
    
    state.force_moment[1] = sensor(REAL_U);
    state.force_moment[2] = sensor(REAL_V);
    state.force_moment[3] = sensor(REAL_W);
    
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
    
    thruster_current = sensor(C_THRUSTER_CURRENT);
    rudder_angle = sensor(C_RUDDER_ANGLE);
    elevator_angle = sensor(C_ELEVATOR_ANGLE);

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