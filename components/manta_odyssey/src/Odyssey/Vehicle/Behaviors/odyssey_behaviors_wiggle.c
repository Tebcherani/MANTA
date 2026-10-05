#include <math.h>

#include "layered_control.h"
#include "odyssey_behaviors.h"
#include "odyssey_data_struct.h"
#include "odyssey_mission.h"
#include "vehicle.h"

int wiggle(struct behavior *b, int priority, int level) {
    register struct command *cmd = &vvp->c[priority];
    enum {TIME, PERIOD, R_AMP, E_AMP, THRUST};

    double reduced_time;

    if (b->state == B_UNINITIALIZED) {
	    if (stack_busy(cmd)) return -1;
	    b->state = B_ACTIVE;
    }

    if (manage_time(b, bArg(TIME)) == 1) {
	    b->state = B_COMPLETE;
	    return 1;
    }

	// Reduced time corresponds to fraction of this cycle elapsed (0 <= rt < 1)
    reduced_time = fmod(read_sensor(M_PRESENT_TIME) - b->start_time, bArg(PERIOD)) / bArg(PERIOD);

    cmd->speed_mode = S_CURRENT;
    cmd->speed[0] = bArg(THRUST);

    cmd->depth_mode = D_ELEVATOR;
    cmd->depth[0] = fabs(bArg(E_AMP));
	
    // Change elevator during 2nd half of cycle
    if (0.5 <= reduced_time) cmd->depth[0] *= -1;

    cmd->heading_mode = H_RUDDER;
    cmd->heading[0] = fabs(bArg(R_AMP));

	// Change rudder during mid half of cycle
    if (0.25 <= reduced_time && reduced_time < 0.75) cmd->heading[0] *= -1;
    
    return 0;
}
