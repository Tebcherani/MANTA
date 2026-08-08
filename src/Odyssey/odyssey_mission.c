#include "odyssey_mission.h"
#include "vehicle.h"

int stack_busy(struct command *cmd) {
    return (cmd->speed_mode != -1) && (cmd->heading_mode != -1) &&
	(cmd->depth_mode != -1);
}

int update_odyssey_mission(void) {
    if (stack_busy(&vvp->c[0])) return 0;
    vvp->cstate++;
    return vvp->cstate >= vvp->nstates;
}