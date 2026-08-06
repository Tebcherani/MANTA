#include "odyssey_mission.h"
#include "vehicle.h"

int stack_busy(struct command *command);

int update_odyssey_mission(void) {
    if (stack_busy(&vvp->c[0])) return 0;
    vvp->cstate++;
    return vvp->cstate >= vvp->nstates;
}