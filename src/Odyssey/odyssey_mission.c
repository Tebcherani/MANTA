#include "diagnostic.h"
#include "odyssey_mission.h"
#include "sim_log.h"
#include "vehicle.h"

int stack_busy(struct command *command);

int update_odyssey_mission(void) {
    int completed_state;

    if (stack_busy(&vvp->c[0])) return 0;

    completed_state = vvp->cstate;
    vvp->cstate++;

    write_to_sim_log( "%8.2f: state %d completed\n", sensor(M_PRESENT_TIME), completed_state);

    if (vvp->cstate >= vvp->nstates) return 1;

    return 0;
}