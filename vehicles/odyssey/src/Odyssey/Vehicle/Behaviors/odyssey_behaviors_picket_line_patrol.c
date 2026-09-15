#include <math.h>

#include "angle_utils.h"
#include "layered_control.h"
#include "odyssey_behaviors.h"
#include "odyssey_data_struct.h"
#include "odyssey_mission.h"
#include "vehicle.h"

/*
behavior: picket_line_patrol
  b_arg: start_n(m) 0
  b_arg: start_e(m) 0
  b_arg: end_n(m) 0
  b_arg: end_e(m) 0
  b_arg: depth(m) 0
  b_arg: speed(m/s) 0
  b_arg: capture(m) 25
*/

int picket_line_patrol(struct behavior *b, int priority, int level) {
    enum {
        START_N,
        START_E,
        END_N,
        END_E,
        DEPTH,
        SPEED,
        CAPTURE
    };

    register struct command *cmd = &vvp->c[priority];

    static int travelling_to_end = 1;

    double target_n;
    double target_e;
    double dn;
    double de;
    double distance;

    (void)level;

    if (b->state == B_UNINITIALIZED) {
        if (stack_busy(cmd))
            return -1;

        travelling_to_end = 1;
        b->state = B_ACTIVE;
    }

    target_n = travelling_to_end ? bArg(END_N) : bArg(START_N);
    target_e = travelling_to_end ? bArg(END_E) : bArg(START_E);

    dn = target_n - read_sensor(M_POS_N);
    de = target_e - read_sensor(M_POS_E);
    distance = sqrt(dn * dn + de * de);

    /*
     * Reverse direction after entering the endpoint's capture radius.
     * Recalculate the target immediately so the next command points away
     * from the endpoint rather than continuing through it.
     */
    if (distance <= fabs(bArg(CAPTURE))) {
        travelling_to_end = !travelling_to_end;

        target_n = travelling_to_end ? bArg(END_N) : bArg(START_N);
        target_e = travelling_to_end ? bArg(END_E) : bArg(START_E);

        dn = target_n - read_sensor(M_POS_N);
        de = target_e - read_sensor(M_POS_E);
        distance = sqrt(dn * dn + de * de);
    }

    update_sensor(WP_DTW, distance);
    update_sensor(WP_XTE, 0.0);

    cmd->depth_mode = 1;
    cmd->heading_mode = 1;
    cmd->speed_mode = 1;

    cmd->depth[0] = bArg(DEPTH);
    cmd->heading[0] = wrap_angle(atan2(de, dn));
    cmd->speed[0] = bArg(SPEED);

    return 0;
}