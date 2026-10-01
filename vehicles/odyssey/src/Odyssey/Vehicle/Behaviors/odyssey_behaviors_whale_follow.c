#include <math.h>

#include "angle_utils.h"
#include "layered_control.h"
#include "odyssey_behaviors.h"
#include "odyssey_data_struct.h"
#include "odyssey_mission.h"
#include "vehicle.h"

static int target_available = 0;
static double target_north = 0.0;
static double target_east = 0.0;

void set_whale_follow_target(double north, double east) {
    target_north = north;
    target_east = east;
    target_available = 1;
}

/*
behavior: whale_follow
  b_arg: depth(m) 0
  b_arg: speed(m/s) 0
  b_arg: capture(m) 5
*/

int whale_follow(struct behavior *b, int priority, int level) {
    enum {
        DEPTH,
        SPEED,
        CAPTURE
    };

    struct command *cmd = &vvp->c[priority];
    double north_difference;
    double east_difference;
    double distance;

    (void)level;

    if (!target_available) return -1;

    if (b->state == B_UNINITIALIZED) b->state = B_ACTIVE;

    north_difference = target_north - read_sensor(M_POS_N);
    east_difference = target_east - read_sensor(M_POS_E);
    distance = sqrt(north_difference * north_difference + east_difference * east_difference);

    update_sensor(WP_DTW, distance);
    update_sensor(WP_XTE, 0.0);

    cmd->depth_mode = 1;
    cmd->heading_mode = 1;
    cmd->speed_mode = 1;

    cmd->depth[0] = bArg(DEPTH);
    cmd->heading[0] = wrap_angle(atan2(east_difference, north_difference));
    cmd->speed[0] = distance <= fabs(bArg(CAPTURE)) ? 0.0 : bArg(SPEED);

    return 0;
}