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
static double target_time = 0.0;

static double velocity_north = 1.5;
static double velocity_east = 0.0;

void set_whale_trailing_target(
    double north,
    double east,
    double measurement_time
) {
    if (!isfinite(north) || !isfinite(east) ||
        !isfinite(measurement_time)) {
        return;
    }

    if (target_available) {
        const double dt = measurement_time - target_time;

        /* Ignore duplicate or out-of-order measurements. */
        if (dt <= 1.0e-6) return;

        /* Estimate average velocity between the latest two calls. */
        const double new_velocity_north =
            (north - target_north) / dt;
        const double new_velocity_east =
            (east - target_east) / dt;

        if (!isfinite(new_velocity_north) ||
            !isfinite(new_velocity_east)) {
            return;
        }

        velocity_north = new_velocity_north;
        velocity_east = new_velocity_east;
    }

    /* Store the latest measured position, not a predicted position. */
    target_north = north;
    target_east = east;
    target_time = measurement_time;
    target_available = 1;
}

/*
behavior: whale_trailing
  b_arg: depth(m) 0
  b_arg: speed(m/s) 0
  b_arg: capture(m) 5
  b_arg: following_distance(m) 0
  b_arg: side_offset(m) 0
*/

int whale_trailing(struct behavior *b, int priority, int level) {
    enum {
        DEPTH,
        SPEED,
        CAPTURE,
        FOLLOWING_DISTANCE,
        SIDE_OFFSET
    };

    struct command *cmd = &vvp->c[priority];
    double north_difference;
    double east_difference;
    double distance;

    (void)level;

    if (b->state == B_UNINITIALIZED) b->state = B_ACTIVE;

    // Keep the behavior active without commanding motion until a call is detected.
    if (!target_available) {
        cmd->depth_mode = 1;
        cmd->heading_mode = 1;
        cmd->speed_mode = 1;

        cmd->depth[0] = bArg(DEPTH);
        cmd->heading[0] = read_sensor(M_COMPASS_YAW);
        cmd->speed[0] = 0.0;

        return 0;
    }

        /* Predict the whale's position between calls. */
    const double now = read_sensor(M_PRESENT_TIME);
    const double elapsed = fmax(0.0, now - target_time);

    const double predicted_north = target_north + velocity_north * elapsed;
    const double predicted_east = target_east + velocity_east * elapsed;

    /* Apply the existing fixed north/east formation offsets. */
    const double formation_north = predicted_north - bArg(FOLLOWING_DISTANCE);
    const double formation_east = predicted_east + bArg(SIDE_OFFSET);

    north_difference = formation_north - read_sensor(M_POS_N);
    east_difference = formation_east - read_sensor(M_POS_E);
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