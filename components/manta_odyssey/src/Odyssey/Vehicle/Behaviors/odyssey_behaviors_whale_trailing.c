#include <math.h>

#include "angle_utils.h"
#include "layered_control.h"
#include "odyssey_behaviors.h"
#include "odyssey_data_struct.h"
#include "odyssey_mission.h"
#include "vehicle.h"

/*
 * The ROS whale estimator owns position and velocity filtering. Odyssey keeps
 * its latest published estimate and projects it forward for control updates.
 * Positions are local north/east metres; times are simulated seconds.
 */
typedef struct {
    int available;
    double measurement_time;
    double north;
    double east;
    double velocity_north;
    double velocity_east;
    double nominal_north;
    double nominal_east;
    double return_time;
} whale_guidance_estimate_t;

static whale_guidance_estimate_t whale_estimate = {0};
static double direction_north = 1.0;
static double direction_east = 0.0;

void set_whale_trailing_estimate(double north, double east, double measurement_time,
                                 double velocity_north, double velocity_east,
                                 double velocity_variance_north, double velocity_variance_east,
                                 double nominal_north, double nominal_east, double return_time) {
    double speed;
    if (!isfinite(north) || !isfinite(east) || !isfinite(measurement_time) ||
        !isfinite(velocity_north) || !isfinite(velocity_east) ||
        !isfinite(velocity_variance_north) || !isfinite(velocity_variance_east) ||
        !isfinite(nominal_north) || !isfinite(nominal_east) ||
        !isfinite(return_time) || return_time <= 0.0 ||
        velocity_variance_north < 0.0 || velocity_variance_east < 0.0 ||
        (whale_estimate.available && measurement_time <= whale_estimate.measurement_time)) return;
    whale_estimate = (whale_guidance_estimate_t){
        1, measurement_time, north, east, velocity_north, velocity_east,
        nominal_north, nominal_east, return_time
    };
    speed = hypot(velocity_north, velocity_east);
    if (speed > 1.0e-6) {
        direction_north = velocity_north / speed;
        direction_east = velocity_east / speed;
    }
}

static double predict_axis_position(double position, double velocity,
                                    double nominal_velocity, double elapsed) {
    const double travel_time = -whale_estimate.return_time *
        expm1(-elapsed / whale_estimate.return_time);
    return position + nominal_velocity * elapsed +
        (velocity - nominal_velocity) * travel_time;
}

static double predict_axis_velocity(double velocity, double nominal_velocity,
                                    double elapsed) {
    return nominal_velocity + (velocity - nominal_velocity) *
        exp(-elapsed / whale_estimate.return_time);
}

/*
behavior: whale_trailing
  b_arg: depth(m) 0
  b_arg: speed(m/s) 0
  b_arg: capture(m) 5
  b_arg: following_distance(m) 0
  b_arg: side_offset(m) 0
*/

/* Project the latest ROS estimate without changing its measurement time. */
int get_whale_trailing_prediction(double prediction_time, double *measurement_time,
                                  double *north, double *east) {
    double elapsed;
    if (!whale_estimate.available || measurement_time == NULL || north == NULL || east == NULL ||
        !isfinite(prediction_time)) return 0;
    elapsed = prediction_time - whale_estimate.measurement_time;
    if (!isfinite(elapsed) || elapsed < 0.0) return 0;
    *measurement_time = whale_estimate.measurement_time;
    *north = predict_axis_position(whale_estimate.north, whale_estimate.velocity_north,
                                   whale_estimate.nominal_north, elapsed);
    *east = predict_axis_position(whale_estimate.east, whale_estimate.velocity_east,
                                  whale_estimate.nominal_east, elapsed);
    return isfinite(*north) && isfinite(*east);
}

/* Convert the whale prediction into depth, heading, and speed commands. */
int whale_trailing(struct behavior *b, int priority, int level) {
    enum {
        DEPTH,
        SPEED,
        CAPTURE,
        FOLLOWING_DISTANCE,
        SIDE_OFFSET
    };

    /* Write commands into this behavior's layer of the vehicle controller. */
    struct command *cmd = &vvp->c[priority];
    double north_difference;
    double east_difference;
    double distance;

    (void)level;

    if (b->state == B_UNINITIALIZED) b->state = B_ACTIVE;

    double measurement_time;
    double predicted_north;
    double predicted_east;

    const double now = read_sensor(M_PRESENT_TIME);

    /* Hold position until a valid whale prediction is available. */
    if (!get_whale_trailing_prediction(now, &measurement_time, &predicted_north, &predicted_east)) {
        cmd->depth_mode = 1;
        cmd->heading_mode = 1;
        cmd->speed_mode = 1;

        cmd->depth[0] = bArg(DEPTH);
        cmd->heading[0] = read_sensor(M_COMPASS_YAW);
        cmd->speed[0] = 0.0;

        return 0;
    }

    /* Keep formation orientation consistent with the predicted velocity. */
    const double elapsed = now - whale_estimate.measurement_time;
    const double current_velocity_north = predict_axis_velocity(
        whale_estimate.velocity_north, whale_estimate.nominal_north, elapsed);
    const double current_velocity_east = predict_axis_velocity(
        whale_estimate.velocity_east, whale_estimate.nominal_east, elapsed);
    const double current_speed = hypot(current_velocity_north, current_velocity_east);
    if (current_speed > 1.0e-6) {
        direction_north = current_velocity_north / current_speed;
        direction_east = current_velocity_east / current_speed;
    }

    /* Translate body-relative formation offsets into north/east coordinates:
     * following is behind the predicted motion, and side is to its right.
     */
    const double following = bArg(FOLLOWING_DISTANCE);
    const double side = bArg(SIDE_OFFSET);

    // direction_* points forward. The side vector (-direction_east, direction_north) points to the whale's right.
    const double formation_north = predicted_north - following * direction_north - side * direction_east;
    const double formation_east = predicted_east - following * direction_east + side * direction_north;

    north_difference = formation_north - read_sensor(M_POS_N);
    east_difference = formation_east - read_sensor(M_POS_E);
    distance = sqrt(north_difference * north_difference + east_difference * east_difference);

    /* Publish distance-to-target for behavior monitoring; no line cross-track error is used. */
    update_sensor(WP_DTW, distance);
    update_sensor(WP_XTE, 0.0);

    cmd->depth_mode = 1;
    cmd->heading_mode = 1;
    cmd->speed_mode = 1;

    cmd->depth[0] = bArg(DEPTH);
    /* Steer toward the formation point and stop propulsion inside the capture radius. */
    cmd->heading[0] = wrap_angle(atan2(east_difference, north_difference));
    cmd->speed[0] = distance <= fabs(bArg(CAPTURE)) ? 0.0 : bArg(SPEED);

    return 0;
}
