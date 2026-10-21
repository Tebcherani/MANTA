#include <math.h>

#include "angle_utils.h"
#include "layered_control.h"
#include "odyssey_behaviors.h"
#include "odyssey_data_struct.h"
#include "odyssey_mission.h"
#include "vehicle.h"

/*
 * This behavior consumes triangulated whale positions and steers an AUV toward
 * an offset from the predicted whale position. It does not move the real or
 * simulated whale. All positions use local north/east metres, and all times are
 * simulated measurement times in seconds.
 *
 * Accepted positions are treated as exact (R = 0). Velocity starts with a
 * mean-reverting Kalman estimate, then uses a rolling linear fit once at least
 * three recent fixes are available. Between fixes, velocity gradually returns
 * toward the nominal northward velocity below.
 */

/* Starting settings; tune acceleration PSD against whale ground truth. */
#define WHALE_VELOCITY_WINDOW_SIZE 5  /* Maximum accepted fixes in the fit. */
#define WHALE_VELOCITY_WINDOW_MAX_AGE_S 1800.0  /* Discard older fixes on arrival. */
#define WHALE_ACCELERATION_PSD 0.001  /* m^2/s^3 */
#define WHALE_VELOCITY_RETURN_TIME_S 1800.0  /* 30-minute e-folding time */
#define WHALE_NOMINAL_NORTH_VELOCITY 1.5
#define WHALE_NOMINAL_EAST_VELOCITY 0.0
#define WHALE_INITIAL_VELOCITY_VARIANCE 1.0  /* (m/s)^2 */

/* Posterior for one axis at the last accepted measurement time.
 * Exact position fixes make posterior position variance and position/velocity
 * covariance zero; velocity variance is the only covariance stored here.
 */
typedef struct {
    double position;
    double velocity;
    double velocity_variance;
} whale_axis_filter_t;

/* One accepted triangulated fix, timestamped at the call rather than receipt. */
typedef struct {
    double time;
    double north;
    double east;
} whale_position_fix_t;

/* Per-process estimator state; recent_fixes is ordered oldest to newest. */
typedef struct {
    int initialized;
    double measurement_time;
    whale_axis_filter_t north;
    whale_axis_filter_t east;
    whale_position_fix_t recent_fixes[WHALE_VELOCITY_WINDOW_SIZE];
    int recent_fix_count;
} whale_filter_t;

static whale_filter_t whale_filter = {0};

/* Last usable unit direction; retain it when predicted speed is near zero. */
static double direction_north = 1.0;
static double direction_east = 0.0;

/*
 * Integrate v(t) = nominal + (initial - nominal) * exp(-t / tau).
 * Integrating the changing velocity avoids a jump in position as velocity
 * returns to nominal. expm1 preserves precision for very small elapsed times.
 */
static double predict_whale_axis_position(const whale_axis_filter_t *axis, double elapsed, double nominal_velocity) {
    const double travel_time = -WHALE_VELOCITY_RETURN_TIME_S * expm1(-elapsed / WHALE_VELOCITY_RETURN_TIME_S);
    return axis->position + nominal_velocity * elapsed + (axis->velocity - nominal_velocity) * travel_time;
}

/* Query the model velocity without advancing the stored measurement state. */
static double predict_whale_axis_velocity(const whale_axis_filter_t *axis, double elapsed, double nominal_velocity) {
    return nominal_velocity + (axis->velocity - nominal_velocity) * exp(-elapsed / WHALE_VELOCITY_RETURN_TIME_S);
}

/*
 * Exact continuous-time mean-reverting prediction and an exact-position update
 * (R = 0). Position covariance and position/velocity covariance vanish after
 * each fix, so only posterior velocity variance is retained between fixes.
 */
static int update_whale_axis(whale_axis_filter_t *axis, double measured_position, double dt, double nominal_velocity) {
    const double tau = WHALE_VELOCITY_RETURN_TIME_S;
    const double u = dt / tau;
    const double decay = exp(-u);
    const double one_minus_decay = -expm1(-u);
    const double travel_time = tau * one_minus_decay;
    const double q = WHALE_ACCELERATION_PSD;

    /* Integrated acceleration-noise covariance: vv is velocity variance,
     * pv is position/velocity covariance, and pp is position variance.
     */
    const double noise_vv = -0.5 * q * tau * expm1(-2.0 * u);
    const double noise_pv = 0.5 * q * travel_time * travel_time;
    double noise_pp;
    if (u < 0.01) {
        /* Avoid cancellation in the integrated process noise at small dt. */
        noise_pp = q * dt * dt * dt * (1.0 / 3.0 + u * (-1.0 / 4.0 + u * (7.0 / 60.0 + u * (-1.0 / 24.0 + u * 31.0 / 2520.0))));
    } else {
        noise_pp = q * tau * tau * (dt - 2.0 * travel_time - 0.5 * tau * expm1(-2.0 * u));
    }

    /* Propagate the previous velocity uncertainty through the same motion model. */
    const double predicted_pp = travel_time * travel_time * axis->velocity_variance + noise_pp;
    const double predicted_pv = travel_time * decay * axis->velocity_variance + noise_pv;
    const double predicted_vv = decay * decay * axis->velocity_variance + noise_vv;
    if (!isfinite(predicted_pp) || predicted_pp <= 0.0 || !isfinite(predicted_pv) || !isfinite(predicted_vv)) return 0;

    /* With R = 0, position gain is one. This is the velocity gain: it turns
     * the position prediction error into a correction to predicted velocity.
     */
    const double gain = predicted_pv / predicted_pp;
    const double predicted_position = predict_whale_axis_position(axis, dt, nominal_velocity);
    const double predicted_velocity = predict_whale_axis_velocity(axis, dt, nominal_velocity);
    const double updated_velocity = predicted_velocity + gain * (measured_position - predicted_position);
    double updated_variance = predicted_vv - gain * predicted_pv;
    if (!isfinite(updated_velocity) || !isfinite(updated_variance) || updated_variance < -1.0e-12 * fmax(1.0, predicted_vv)) return 0;
    /* Permit tiny negative roundoff, but reject materially invalid covariance. */
    updated_variance = fmax(0.0, updated_variance);

    /* Anchor position exactly to the fix; velocity remains an estimate. */
    axis->position = measured_position;
    axis->velocity = updated_velocity;
    axis->velocity_variance = updated_variance;
    return 1;
}

/*
 * Fit separate north/east position-versus-time lines using recent fixes.
 * Their slopes estimate average velocity over the window, not instantaneous
 * velocity at its endpoint. All retained fixes have equal regression weight;
 * their actual timestamps handle irregular call and communication intervals.
 * The fitted velocity replaces the Kalman velocity rather than being fused as
 * an independent observation, because both estimates use the same position data.
 */
static void update_recent_whale_velocity(double north, double east, double time) {
    int i, j;
    int count = 0;
    /* Compact the window after outages so stale fixes do not influence reacquisition. */
    for (i = 0; i < whale_filter.recent_fix_count; ++i) {
        if (time - whale_filter.recent_fixes[i].time <= WHALE_VELOCITY_WINDOW_MAX_AGE_S) {
            whale_filter.recent_fixes[count++] = whale_filter.recent_fixes[i];
        }
    }
    /* If still full, remove the oldest remaining fix before appending this one. */
    if (count == WHALE_VELOCITY_WINDOW_SIZE) {
        for (i = 1; i < count; ++i) {
            whale_filter.recent_fixes[i - 1] = whale_filter.recent_fixes[i];
        }
        --count;
    }
    whale_filter.recent_fixes[count++] = (whale_position_fix_t){time, north, east};
    whale_filter.recent_fix_count = count;
    /* Leave the Kalman/prior velocity in place until a fit has residual degrees of freedom. */
    if (count < 3) return;

    /* Center relative coordinates to preserve precision at large positions/times. */
    const whale_position_fix_t origin = whale_filter.recent_fixes[count - 1];
    double mean_t = 0.0;
    double mean_position[2] = {0.0, 0.0};
    for (i = 0; i < count; ++i) {
        const whale_position_fix_t *fix = &whale_filter.recent_fixes[i];
        mean_t += (fix->time - origin.time) / count;
        mean_position[0] += (fix->north - origin.north) / count;
        mean_position[1] += (fix->east - origin.east) / count;
    }
    /* Least-squares slope is sum(centered_time * centered_position) / sum(time^2). */
    double time_ss = 0.0;
    double cross[2] = {0.0, 0.0};
    for (i = 0; i < count; ++i) {
        const whale_position_fix_t *fix = &whale_filter.recent_fixes[i];
        const double t = fix->time - origin.time - mean_t;
        time_ss += t * t;
        cross[0] += t * (fix->north - origin.north - mean_position[0]);
        cross[1] += t * (fix->east - origin.east - mean_position[1]);
    }
    if (!isfinite(time_ss) || time_ss <= 0.0) return;
    double velocity[2] = {cross[0] / time_ss, cross[1] / time_ss};
    /* Residuals describe how poorly a constant-velocity line fits this window. */
    double residual_ss[2] = {0.0, 0.0};
    for (i = 0; i < count; ++i) {
        const whale_position_fix_t *fix = &whale_filter.recent_fixes[i];
        const double t = fix->time - origin.time - mean_t;
        const double residual[2] = {
            fix->north - origin.north - mean_position[0] - velocity[0] * t,
            fix->east - origin.east - mean_position[1] - velocity[1] * t
        };
        for (j = 0; j < 2; ++j) residual_ss[j] += residual[j] * residual[j];
    }
    const double span = time - whale_filter.recent_fixes[0].time;
    double variance[2];
    for (j = 0; j < 2; ++j) {
        /* Residual slope uncertainty plus a process-noise allowance for turns.
         * This is a rolling-fit heuristic, not an independent Kalman observation.
         */
        variance[j] = residual_ss[j] / (count - 2) / time_ss + WHALE_ACCELERATION_PSD * span / 3.0;
        if (!isfinite(velocity[j]) || !isfinite(variance[j])) return;
    }
    whale_filter.north.velocity = velocity[0];
    whale_filter.east.velocity = velocity[1];
    whale_filter.north.velocity_variance = variance[0];
    whale_filter.east.velocity_variance = variance[1];
}

/*
 * Accept one valid same-cycle triangulated position from the ROS adapter.
 * Initialize on the first fix, otherwise predict/update to its measurement time.
 * Repeated or older timestamps are ignored before either filter or window changes.
 */
void set_whale_trailing_target(double north, double east, double measurement_time) {
    if (!isfinite(north) || !isfinite(east) || !isfinite(measurement_time)) return;

    if (!whale_filter.initialized) {
        whale_filter.north = (whale_axis_filter_t){north, WHALE_NOMINAL_NORTH_VELOCITY, WHALE_INITIAL_VELOCITY_VARIANCE};
        whale_filter.east = (whale_axis_filter_t){east, WHALE_NOMINAL_EAST_VELOCITY, WHALE_INITIAL_VELOCITY_VARIANCE};

        whale_filter.measurement_time = measurement_time;
        whale_filter.initialized = 1;
    } else {
        const double dt = measurement_time - whale_filter.measurement_time;

        /* Reject duplicate, stale, or invalid measurement times. */
        if (!isfinite(dt) || dt <= 1.0e-6) return;

        /*
         * Work on copies so a failed update cannot leave one axis
         * updated and the other unchanged.
         */
        whale_axis_filter_t next_north = whale_filter.north;
        whale_axis_filter_t next_east = whale_filter.east;

        if (!update_whale_axis(&next_north, north, dt, WHALE_NOMINAL_NORTH_VELOCITY) ||
            !update_whale_axis(&next_east, east, dt, WHALE_NOMINAL_EAST_VELOCITY)) {
            return;
        }

        whale_filter.north = next_north;
        whale_filter.east = next_east;
        whale_filter.measurement_time = measurement_time;
    }

    /* Once the window is populated, use its smoothed slope for subsequent prediction. */
    update_recent_whale_velocity(north, east, measurement_time);

    const double speed = hypot(
        whale_filter.north.velocity,
        whale_filter.east.velocity
    );

    if (speed > 1.0e-6) {
        direction_north = whale_filter.north.velocity / speed;
        direction_east = whale_filter.east.velocity / speed;
    }
}

/*
behavior: whale_trailing
  b_arg: depth(m) 0
  b_arg: speed(m/s) 0
  b_arg: capture(m) 5
  b_arg: following_distance(m) 0
  b_arg: side_offset(m) 0
*/

/*
 * Shared, read-only position prediction for guidance and output logging.
 * Keep the stored state at the last fix so delayed fixes can still be applied
 * chronologically. measurement_time reports fix age separately from prediction time.
 */
int get_whale_trailing_prediction(double prediction_time, double *measurement_time, double *north, double *east) {
    if (!whale_filter.initialized || !isfinite(prediction_time) || measurement_time == NULL || north == NULL || east == NULL) return 0;

    const double elapsed = prediction_time - whale_filter.measurement_time;

    if (!isfinite(elapsed) || elapsed < 0.0) return 0;

    const double predicted_north = predict_whale_axis_position(&whale_filter.north, elapsed, WHALE_NOMINAL_NORTH_VELOCITY);
    const double predicted_east = predict_whale_axis_position(&whale_filter.east, elapsed, WHALE_NOMINAL_EAST_VELOCITY);

    if (!isfinite(predicted_north) || !isfinite(predicted_east)) return 0;

    *measurement_time = whale_filter.measurement_time;
    *north = predicted_north;
    *east = predicted_east;

    return 1;
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
    const double elapsed = now - whale_filter.measurement_time;
    const double current_velocity_north = predict_whale_axis_velocity(&whale_filter.north, elapsed, WHALE_NOMINAL_NORTH_VELOCITY);
    const double current_velocity_east = predict_whale_axis_velocity(&whale_filter.east, elapsed, WHALE_NOMINAL_EAST_VELOCITY);
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