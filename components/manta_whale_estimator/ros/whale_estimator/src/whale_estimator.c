#include "whale_estimator.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/* Positions are local north/east metres and times are simulated seconds.
 * Exact position fixes (R = 0) update a mean-reverting Kalman velocity.
 * A rolling fit replaces velocity after three recent fixes are available.
 */

/*
 * Integrate v(t) = nominal + (initial - nominal) * exp(-t / tau).
 * Integrating the changing velocity avoids a jump in position as velocity
 * returns to nominal. expm1 preserves precision for very small elapsed times.
 */
static double predict_whale_axis_position(const whale_axis_filter_t *axis, double elapsed, double nominal_velocity, double tau) {
    const double travel_time = -tau * expm1(-elapsed / tau);
    return axis->position + nominal_velocity * elapsed + (axis->velocity - nominal_velocity) * travel_time;
}

/* Query the model velocity without advancing the stored measurement state. */
static double predict_whale_axis_velocity(const whale_axis_filter_t *axis, double elapsed, double nominal_velocity, double tau) {
    return nominal_velocity + (axis->velocity - nominal_velocity) * exp(-elapsed / tau);
}

/*
 * Exact continuous-time mean-reverting prediction and an exact-position update
 * (R = 0). Position covariance and position/velocity covariance vanish after
 * each fix, so only posterior velocity variance is retained between fixes.
 */
static int update_whale_axis(whale_axis_filter_t *axis, double measured_position, double dt, double nominal_velocity, const whale_estimator_config_t *config) {
    const double tau = config->velocity_return_time_s;
    const double u = dt / tau;
    const double decay = exp(-u);
    const double one_minus_decay = -expm1(-u);
    const double travel_time = tau * one_minus_decay;
    const double q = config->acceleration_psd_m2_s3;

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
    const double predicted_position = predict_whale_axis_position(axis, dt, nominal_velocity, tau);
    const double predicted_velocity = predict_whale_axis_velocity(axis, dt, nominal_velocity, tau);
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
static void update_recent_whale_velocity(whale_estimator_state_t *estimator) {
    int i, j;
    int count = 0;
    const double north = estimator->received_fix.north;
    const double east = estimator->received_fix.east;
    const double time = estimator->received_fix.time;
    /* Compact the window after outages so stale fixes do not influence reacquisition. */
    for (i = 0; i < estimator->recent_fix_count; ++i) {
        if (time - estimator->recent_fixes[i].time <= estimator->config.velocity_window_max_age_s) {
            estimator->recent_fixes[count++] = estimator->recent_fixes[i];
        }
    }
    /* If still full, remove the oldest remaining fix before appending this one. */
    if (count == WHALE_VELOCITY_WINDOW_SIZE) {
        for (i = 1; i < count; ++i) {
            estimator->recent_fixes[i - 1] = estimator->recent_fixes[i];
        }
        --count;
    }
    estimator->recent_fixes[count++] = (whale_position_fix_t){time, north, east};
    estimator->recent_fix_count = count;
    /* Leave the Kalman/prior velocity in place until a fit has residual degrees of freedom. */
    if (count < 3) return;

    /* Center relative coordinates to preserve precision at large positions/times. */
    const whale_position_fix_t origin = estimator->recent_fixes[count - 1];
    double mean_t = 0.0;
    double mean_position[2] = {0.0, 0.0};
    for (i = 0; i < count; ++i) {
        const whale_position_fix_t *fix = &estimator->recent_fixes[i];
        mean_t += (fix->time - origin.time) / count;
        mean_position[0] += (fix->north - origin.north) / count;
        mean_position[1] += (fix->east - origin.east) / count;
    }
    /* Least-squares slope is sum(centered_time * centered_position) / sum(time^2). */
    double time_ss = 0.0;
    double cross[2] = {0.0, 0.0};
    for (i = 0; i < count; ++i) {
        const whale_position_fix_t *fix = &estimator->recent_fixes[i];
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
        const whale_position_fix_t *fix = &estimator->recent_fixes[i];
        const double t = fix->time - origin.time - mean_t;
        const double residual[2] = {
            fix->north - origin.north - mean_position[0] - velocity[0] * t,
            fix->east - origin.east - mean_position[1] - velocity[1] * t
        };
        for (j = 0; j < 2; ++j) residual_ss[j] += residual[j] * residual[j];
    }
    const double span = time - estimator->recent_fixes[0].time;
    double variance[2];
    for (j = 0; j < 2; ++j) {
        /* Residual slope uncertainty plus a process-noise allowance for turns.
         * This is a rolling-fit heuristic, not an independent Kalman observation.
         */
        variance[j] = residual_ss[j] / (count - 2) / time_ss + estimator->config.acceleration_psd_m2_s3 * span / 3.0;
        if (!isfinite(velocity[j]) || !isfinite(variance[j])) return;
    }
    estimator->north.velocity = velocity[0];
    estimator->east.velocity = velocity[1];
    estimator->north.velocity_variance = variance[0];
    estimator->east.velocity_variance = variance[1];
}

void whale_estimator_init(whale_estimator_state_t *state) {
    if (state == NULL) return;
    memset(state, 0, sizeof(*state));
    state->config.velocity_window_max_age_s = WHALE_VELOCITY_WINDOW_MAX_AGE_S;
    state->config.acceleration_psd_m2_s3 = WHALE_ACCELERATION_PSD;
    state->config.velocity_return_time_s = WHALE_VELOCITY_RETURN_TIME_S;
    state->config.nominal_north_velocity_mps = WHALE_NOMINAL_NORTH_VELOCITY;
    state->config.nominal_east_velocity_mps = WHALE_NOMINAL_EAST_VELOCITY;
    state->config.initial_velocity_variance_m2ps2 = WHALE_INITIAL_VELOCITY_VARIANCE;
    state->configured_b = 1;
}

/* Accept the latest triangulated fix stored in state. Repeated or older
 * timestamps leave the filter and recent-fix window unchanged.
 */
int whale_estimator_update(whale_estimator_state_t *state) {
    double north, east, measurement_time;
    if (state == NULL) return 0;
    state->fix_accepted_b = 0;
    if (!state->configured_b) return 0;
    north = state->received_fix.north;
    east = state->received_fix.east;
    measurement_time = state->received_fix.time;
    if (!isfinite(north) || !isfinite(east) || !isfinite(measurement_time)) return 0;
    if (!isfinite(state->config.velocity_return_time_s) || state->config.velocity_return_time_s <= 0.0 ||
        !isfinite(state->config.acceleration_psd_m2_s3) || state->config.acceleration_psd_m2_s3 < 0.0 ||
        !isfinite(state->config.velocity_window_max_age_s) || state->config.velocity_window_max_age_s <= 0.0 ||
        !isfinite(state->config.initial_velocity_variance_m2ps2) || state->config.initial_velocity_variance_m2ps2 < 0.0 ||
        !isfinite(state->config.nominal_north_velocity_mps) || !isfinite(state->config.nominal_east_velocity_mps)) return 0;

    if (!state->initialized_b) {
        state->north = (whale_axis_filter_t){north, state->config.nominal_north_velocity_mps, state->config.initial_velocity_variance_m2ps2};
        state->east = (whale_axis_filter_t){east, state->config.nominal_east_velocity_mps, state->config.initial_velocity_variance_m2ps2};
        state->measurement_time_s = measurement_time;
        state->initialized_b = 1;
    } else {
        const double dt = measurement_time - state->measurement_time_s;
        whale_axis_filter_t next_north, next_east;
        if (!isfinite(dt) || dt <= 1.0e-6) return 0;
        next_north = state->north;
        next_east = state->east;
        if (!update_whale_axis(&next_north, north, dt, state->config.nominal_north_velocity_mps, &state->config) ||
            !update_whale_axis(&next_east, east, dt, state->config.nominal_east_velocity_mps, &state->config)) return 0;
        state->north = next_north;
        state->east = next_east;
        state->measurement_time_s = measurement_time;
    }

    update_recent_whale_velocity(state);
    state->fix_accepted_b = 1;
    return 1;
}

/* Predict from the last accepted fix without advancing the filter. */
int whale_estimator_predict(whale_estimator_state_t *state) {
    double elapsed, predicted_north, predicted_east, velocity_north, velocity_east;
    if (state == NULL) return 0;
    state->prediction = (whale_prediction_t){0};
    if (!state->configured_b || !state->initialized_b || !isfinite(state->prediction_time_s) ||
        !isfinite(state->config.velocity_return_time_s) || state->config.velocity_return_time_s <= 0.0) return 0;
    elapsed = state->prediction_time_s - state->measurement_time_s;
    if (!isfinite(elapsed) || elapsed < 0.0) return 0;
    predicted_north = predict_whale_axis_position(&state->north, elapsed, state->config.nominal_north_velocity_mps, state->config.velocity_return_time_s);
    predicted_east = predict_whale_axis_position(&state->east, elapsed, state->config.nominal_east_velocity_mps, state->config.velocity_return_time_s);
    velocity_north = predict_whale_axis_velocity(&state->north, elapsed, state->config.nominal_north_velocity_mps, state->config.velocity_return_time_s);
    velocity_east = predict_whale_axis_velocity(&state->east, elapsed, state->config.nominal_east_velocity_mps, state->config.velocity_return_time_s);
    if (!isfinite(predicted_north) || !isfinite(predicted_east) || !isfinite(velocity_north) || !isfinite(velocity_east)) return 0;

    state->prediction = (whale_prediction_t){1, state->measurement_time_s, predicted_north, predicted_east, velocity_north, velocity_east};
    return 1;
}

