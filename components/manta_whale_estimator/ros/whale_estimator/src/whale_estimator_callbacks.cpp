#include "whale_estimator/whale_estimator_node.hpp"
#include "whale_estimator/whale_estimator_state_json.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

/*
 * Update one direction (north or east) of the whale estimate.
 * The motion model lets velocity gradually return toward its configured
 * baseline, while q adds uncertainty for unmodelled acceleration. The incoming
 * position has its own variance (R), so the filter weighs it against its
 * predicted position and updates both position and velocity uncertainty.
 */
int WhaleEstimatorNode::update_whale_axis() {
    whale_estimator_state_t *state = &state_;
    whale_estimate_one_axis_t *axis = &state->axis_update;

    // Select the measurement and baseline velocity for the axis being updated.
    const double measured_position = state->axis_update_is_north_b ? state->received_bearing_whale_position.north : state->received_bearing_whale_position.east;
    const double measurement_variance = state->axis_update_is_north_b
        ? state->received_bearing_whale_position.north_variance
        : state->received_bearing_whale_position.east_variance;
    const double nominal_velocity = state->axis_update_is_north_b ? state->config.nominal_north_velocity_mps : state->config.nominal_east_velocity_mps;
    const double dt = state->axis_update_elapsed_s;
    const double tau = state->config.velocity_return_time_s;
    const double q = state->config.acceleration_psd_m2_s3;

    // These values describe how much the velocity-return model changes over dt.
    const double u = dt / tau;
    const double decay = std::exp(-u);
    const double one_minus_decay = -std::expm1(-u);
    const double travel_time = tau * one_minus_decay;

    // This calculates how much the assumed random acceleration adds uncertainty during the time dt between position fixes. The filter tracks uncertainty in three connected quantities:
    // - noise_v: added uncertainty in velocity.
    // - noise_p: added uncertainty in position.
    // - noise_pv: added connection between position and velocity uncertainty. This matters because a position error can help correct the velocity estimate.
    // q controls how much random acceleration the model allows. tau controls how quickly velocity returns toward its baseline. travel_time accounts for that return while calculating the noise contributions
    const double noise_v = -0.5 * q * tau * std::expm1(-2.0 * u);
    const double noise_pv = 0.5 * q * travel_time * travel_time;
    double noise_p;
    if (u < 0.01) noise_p = q * dt * dt * dt * (1.0 / 3.0 + u * (-1.0 / 4.0 + u * (7.0 / 60.0 + u * (-1.0 / 24.0 + u * 31.0 / 2520.0))));
    else noise_p = q * tau * tau * (dt - 2.0 * travel_time - 0.5 * tau * std::expm1(-2.0 * u));

    // Predict the full position/velocity covariance. Position uncertainty and
    // position-velocity correlation now carry forward between measurements.
    const double predicted_p = axis->position_variance +
        2.0 * travel_time * axis->position_velocity_covariance +
        travel_time * travel_time * axis->velocity_variance + noise_p;
    const double predicted_pv = decay *
        (axis->position_velocity_covariance + travel_time * axis->velocity_variance) + noise_pv;
    const double predicted_v = decay * decay * axis->velocity_variance + noise_v;
    const double innovation_variance = predicted_p + measurement_variance;
    if (!std::isfinite(predicted_p) || predicted_p < 0.0 ||
        !std::isfinite(predicted_pv) || !std::isfinite(predicted_v) || predicted_v < 0.0 ||
        !std::isfinite(measurement_variance) || measurement_variance < 0.0 ||
        !std::isfinite(innovation_variance) || innovation_variance <= 0.0) return 0;

    // The innovation is the measured position minus the predicted position.
    // Measurement uncertainty reduces the correction: a noisier fix gets a
    // smaller gain, while velocity is corrected through its covariance with position.
    const double predicted_position = predict_whale_axis_position();
    const double predicted_velocity = predict_whale_axis_velocity();
    const double position_gain = predicted_p / innovation_variance;
    const double velocity_gain = predicted_pv / innovation_variance;
    const double residual = measured_position - predicted_position;
    const double updated_position = predicted_position + position_gain * residual;
    const double updated_velocity = predicted_velocity + velocity_gain * residual;
    double updated_position_variance = predicted_p * measurement_variance / innovation_variance;
    double updated_position_velocity_covariance = predicted_pv * measurement_variance / innovation_variance;
    double updated_velocity_variance = predicted_v - predicted_pv * predicted_pv / innovation_variance;
    if (!std::isfinite(updated_position) || !std::isfinite(updated_velocity) ||
        !std::isfinite(updated_position_variance) || !std::isfinite(updated_position_velocity_covariance) ||
        !std::isfinite(updated_velocity_variance) || updated_position_variance < 0.0 ||
        updated_velocity_variance < -1.0e-12 * std::fmax(1.0, predicted_v)) return 0;
    // Remove tiny negative variance caused by floating-point roundoff.
    updated_velocity_variance = std::fmax(0.0, updated_velocity_variance);

    // Store the corrected position and velocity together with their updated
    // variances and covariance for the next prediction step.
    axis->position = updated_position;
    axis->velocity = updated_velocity;
    axis->position_variance = updated_position_variance;
    axis->position_velocity_covariance = updated_position_velocity_covariance;
    axis->velocity_variance = updated_velocity_variance;
    return 1;
}

double WhaleEstimatorNode::predict_whale_axis_position() {
    const whale_estimator_state_t &state = state_;
    const whale_estimate_one_axis_t &axis = state.axis_update;
    const double nominal_velocity = state.axis_update_is_north_b
        ? state.config.nominal_north_velocity_mps : state.config.nominal_east_velocity_mps;
    const double elapsed = state.axis_update_elapsed_s;
    const double tau = state.config.velocity_return_time_s;
    const double travel_time = -tau * std::expm1(-elapsed / tau);
    return axis.position + nominal_velocity * elapsed + (axis.velocity - nominal_velocity) * travel_time;
}

double WhaleEstimatorNode::predict_whale_axis_velocity() {
    const whale_estimator_state_t &state = state_;
    const whale_estimate_one_axis_t &axis = state.axis_update;
    const double nominal_velocity = state.axis_update_is_north_b
        ? state.config.nominal_north_velocity_mps : state.config.nominal_east_velocity_mps;
    const double elapsed = state.axis_update_elapsed_s;
    const double tau = state.config.velocity_return_time_s;
    return nominal_velocity + (axis.velocity - nominal_velocity) * std::exp(-elapsed / tau);
}

namespace {

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
    const double north = estimator->received_bearing_whale_position.north;
    const double east = estimator->received_bearing_whale_position.east;
    const double time = estimator->received_bearing_whale_position.time;
    /* Compact the window after outages so stale fixes do not influence reacquisition. */
    for (i = 0; i < estimator->recent_fix_count; ++i) {
        if (time - estimator->recent_fixes[i].time <= estimator->config.velocity_window_max_age_s) {
            estimator->recent_fixes[count++] = estimator->recent_fixes[i];
        }
    }
    /* If still full, remove the oldest remaining fix before appending this one. */
    if (count == WHALE_VELOCITY_WINDOW_SIZE) {
        for (i = 1; i < count; ++i) estimator->recent_fixes[i - 1] = estimator->recent_fixes[i];
        --count;
    }
    estimator->recent_fixes[count++] = estimator->received_bearing_whale_position;
    estimator->recent_fix_count = count;
    /* Leave the Kalman/prior velocity in place until a fit has residual degrees of freedom. */
    if (count < 3) return;

    /* Center relative coordinates to preserve precision at large positions/times. */
    const whale_bearing_position_t origin = estimator->recent_fixes[count - 1];
    double mean_t = 0.0;
    double mean_position[2] = {0.0, 0.0};
    for (i = 0; i < count; ++i) {
        const whale_bearing_position_t *fix = &estimator->recent_fixes[i];
        mean_t += (fix->time - origin.time) / count;
        mean_position[0] += (fix->north - origin.north) / count;
        mean_position[1] += (fix->east - origin.east) / count;
    }
    /* Least-squares slope is sum(centered_time * centered_position) / sum(time^2). */
    double time_ss = 0.0;
    double cross[2] = {0.0, 0.0};
    for (i = 0; i < count; ++i) {
        const whale_bearing_position_t *fix = &estimator->recent_fixes[i];
        const double t = fix->time - origin.time - mean_t;
        time_ss += t * t;
        cross[0] += t * (fix->north - origin.north - mean_position[0]);
        cross[1] += t * (fix->east - origin.east - mean_position[1]);
    }
    if (!std::isfinite(time_ss) || time_ss <= 0.0) return;
    double velocity[2] = {cross[0] / time_ss, cross[1] / time_ss};
    /* Residuals describe how poorly a constant-velocity line fits this window. */
    double residual_ss[2] = {0.0, 0.0};
    for (i = 0; i < count; ++i) {
        const whale_bearing_position_t *fix = &estimator->recent_fixes[i];
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
        if (!std::isfinite(velocity[j]) || !std::isfinite(variance[j])) return;
    }
    estimator->north.velocity = velocity[0];
    estimator->east.velocity = velocity[1];
    // The rolling fit reuses the same position measurements, so do not treat
    // it as an independent Kalman observation or add another covariance update.
    estimator->north.position_velocity_covariance = 0.0;
    estimator->east.position_velocity_covariance = 0.0;
    estimator->north.velocity_variance = variance[0];
    estimator->east.velocity_variance = variance[1];
}

}

/*
 * Update the filter using the position fix and configuration stored in state_.
 * Returns 1 when the fix is accepted and the estimate state is updated, or 0
 * when the input is invalid, out of order, or the filter update is invalid.
 */
int WhaleEstimatorNode::update_estimator_state() {
    whale_estimator_state_t *state = &state_;

    // Clear the result first so every early return leaves this bearing position unaccepted.
    if (state == NULL) return 0;
    state->bearing_whale_position_accepted_b = 0;
    if (!state->estimator_configured_b) return 0;

    // Reject non-finite measurements and invalid tuning values before using them in the filter equations.
    if (!std::isfinite(state->received_bearing_whale_position.north) || !std::isfinite(state->received_bearing_whale_position.east) || !std::isfinite(state->received_bearing_whale_position.time) ||
        !std::isfinite(state->received_bearing_whale_position.north_variance) || state->received_bearing_whale_position.north_variance < 0.0 ||
        !std::isfinite(state->received_bearing_whale_position.east_variance) || state->received_bearing_whale_position.east_variance < 0.0) return 0;
    if (!std::isfinite(state->config.velocity_return_time_s) || state->config.velocity_return_time_s <= 0.0 ||
        !std::isfinite(state->config.acceleration_psd_m2_s3) || state->config.acceleration_psd_m2_s3 < 0.0 ||
        !std::isfinite(state->config.velocity_window_max_age_s) || state->config.velocity_window_max_age_s <= 0.0 ||
        !std::isfinite(state->config.initial_velocity_variance_m2ps2) || state->config.initial_velocity_variance_m2ps2 < 0.0 ||
        !std::isfinite(state->config.nominal_north_velocity_mps) || !std::isfinite(state->config.nominal_east_velocity_mps)) return 0;

    // The first bearing position anchors position; configured baseline velocities and initial velocity variance provide the starting velocity state.
    if (!state->estimator_initialized_b) {
        state->north = whale_estimate_one_axis_t{
            state->received_bearing_whale_position.north, state->config.nominal_north_velocity_mps,
            state->received_bearing_whale_position.north_variance, 0.0,
            state->config.initial_velocity_variance_m2ps2};
        state->east = whale_estimate_one_axis_t{
            state->received_bearing_whale_position.east, state->config.nominal_east_velocity_mps,
            state->received_bearing_whale_position.east_variance, 0.0,
            state->config.initial_velocity_variance_m2ps2};
        state->measurement_time_s = state->received_bearing_whale_position.time;
        state->estimator_initialized_b = 1;
    } else {
        whale_estimate_one_axis_t next_north, next_east;

        // Both bearing positions must advance simulated time. Keep each axis update in a temporary so a failed east or north update cannot partially commit.
        state->axis_update_elapsed_s = state->received_bearing_whale_position.time - state->measurement_time_s;
        if (!std::isfinite(state->axis_update_elapsed_s) || state->axis_update_elapsed_s <= 1.0e-6) return 0;

        // Run the same mean-reverting Kalman update for north and east using the selected axis and elapsed time stored in estimator state.
        state->axis_update_is_north_b = 1;
        state->axis_update = state->north;
        if (!update_whale_axis()) return 0;
        next_north = state->axis_update;

        state->axis_update_is_north_b = 0;
        state->axis_update = state->east;
        if (!update_whale_axis()) return 0;
        next_east = state->axis_update;

        state->north = next_north;
        state->east = next_east;
        state->measurement_time_s = state->received_bearing_whale_position.time;
    }

    // Refine velocity from the recent position window, then mark the input as
    // accepted so the callback can publish the resulting estimate.
    update_recent_whale_velocity(state);
    state->bearing_whale_position_accepted_b = 1;
    return 1;
}


//----------------------------------------------------
// Handle functions
//----------------------------------------------------

void WhaleEstimatorNode::handle_shutdown() {
    /* Stop the ROS node when the received shutdown message requests it. */
    if (!state_.received_shutdown_b) return;
    state_.shutdown_requested_b = 1;
    rclcpp::shutdown();
}

/*
 * Process the bearing-derived whale position currently stored in state_.
 * Reject fixes for another vehicle, update the filter for accepted fixes, and
 * publish the resulting Kalman estimate using the stored call cycle.
 */
void WhaleEstimatorNode::handle_whale_bearing_position() {
    if (state_.received_vehicle_id != vehicle_id_) return;
    const bool accepted = update_estimator_state();
    if (accepted) {
        whale_estimator_interfaces::msg::WhaleKalmanEstimate estimate;
        estimate.vehicle_id = vehicle_id_;
        estimate.call_cycle = state_.received_call_cycle;
        estimate.measurement_time_s = state_.measurement_time_s;
        estimate.whale_kalman_north_m = state_.north.position;
        estimate.whale_kalman_east_m = state_.east.position;
        estimate.whale_kalman_position_variance_north_m2 = state_.north.position_variance;
        estimate.whale_kalman_position_variance_east_m2 = state_.east.position_variance;
        estimate.whale_kalman_velocity_north_mps = state_.north.velocity;
        estimate.whale_kalman_velocity_east_mps = state_.east.velocity;
        estimate.whale_kalman_velocity_variance_north_m2ps2 = state_.north.velocity_variance;
        estimate.whale_kalman_velocity_variance_east_m2ps2 = state_.east.velocity_variance;
        estimate.whale_baseline_velocity_north_mps = state_.config.nominal_north_velocity_mps;
        estimate.whale_baseline_velocity_east_mps = state_.config.nominal_east_velocity_mps;
        estimate.whale_velocity_return_time_s = state_.config.velocity_return_time_s;
        estimate_pub_->publish(estimate);
    }
}
