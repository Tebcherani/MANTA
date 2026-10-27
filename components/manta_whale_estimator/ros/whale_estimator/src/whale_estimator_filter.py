"""Constant-rate return Kalman model and recent-position velocity fit."""

from copy import deepcopy
from math import isfinite

import numpy as np
from filterpy.kalman import KalmanFilter

from .whale_estimator_state import WhaleEstimatorState
from .whale_estimator_types import VELOCITY_WINDOW_SIZE


def _new_axis(state: WhaleEstimatorState) -> None:
    """Create the initial filter for the axis selected in state."""

    axis = KalmanFilter(dim_x=2, dim_z=1)
    axis.x = np.array([[state.axis_update_position_m], [state.axis_update_baseline_velocity_mps]], dtype=float)
    axis.P = np.diag([state.axis_update_position_variance_m2, state.axis_update_velocity_variance_m2ps2]).astype(float)
    axis.H = np.array([[1.0, 0.0]]) # Tells the filter what a measurement observes (only position).
    axis.B = np.eye(2)              # Determines how the control is applied (first input affects position, second affects velocity).
    state.axis_update_filter = axis


def _prepare_axis_update(state: WhaleEstimatorState) -> None:
    """Load inputs for the selected north or east axis into state fields."""
    measurement = state.received_bearing_position_measurement

    if state.axis_update_is_north_b:
        state.axis_update_position_m = measurement.north_m
        state.axis_update_position_variance_m2 = measurement.north_variance_m2
        state.axis_update_baseline_velocity_mps = (state.config.nominal_north_velocity_mps)
        state.axis_update_filter = state.north_axis_estimate
    else:
        state.axis_update_position_m = measurement.east_m
        state.axis_update_position_variance_m2 = measurement.east_variance_m2
        state.axis_update_baseline_velocity_mps = (state.config.nominal_east_velocity_mps)
        state.axis_update_filter = state.east_axis_estimate

    state.axis_update_velocity_return_time_s = state.config.velocity_return_time_s
    state.axis_update_q_m2_s3 = state.config.q
    state.axis_update_elapsed_s = measurement.sim_time_s - state.measurement_time_s
    state.axis_update_velocity_variance_m2ps2 = (state.config.initial_velocity_variance_m2ps2)


def _process_noise(state: WhaleEstimatorState) -> None:
    """Calculate white-acceleration noise over the current time step."""

    dt_s = state.axis_update_elapsed_s
    q = state.axis_update_q_m2_s3

    # Integrating white acceleration noise gives position variance, the position/velocity covariance, and velocity variance for this step.
    noise_p = q * dt_s**3 / 3.0
    noise_pv = q * dt_s**2 / 2.0
    noise_v = q * dt_s

    state.axis_update_process_noise = np.array([[noise_p, noise_pv], [noise_pv, noise_v]], dtype=float)


def _update_axis(state: WhaleEstimatorState) -> None:
    """Predict one axis, then weight the new position by its variance R."""
    state.axis_update_succeeded_b = False
    updated = deepcopy(state.axis_update_filter)
    dt = state.axis_update_elapsed_s
    velocity_return_time = state.axis_update_velocity_return_time_s
    baseline = state.axis_update_baseline_velocity_mps
    measurement = state.axis_update_position_m
    variance = state.axis_update_position_variance_m2

    # Remove the starting velocity deviation at a constant rate. If the full return time elapses, the deviation is gone and velocity stays at baseline.
    if dt < velocity_return_time:
        T = dt - dt**2 / (2.0 * velocity_return_time)
    else:
        T = velocity_return_time / 2.0
    state.axis_update_travel_time_s = T

    a = max(0.0, 1.0 - dt / velocity_return_time)

    updated.F = np.array([[1.0, T], [0.0, a]])
    
    _process_noise(state)
    updated.Q = state.axis_update_process_noise

    u = np.array([[baseline * (dt - T)], [baseline * (1.0 - a)]])
    updated.predict(u=u)

    # The predicted position uncertainty plus measurement uncertainty is the variance of their difference (the innovation).
    innovation_variance = float(updated.P[0, 0]) + variance
    if (not np.isfinite(updated.x).all() or not np.isfinite(updated.P).all() or not np.isfinite(updated.Q).all() or not isfinite(innovation_variance)
        or updated.P[0, 0] < 0.0 or updated.P[1, 1] < 0.0 or innovation_variance <= 0.0
    ):
        return

    # FilterPy computes the Kalman gain from P and R=variance. A more precise measurement (smaller R) pulls the estimate closer to the measurement.
    updated.update(measurement, R=variance)
    updated.P = 0.5 * (updated.P + updated.P.T) # Roundoff can make the covariance matrix slightly asymmetric or negative.
    if (not np.isfinite(updated.x).all() or not np.isfinite(updated.P).all() or updated.P[0, 0] < -1.0e-12 or updated.P[1, 1] < -1.0e-12):
        return
    updated.P[0, 0] = max(0.0, updated.P[0, 0])
    updated.P[1, 1] = max(0.0, updated.P[1, 1])

    state.axis_update_filter = updated
    state.axis_update_succeeded_b = True


def _fit_recent_velocity(state: WhaleEstimatorState) -> None:
    """Fit velocity from recent position measurements after at least three arrive."""
    current = state.received_bearing_position_measurement

    # Keep only measurements in the configured time window, with a fixed cap so old positions cannot dominate the current velocity estimate.
    window = [
        measurement for measurement in state.recent_bearing_position_measurements
        if current.sim_time_s - measurement.sim_time_s <= state.config.velocity_window_max_age_s
    ]
    state.recent_bearing_position_measurements = (window + [current])[-VELOCITY_WINDOW_SIZE:]
    if len(state.recent_bearing_position_measurements) < 3:
        return

    # Shift time and position near zero before fitting; this also avoids precision loss when simulated timestamps become large.
    origin = state.recent_bearing_position_measurements[-1]
    count = len(state.recent_bearing_position_measurements)
    measurements = state.recent_bearing_position_measurements

    mean_t = sum((measurement.sim_time_s - origin.sim_time_s) / count for measurement in measurements)
    means = [
        sum((getattr(measurement, direction) - getattr(origin, direction)) / count for measurement in measurements)
        for direction in ("north_m", "east_m")
    ]
    centered_times = [measurement.sim_time_s - origin.sim_time_s - mean_t for measurement in measurements]
    time_ss = sum(t * t for t in centered_times)
    if not isfinite(time_ss) or time_ss <= 0.0:
        return

    # The slope of the best-fit line is velocity. Residuals measure how much
    # the positions depart from that line and contribute to its uncertainty.
    velocities = []
    variances = []
    span = current.sim_time_s - measurements[0].sim_time_s
    for direction, mean in zip(("north_m", "east_m"), means):
        positions = [getattr(measurement, direction) - getattr(origin, direction) - mean for measurement in measurements]
        velocity = sum(t * p for t, p in zip(centered_times, positions)) / time_ss
        residual_ss = sum((p - velocity * t) ** 2 for t, p in zip(centered_times, positions))
        variance = residual_ss / (count - 2) / time_ss + state.config.q * span / 3.0
        if not isfinite(velocity) or not isfinite(variance):
            return
        velocities.append(velocity)
        variances.append(variance)

    # The fit reuses the same measurements. It replaces velocity, rather than being fused as an independent Kalman observation.
    for axis, velocity, variance in zip((state.north_axis_estimate, state.east_axis_estimate), velocities, variances):
        axis.x[1, 0] = velocity
        axis.P[0, 1] = axis.P[1, 0] = 0.0
        axis.P[1, 1] = variance


def update_estimator(state: WhaleEstimatorState) -> None:
    """Use one stored position measurement to update north and east together."""

    state.bearing_whale_position_accepted_b = False
    if not state.estimator_configured_b:
        return

    measurement = state.received_bearing_position_measurement
    values = (
        measurement.sim_time_s,
        measurement.north_m,
        measurement.east_m,
        measurement.north_variance_m2,
        measurement.east_variance_m2,
    )
    if (not all(isfinite(value) for value in values) or min(measurement.north_variance_m2, measurement.east_variance_m2) < 0.0):
        return

    if not state.estimator_initialized_b:
        # Initialize both candidate filters before installing either one.
        state.axis_update_is_north_b = True
        _prepare_axis_update(state)
        _new_axis(state)
        state.pending_north_axis_estimate = state.axis_update_filter

        state.axis_update_is_north_b = False
        _prepare_axis_update(state)
        _new_axis(state)
        state.pending_east_axis_estimate = state.axis_update_filter

        state.north_axis_estimate = state.pending_north_axis_estimate
        state.east_axis_estimate = state.pending_east_axis_estimate
        state.estimator_initialized_b = True
    else:
        state.axis_update_is_north_b = True
        _prepare_axis_update(state)
        if (not isfinite(state.axis_update_elapsed_s) or state.axis_update_elapsed_s <= 1.0e-6):
            return

        _update_axis(state)
        if not state.axis_update_succeeded_b:
            return
        state.pending_north_axis_estimate = state.axis_update_filter

        state.axis_update_is_north_b = False
        _prepare_axis_update(state)
        if (not isfinite(state.axis_update_elapsed_s) or state.axis_update_elapsed_s <= 1.0e-6):
            return
        _update_axis(state)
        if not state.axis_update_succeeded_b:
            return
        state.pending_east_axis_estimate = state.axis_update_filter

        state.north_axis_estimate = state.pending_north_axis_estimate
        state.east_axis_estimate = state.pending_east_axis_estimate

    state.measurement_time_s = measurement.sim_time_s
    _fit_recent_velocity(state)
    state.bearing_whale_position_accepted_b = True
