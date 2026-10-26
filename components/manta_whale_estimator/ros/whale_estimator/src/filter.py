"""Mean-reverting Kalman model and velocity fit from recent measurements."""

from copy import deepcopy
from math import exp, expm1, isfinite

import numpy as np
from filterpy.kalman import KalmanFilter

from .whale_estimator_state import WhaleEstimatorState
from .whale_estimator_types import VELOCITY_WINDOW_SIZE


def _new_axis(position: float, position_variance: float, baseline: float, velocity_variance: float) -> KalmanFilter:
    axis = KalmanFilter(dim_x=2, dim_z=1)
    axis.x = np.array([[position], [baseline]], dtype=float)
    axis.P = np.diag([position_variance, velocity_variance]).astype(float)
    axis.H = np.array([[1.0, 0.0]])
    axis.B = np.eye(2)
    return axis


def _process_noise(dt: float, tau: float, q: float, travel_time: float) -> np.ndarray:
    """Integrated acceleration noise for the mean-reverting velocity model."""
    u = dt / tau
    noise_v = -0.5 * q * tau * expm1(-2.0 * u)
    noise_pv = 0.5 * q * travel_time * travel_time
    if u < 0.01:
        # The direct expression subtracts nearly equal numbers for small dt.
        noise_p = q * dt**3 * (1.0 / 3.0 + u * (-1.0 / 4.0 + u * (7.0 / 60.0 + u * (-1.0 / 24.0 + u * 31.0 / 2520.0))))
    else:
        noise_p = q * tau**2 * (dt - 2.0 * travel_time - 0.5 * tau * expm1(-2.0 * u))
    return np.array([[noise_p, noise_pv], [noise_pv, noise_v]], dtype=float)


def _update_axis(axis: KalmanFilter, measurement: float, variance: float, baseline: float, dt: float, tau: float, q: float) -> KalmanFilter | None:
    """Predict one axis, then weight the new position by its variance R."""
    updated = deepcopy(axis)
    decay = exp(-dt / tau)
    travel_time = -tau * expm1(-dt / tau)
    updated.F = np.array([[1.0, travel_time], [0.0, decay]])
    updated.Q = _process_noise(dt, tau, q, travel_time)
    # F describes deviations from baseline velocity. This control term adds
    # the baseline motion back to both predicted position and velocity.
    control = np.array([[baseline * (dt - travel_time)], [baseline * (1.0 - decay)]])
    updated.predict(u=control)
    innovation_variance = float(updated.P[0, 0]) + variance
    if (not np.isfinite(updated.x).all() or not np.isfinite(updated.P).all() or not np.isfinite(updated.Q).all() or not isfinite(innovation_variance) or
            updated.P[0, 0] < 0.0 or updated.P[1, 1] < 0.0 or innovation_variance <= 0.0):
        return None
    updated.update(measurement, R=variance)
    updated.P = 0.5 * (updated.P + updated.P.T)
    if (not np.isfinite(updated.x).all() or not np.isfinite(updated.P).all() or updated.P[0, 0] < -1.0e-12 or updated.P[1, 1] < -1.0e-12):
        return None
    updated.P[0, 0] = max(0.0, updated.P[0, 0])
    updated.P[1, 1] = max(0.0, updated.P[1, 1])
    return updated


def _fit_recent_velocity(state: WhaleEstimatorState) -> None:
    """Fit velocity from recent position measurements after at least three arrive."""
    current = state.received_bearing_position_measurement
    window = [measurement for measurement in state.recent_bearing_position_measurements
              if current.sim_time_s - measurement.sim_time_s <= state.config.velocity_window_max_age_s]
    state.recent_bearing_position_measurements = (window + [current])[-VELOCITY_WINDOW_SIZE:]
    if len(state.recent_bearing_position_measurements) < 3:
        return

    origin = state.recent_bearing_position_measurements[-1]
    count = len(state.recent_bearing_position_measurements)
    mean_t = sum((measurement.sim_time_s - origin.sim_time_s) / count for measurement in state.recent_bearing_position_measurements)
    means = [sum((getattr(measurement, direction) - getattr(origin, direction)) / count for measurement in state.recent_bearing_position_measurements) for direction in ("north_m", "east_m")]
    centered_times = [measurement.sim_time_s - origin.sim_time_s - mean_t for measurement in state.recent_bearing_position_measurements]
    time_ss = sum(t * t for t in centered_times)
    if not isfinite(time_ss) or time_ss <= 0.0:
        return

    velocities = []
    variances = []
    span = current.sim_time_s - state.recent_bearing_position_measurements[0].sim_time_s
    for direction, mean in zip(("north_m", "east_m"), means):
        positions = [getattr(measurement, direction) - getattr(origin, direction) - mean for measurement in state.recent_bearing_position_measurements]
        velocity = sum(t * p for t, p in zip(centered_times, positions)) / time_ss
        residual_ss = sum((p - velocity * t) ** 2 for t, p in zip(centered_times, positions))
        variance = (residual_ss / (count - 2) / time_ss + state.config.q * span / 3.0)
        if not isfinite(velocity) or not isfinite(variance):
            return
        velocities.append(velocity)
        variances.append(variance)

    # The fit reuses the same measurements. It replaces velocity, rather than being
    # fused as an independent Kalman observation.
    for axis, velocity, variance in zip((state.north_axis_estimate, state.east_axis_estimate), velocities, variances):
        axis.x[1, 0] = velocity
        axis.P[0, 1] = axis.P[1, 0] = 0.0
        axis.P[1, 1] = variance


def update_estimator(state: WhaleEstimatorState) -> bool:
    """Consume the stored bearing position and update both axes atomically."""
    state.bearing_whale_position_accepted_b = False

    if not state.estimator_configured_b:
        return False
    
    measurement = state.received_bearing_position_measurement

    values = (measurement.sim_time_s, measurement.north_m, measurement.east_m, measurement.north_variance_m2, measurement.east_variance_m2)
    if (not all(isfinite(value) for value in values) or min(measurement.north_variance_m2, measurement.east_variance_m2) < 0.0):
        return False
    
    config = state.config
    if not state.estimator_initialized_b:
        state.north_axis_estimate = _new_axis(measurement.north_m, measurement.north_variance_m2, config.nominal_north_velocity_mps, config.initial_velocity_variance_m2ps2)
        state.east_axis_estimate = _new_axis(measurement.east_m, measurement.east_variance_m2, config.nominal_east_velocity_mps, config.initial_velocity_variance_m2ps2)
        state.estimator_initialized_b = True
    else:
        dt = measurement.sim_time_s - state.measurement_time_s
        if not isfinite(dt) or dt <= 1.0e-6:
            return False
        
        north = _update_axis(state.north_axis_estimate, measurement.north_m, measurement.north_variance_m2, config.nominal_north_velocity_mps, dt, config.velocity_return_time_s, config.q)
        east = _update_axis(state.east_axis_estimate, measurement.east_m, measurement.east_variance_m2, config.nominal_east_velocity_mps, dt, config.velocity_return_time_s, config.q)
        if north is None or east is None:
            return False
        state.north_axis_estimate, state.east_axis_estimate = north, east

    state.measurement_time_s = measurement.sim_time_s
    _fit_recent_velocity(state)
    state.bearing_whale_position_accepted_b = True
    return True
