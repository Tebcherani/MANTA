"""Reusable configuration and measurement types for the whale estimator."""

from dataclasses import dataclass


VELOCITY_WINDOW_SIZE = 5


@dataclass
class EstimatorConfig:
    """Tunable motion-model and filter settings loaded from ROS parameters."""

    velocity_window_max_age_s: float = 0.0
    q: float = 0.0  # Continuous-time acceleration-noise power spectral density, in m²/s³.
    velocity_return_time_s: float = 0.0
    nominal_north_velocity_mps: float = 0.0
    nominal_east_velocity_mps: float = 0.0
    initial_velocity_variance_m2ps2: float = 0.0


@dataclass
class BearingPositionMeasurement:
    """A measured whale position and its north/east measurement variances."""

    sim_time_s: float = 0.0
    north_m: float = 0.0
    east_m: float = 0.0
    north_variance_m2: float = 0.0
    east_variance_m2: float = 0.0
