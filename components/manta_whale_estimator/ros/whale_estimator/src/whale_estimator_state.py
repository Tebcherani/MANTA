"""Filter history owned by one estimator node, with units documented per field."""

from dataclasses import dataclass, field
from typing import Any

from .whale_estimator_types import BearingPositionMeasurement, EstimatorConfig

@dataclass
class WhaleEstimatorState:
    config: EstimatorConfig = field(default_factory=EstimatorConfig)  # Config values carry units in their field names.
    estimator_configured_b: bool = False  # Boolean flag; no physical unit.
    shutdown_requested_b: bool = False  # Boolean flag; no physical unit.
    received_shutdown_b: bool = False  # Boolean flag; no physical unit.
    estimator_initialized_b: bool = False  # Boolean flag; no physical unit.
    measurement_time_s: float = 0.0  # Simulation time in seconds.
    north_axis_estimate: Any = None  # North-axis [position_m, velocity_mps]; covariance units m², m²/s, and m²/s².
    east_axis_estimate: Any = None  # East-axis [position_m, velocity_mps]; covariance units m², m²/s, and m²/s².
    recent_bearing_position_measurements: list[BearingPositionMeasurement] = field(default_factory=list)  # Count; entries contain seconds, metres, and m².
    received_bearing_position_measurement: BearingPositionMeasurement = field(default_factory=BearingPositionMeasurement)  # Contains seconds, metres, and m².
    received_vehicle_id: str = ""  # Identifier; no physical unit.
    received_call_cycle: int = 0  # Cycle index; count, not a physical unit.
    bearing_whale_position_accepted_b: bool = False  # Boolean flag; no physical unit.

    # Working values used while updating one axis. The axis flag selects north
    # or east; all numeric fields use the units in their names.
    axis_update_is_north_b: bool = False  # Boolean flag; no physical unit.
    axis_update_filter: Any = None  # Current one-axis Kalman filter; see covariance units above.
    axis_update_position_m: float = 0.0  # Measured position in metres.
    axis_update_position_variance_m2: float = 0.0  # Measurement variance in square metres.
    axis_update_baseline_velocity_mps: float = 0.0  # Baseline velocity in metres per second.
    axis_update_velocity_variance_m2ps2: float = 0.0  # Initial velocity variance in (metres/second)².
    axis_update_elapsed_s: float = 0.0  # Time since the previous measurement in seconds.
    axis_update_velocity_return_time_s: float = 0.0  # Duration for the starting velocity difference to return to baseline, in seconds.
    axis_update_q_m2_s3: float = 0.0  # Acceleration-noise intensity in m²/s³.
    axis_update_travel_time_s: float = 0.0  # Effective travel time in seconds.
    axis_update_process_noise: Any = None  # Process-noise covariance matrix; units follow position/velocity covariance.
    axis_update_succeeded_b: bool = False  # Boolean flag; no physical unit.

    # Temporary results are kept here so both axes can be accepted together.
    pending_north_axis_estimate: Any = None  # Candidate north-axis filter.
    pending_east_axis_estimate: Any = None  # Candidate east-axis filter.
