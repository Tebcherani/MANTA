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
