"""Build state snapshots and write them when estimator state changes."""

import json
import math
import os
from pathlib import Path

from .whale_estimator_state import WhaleEstimatorState


def axis_snapshot(axis) -> dict[str, float]:
    """Convert a FilterPy axis into plain values for JSON serialization."""
    if axis is None:
        return dict.fromkeys(
            ("position_m", "velocity_mps", "position_variance_m2",
             "position_velocity_covariance_m2ps", "velocity_variance_m2ps2"), 0.0
        )
    return {
        "position_m": float(axis.x[0, 0]),
        "velocity_mps": float(axis.x[1, 0]),
        "position_variance_m2": float(axis.P[0, 0]),
        "position_velocity_covariance_m2ps": float(axis.P[0, 1]),
        "velocity_variance_m2ps2": float(axis.P[1, 1]),
    }


def state_snapshot(state: WhaleEstimatorState) -> dict:
    """Return the complete persistent state for change-only JSON logging."""
    return {
        "config": vars(state.config).copy(),
        "estimator_configured_b": state.estimator_configured_b,
        "shutdown_requested_b": state.shutdown_requested_b,
        "received_shutdown_b": state.received_shutdown_b,
        "estimator_initialized_b": state.estimator_initialized_b,
        "measurement_time_s": state.measurement_time_s,
        "north_axis_estimate": axis_snapshot(state.north_axis_estimate),
        "east_axis_estimate": axis_snapshot(state.east_axis_estimate),
        "recent_bearing_position_measurements": [vars(measurement).copy() for measurement in state.recent_bearing_position_measurements],
        "recent_bearing_position_measurement_count": len(state.recent_bearing_position_measurements),
        "received_bearing_position_measurement": vars(state.received_bearing_position_measurement).copy(),
        "received_vehicle_id": state.received_vehicle_id,
        "received_call_cycle": state.received_call_cycle,
        "bearing_whale_position_accepted_b": state.bearing_whale_position_accepted_b,
        "axis_update_is_north_b": state.axis_update_is_north_b,
        "axis_update_filter": axis_snapshot(state.axis_update_filter),
        "axis_update_position_m": state.axis_update_position_m,
        "axis_update_position_variance_m2": state.axis_update_position_variance_m2,
        "axis_update_baseline_velocity_mps": state.axis_update_baseline_velocity_mps,
        "axis_update_velocity_variance_m2ps2": state.axis_update_velocity_variance_m2ps2,
        "axis_update_elapsed_s": state.axis_update_elapsed_s,
        "axis_update_velocity_return_time_s": state.axis_update_velocity_return_time_s,
        "axis_update_q_m2_s3": state.axis_update_q_m2_s3,
        "axis_update_travel_time_s": state.axis_update_travel_time_s,
        "axis_update_process_noise": state.axis_update_process_noise.tolist()
        if state.axis_update_process_noise is not None else None,
        "axis_update_succeeded_b": state.axis_update_succeeded_b,
        "pending_north_axis_estimate": axis_snapshot(state.pending_north_axis_estimate),
        "pending_east_axis_estimate": axis_snapshot(state.pending_east_axis_estimate),
    }


def _json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return "NaN" if math.isnan(value) else ("Infinity" if value > 0 else "-Infinity")
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_json_safe(item) for item in value]
    return value


class StateLog:
    def __init__(self, filename: str):
        directory = Path(os.environ.get("ROS_LOG_DIR") or "/odyssey/output/Logs")
        directory.mkdir(parents=True, exist_ok=True)
        self._stream = (directory / filename).open("w", encoding="utf-8")
        self._previous = None

    def record(self, snapshot: dict) -> None:
        encoded = json.dumps(_json_safe(snapshot), separators=(",", ":"), allow_nan=False)
        if encoded != self._previous:
            self._stream.write(encoded + "\n")
            self._stream.flush()
            self._previous = encoded

    def close(self) -> None:
        self._stream.close()
