"""ROS interfaces for one vehicle's whale estimator."""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from odyssey_interfaces.msg import WhaleBearingPositionEstimate
from std_msgs.msg import Bool
from whale_estimator_interfaces.msg import WhaleKalmanEstimate

from .whale_estimator_filter import update_estimator
from .whale_estimator_state import WhaleEstimatorState
from .whale_estimator_state_log import StateLog, state_snapshot
from .whale_estimator_types import BearingPositionMeasurement


class WhaleEstimatorNode(Node):
    def __init__(self):
        super().__init__("whale_estimator_node")
        self.state = WhaleEstimatorState()
        self._load_config()
        self.state_log = StateLog(f"{self.vehicle_id}_whale_estimator_state.jsonl")
        self._log_state_if_changed()
        self._create_ros_interfaces()
        self.estimator_ready_pub.publish(Bool(data=True))

    def _load_config(self) -> None:
        self.vehicle_id = self.declare_parameter("vehicle_id", "").value
        if not self.vehicle_id:
            raise ValueError("vehicle_id must be set.")

        parameters = {
            "velocity_window_max_age_s": "velocity_window_max_age_s",
            "q": "q",
            "velocity_return_time_s": "whale_velocity_return_time_s",
            "nominal_north_velocity_mps": "whale_baseline_velocity_north_mps",
            "nominal_east_velocity_mps": "whale_baseline_velocity_east_mps",
            "initial_velocity_variance_m2ps2": "initial_velocity_variance_m2ps2",
        }
        for field_name, parameter_name in parameters.items():
            value = float(self.declare_parameter(parameter_name, float("nan")).value)
            setattr(self.state.config, field_name, value)
        config = self.state.config
        if (not all(math.isfinite(value) for value in vars(config).values()) or config.velocity_window_max_age_s <= 0.0 or
                config.q < 0.0 or config.velocity_return_time_s <= 0.0 or
                config.initial_velocity_variance_m2ps2 < 0.0):
            raise ValueError("Invalid whale estimator configuration.")
        self.state.estimator_configured_b = True

    def _create_ros_interfaces(self) -> None:
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
        self.estimate_pub = self.create_publisher(WhaleKalmanEstimate, f"/{self.vehicle_id}/whale/kalman_estimate", qos)
        position_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.bearing_position_measurement_sub = self.create_subscription(WhaleBearingPositionEstimate, f"/{self.vehicle_id}/whale/bearing_position_estimate", self._handle_position, position_qos)
        shutdown_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.shutdown_sub = self.create_subscription(Bool, "/sim/shutdown", self._handle_shutdown, shutdown_qos)
        self.estimator_ready_pub = self.create_publisher(Bool, f"/{self.vehicle_id}/whale_estimator/ready", shutdown_qos)

    def _handle_position(self, message: WhaleBearingPositionEstimate) -> None:
        self.state.received_vehicle_id = message.vehicle_id
        self.state.received_call_cycle = message.call_cycle
        self.state.received_bearing_position_measurement = BearingPositionMeasurement(
            sim_time_s=message.sim_time_s,
            north_m=message.north_m,
            east_m=message.east_m,
            north_variance_m2=message.north_variance_m2,
            east_variance_m2=message.east_variance_m2,
        )
        if message.vehicle_id == self.vehicle_id:
            update_estimator(self.state)
            if self.state.bearing_whale_position_accepted_b:
                self._publish_estimate()
        self._log_state_if_changed()

    def _publish_estimate(self) -> None:
        state = self.state
        config = state.config
        estimate = WhaleKalmanEstimate()
        estimate.vehicle_id = self.vehicle_id
        estimate.call_cycle = state.received_call_cycle
        estimate.measurement_time_s = state.measurement_time_s
        estimate.whale_kalman_north_m = float(state.north_axis_estimate.x[0, 0])
        estimate.whale_kalman_east_m = float(state.east_axis_estimate.x[0, 0])
        estimate.whale_kalman_position_variance_north_m2 = float(state.north_axis_estimate.P[0, 0])
        estimate.whale_kalman_position_variance_east_m2 = float(state.east_axis_estimate.P[0, 0])
        estimate.whale_kalman_velocity_north_mps = float(state.north_axis_estimate.x[1, 0])
        estimate.whale_kalman_velocity_east_mps = float(state.east_axis_estimate.x[1, 0])
        estimate.whale_kalman_velocity_variance_north_m2ps2 = float(state.north_axis_estimate.P[1, 1])
        estimate.whale_kalman_velocity_variance_east_m2ps2 = float(state.east_axis_estimate.P[1, 1])
        estimate.whale_baseline_velocity_north_mps = config.nominal_north_velocity_mps
        estimate.whale_baseline_velocity_east_mps = config.nominal_east_velocity_mps
        estimate.whale_velocity_return_time_s = config.velocity_return_time_s
        self.estimate_pub.publish(estimate)

    def _handle_shutdown(self, message: Bool) -> None:
        self.state.received_shutdown_b = bool(message.data)
        if message.data:
            self.state.shutdown_requested_b = True
        self._log_state_if_changed()
        if message.data:
            rclpy.shutdown()

    def _log_state_if_changed(self) -> None:
        self.state_log.record(state_snapshot(self.state))

    def destroy_node(self) -> bool:
        self.state_log.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WhaleEstimatorNode()
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
