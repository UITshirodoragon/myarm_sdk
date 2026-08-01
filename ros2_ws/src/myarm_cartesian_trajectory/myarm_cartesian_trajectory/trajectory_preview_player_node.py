"""Replay a planned joint trajectory to a preview-only ``JointState`` topic.

This node intentionally has no robot adapter or driver publisher.  Its only
output is synthetic state for visualisation.  It uses the same interpolation
kernel as the planner and motion-execution adapter, so RViz preview reflects
the continuous trajectory rather than jumping between submitted knots.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Any, Mapping, Optional, Tuple

import rclpy
from myarm_sdk.core import (
    JointPositions,
    JointTrajectory as SdkJointTrajectory,
    TrajectoryPoint,
    load_sdk_yaml,
)
from myarm_sdk.core.joint_trajectory_interpolation import sample_joint_trajectory
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory


@dataclass(frozen=True)
class _PreviewTrajectory:
    """A validated immutable replay buffer, intentionally separate from robot I/O."""

    trajectory: SdkJointTrajectory

    @property
    def joint_names(self) -> Tuple[str, ...]:
        """Return only the six planner-owned arm joint names."""
        return self.trajectory.joint_names

    @property
    def duration_s(self) -> float:
        """Return the exact final time of the planned trajectory."""
        return self.trajectory.duration_s


class MyArmTrajectoryPreviewPlayerNode(Node):
    """Publish only synthetic JointState values for a Cartesian-plan preview.

    The output defaults to a private topic so it cannot race the real driver on
    ``/joint_states``.  A preview-only launch may explicitly remap the output
    into a visualization-only robot_state_publisher setup.

    Incoming plans always retain their six arm joints.  The synthetic output
    additionally includes a closed ``left_gripper_joint`` so the baseline URDF
    remains complete in RViz without making a gripper command.
    """

    _SERVICES_CONFIG = "service/config/services.yaml"
    _CLOSED_GRIPPER_POSITION_RAD = 0.0

    def __init__(self) -> None:
        super().__init__("myarm_cartesian_trajectory_preview")
        self.declare_parameter("services_config", self._SERVICES_CONFIG)
        self.declare_parameter("service_name", "cartesian_trajectory_planner")
        # Empty means inherit services.<service_name>.topics.joint_preview.
        self.declare_parameter(
            "joint_preview_topic", ""
        )
        self.declare_parameter(
            "output_joint_states_topic",
            "/myarm/cartesian_trajectory/preview_joint_states",
        )
        self.declare_parameter("playback_rate_hz", 5.0)
        self.declare_parameter("loop", False)

        input_topic = self._joint_preview_topic_from_config_or_parameter()
        output_topic = self._topic_parameter("output_joint_states_topic")
        playback_rate_hz = self._positive_float(
            self.get_parameter("playback_rate_hz").value, "playback_rate_hz"
        )
        self._loop = bool(self.get_parameter("loop").value)
        self._lock = threading.RLock()
        self._trajectory: Optional[_PreviewTrajectory] = None
        self._start_monotonic_s: Optional[float] = None
        self._publisher = self.create_publisher(JointState, output_topic, 10)
        preview_subscription_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self._subscription = self.create_subscription(
            JointTrajectory,
            input_topic,
            self._trajectory_callback,
            preview_subscription_qos,
        )
        self.create_timer(1.0 / playback_rate_hz, self._timer_callback)
        self.get_logger().info(
            "Cartesian preview player samples {} at {:.1f} Hz and publishes "
            "synthetic state to {}; it never writes robot commands.".format(
                input_topic, playback_rate_hz, output_topic
            )
        )

    def _joint_preview_topic_from_config_or_parameter(self) -> str:
        """Return an explicit topic override or the shared service default."""
        override = str(self.get_parameter("joint_preview_topic").value).strip()
        if override:
            return override

        services_config = load_sdk_yaml(
            str(self.get_parameter("services_config").value)
        )
        services = self._mapping(services_config.get("services"), "services")
        service_name = str(self.get_parameter("service_name").value).strip()
        if not service_name:
            raise ValueError("service_name must be non-empty")
        service_config = self._mapping(
            services.get(service_name), f"services.{service_name}"
        )
        topics = self._mapping(
            service_config.get("topics"), f"services.{service_name}.topics"
        )
        configured_topic = topics.get("joint_preview")
        if not isinstance(configured_topic, str) or not configured_topic.strip():
            raise ValueError(
                "services.{}.topics.joint_preview must be a non-empty topic".format(
                    service_name
                )
            )
        return configured_topic.strip()

    @staticmethod
    def _mapping(value: Any, name: str) -> Mapping[str, Any]:
        if not isinstance(value, dict):
            raise TypeError(f"{name} must be a mapping")
        return value

    def _topic_parameter(self, name: str) -> str:
        value = str(self.get_parameter(name).value).strip()
        if not value:
            raise ValueError(f"{name} must be a non-empty topic")
        return value

    @staticmethod
    def _positive_float(value, name: str) -> float:
        if isinstance(value, bool):
            raise TypeError(f"{name} must be numeric, not boolean")
        number = float(value)
        if not math.isfinite(number) or number <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
        return number

    def _trajectory_callback(self, message: JointTrajectory) -> None:
        try:
            trajectory = self._validated_trajectory(message)
        except (TypeError, ValueError) as error:
            self.get_logger().warning(f"Cartesian preview trajectory rejected: {error}")
            return
        with self._lock:
            self._trajectory = trajectory
            self._start_monotonic_s = time.monotonic()
        self.get_logger().info(
            "Loaded Cartesian preview trajectory with {} points over {:.3f} s.".format(
                len(trajectory.trajectory.points), trajectory.duration_s
            )
        )

    @staticmethod
    def _validated_trajectory(message: JointTrajectory) -> _PreviewTrajectory:
        names = tuple(message.joint_names)
        if len(names) != 6 or len(set(names)) != len(names):
            raise ValueError("preview trajectory must contain six unique joint names")
        if not all(name.strip() for name in names):
            raise ValueError("preview trajectory joint names must be non-empty")
        if "left_gripper_joint" in names:
            raise ValueError(
                "preview trajectory must contain six arm joints, not left_gripper_joint"
            )
        if not message.points:
            raise ValueError("preview trajectory must contain at least one point")
        points = []
        previous_time_s = -1.0
        for index, point in enumerate(message.points):
            if len(point.positions) != len(names):
                raise ValueError(f"preview point {index} has invalid position length")
            positions = tuple(float(value) for value in point.positions)
            if not all(math.isfinite(value) for value in positions):
                raise ValueError(f"preview point {index} contains non-finite positions")
            velocities = MyArmTrajectoryPreviewPlayerNode._optional_joint_values(
                point.velocities, index, "velocity"
            )
            accelerations = MyArmTrajectoryPreviewPlayerNode._optional_joint_values(
                point.accelerations, index, "acceleration"
            )
            if (velocities is None) != (accelerations is None):
                raise ValueError(
                    f"preview point {index} must provide both velocity and "
                    "acceleration "
                    "or neither"
                )
            time_s = float(point.time_from_start.sec) + float(
                point.time_from_start.nanosec
            ) * 1e-9
            if not math.isfinite(time_s) or time_s < 0.0:
                raise ValueError(f"preview point {index} has invalid timestamp")
            if index == 0 and not math.isclose(time_s, 0.0, abs_tol=1e-12):
                raise ValueError("preview trajectory must start at t=0")
            if time_s <= previous_time_s and index > 0:
                raise ValueError("preview trajectory timestamps must strictly increase")
            previous_time_s = time_s
            points.append(
                TrajectoryPoint(
                    positions=JointPositions(positions),
                    velocities=(JointPositions(velocities) if velocities else None),
                    accelerations=(
                        JointPositions(accelerations) if accelerations else None
                    ),
                    time_from_start_s=time_s,
                )
            )
        return _PreviewTrajectory(
            trajectory=SdkJointTrajectory(joint_names=names, points=tuple(points))
        )

    @staticmethod
    def _optional_joint_values(
        values: Tuple[float, ...], index: int, label: str
    ) -> Optional[Tuple[float, ...]]:
        if not values:
            return None
        if len(values) != 6:
            raise ValueError(f"preview point {index} has invalid {label} length")
        normalized = tuple(float(value) for value in values)
        if not all(math.isfinite(value) for value in normalized):
            raise ValueError(f"preview point {index} contains non-finite {label}")
        return normalized

    def _timer_callback(self) -> None:
        with self._lock:
            trajectory = self._trajectory
            start_s = self._start_monotonic_s
        if trajectory is None or start_s is None:
            return
        elapsed_s = max(0.0, time.monotonic() - start_s)
        duration_s = trajectory.duration_s
        if duration_s > 0.0 and self._loop:
            elapsed_s %= duration_s
        else:
            elapsed_s = min(elapsed_s, duration_s)
        sample = sample_joint_trajectory(trajectory.trajectory, elapsed_s)
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(trajectory.joint_names)
        message.position = list(sample.positions.values)
        message.velocity = list(sample.velocities.values)
        # ``sensor_msgs/JointState`` deliberately has no acceleration field.
        # The shared sampler still resolves qddot, which is important because
        # it keeps the preview polynomial identical to validation/execution.
        message.name.append("left_gripper_joint")
        message.position.append(self._CLOSED_GRIPPER_POSITION_RAD)
        message.velocity.append(0.0)
        self._publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = MyArmTrajectoryPreviewPlayerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
