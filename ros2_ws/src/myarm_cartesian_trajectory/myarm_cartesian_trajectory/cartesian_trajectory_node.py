"""Plan-only ROS 2 boundary for MyArm Cartesian trajectories.

This node deliberately has no driver publisher and no dependency on the
motion-execution implementation.  It turns a Cartesian planning action into a
validated ``JointTrajectory`` preview.  A later application may explicitly
submit that result to the existing ``/myarm/follow_joint_trajectory`` action;
planning a Cartesian path can therefore never move hardware by itself.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import replace
from typing import Any, Mapping, Optional, Sequence, Tuple

import rclpy
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from myarm_interfaces.action import PlanCartesianTrajectory
from myarm_sdk.core import (
    CartesianPathMode,
    CartesianTrajectoryPolicy,
    IKTaskMode,
    JointPositions,
    Pose,
    TimeScalingMode,
    TimeScalingPolicy,
    load_sdk_yaml,
)
from myarm_sdk.service import CartesianTrajectoryPlannerService
from nav_msgs.msg import Path
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory as RosJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class MyArmCartesianTrajectoryNode(Node):
    """Expose Cartesian planning as an action without owning robot I/O."""

    _SERVICES_CONFIG = "service/config/services.yaml"
    _UNKNOWN_WAYPOINT_INDEX = (1 << 32) - 1

    def __init__(self) -> None:
        super().__init__("myarm_cartesian_trajectory")
        self.declare_parameter("services_config", self._SERVICES_CONFIG)
        self.declare_parameter("service_name", "cartesian_trajectory_planner")
        # Empty parameters intentionally inherit the runtime service config.
        # Launch files can override only the endpoints that a deployment needs.
        self.declare_parameter("action_name", "")
        self.declare_parameter("measured_joint_state_topic", "")
        self.declare_parameter("reference_path_topic", "")
        self.declare_parameter("joint_preview_topic", "")
        self.declare_parameter("diagnostics_topic", "")
        self.declare_parameter("measured_state_max_age_s", 0.5)
        self.declare_parameter("tf_lookup_timeout_s", 0.25)
        # A non-positive parameter deliberately inherits service.update_rate_hz.
        self.declare_parameter("diagnostics_rate_hz", 0.0)

        services_config = load_sdk_yaml(
            str(self.get_parameter("services_config").value)
        )
        service_name = str(self.get_parameter("service_name").value)
        self._service_config = self._enabled_service_config(
            services_config, service_name
        )
        self._kinematics_service_config = self._enabled_service_config(
            services_config, "kinematics"
        )
        self._robot_config = self._mapping(services_config.get("robot"), "robot")
        self._planner = CartesianTrajectoryPlannerService.from_config(
            service_config=self._service_config,
            kinematics_service_config=self._kinematics_service_config,
            package_share_directory=get_package_share_directory,
            robot_config=self._robot_config,
        )
        self._joint_names = tuple(self._planner.joint_names)
        self._validate_joint_names(self._joint_names)
        self._base_frame = self._frame_name(self._planner.base_frame, "base_frame")
        self._tool_frame = self._frame_name(self._planner.tool_frame, "tool_frame")

        configured_topics = self._mapping_or_empty(
            self._service_config.get("topics"), "cartesian_trajectory_planner topics"
        )
        measured_topic_override = str(
            self.get_parameter("measured_joint_state_topic").value
        ).strip()
        self._measured_joint_state_topic = measured_topic_override or self._topic(
            configured_topics, "measured_joint_state", "/myarm/state/joint_state"
        )
        self._reference_path_topic = self._parameter_topic(
            "reference_path_topic",
            self._topic(
                configured_topics,
                "reference_path",
                "/myarm/cartesian_trajectory/reference_path",
            ),
        )
        self._joint_preview_topic = self._parameter_topic(
            "joint_preview_topic",
            self._topic(
                configured_topics,
                "joint_preview",
                "/myarm/cartesian_trajectory/joint_preview",
            ),
        )
        self._diagnostics_topic = self._parameter_topic(
            "diagnostics_topic",
            self._topic(
                configured_topics,
                "diagnostics",
                "/myarm/cartesian_trajectory/diagnostics",
            ),
        )
        self._action_name = self._parameter_topic(
            "action_name",
            self._topic(
                configured_topics,
                "plan_action",
                "/myarm/plan_cartesian_trajectory",
            ),
        )
        self._measured_state_max_age_s = self._positive_float(
            self.get_parameter("measured_state_max_age_s").value,
            "measured_state_max_age_s",
        )
        feedback_config = self._mapping_or_empty(
            self._service_config.get("feedback"),
            "cartesian_trajectory_planner feedback",
        )
        if "measured_state_max_age_s" in feedback_config:
            self._measured_state_max_age_s = self._positive_float(
                feedback_config["measured_state_max_age_s"],
                "cartesian_trajectory_planner.feedback.measured_state_max_age_s",
            )
        self._tf_lookup_timeout_s = self._positive_float(
            self.get_parameter("tf_lookup_timeout_s").value,
            "tf_lookup_timeout_s",
        )
        configured_update_rate_hz = self._positive_float(
            self._service_config.get("update_rate_hz", 5.0),
            "cartesian_trajectory_planner.update_rate_hz",
        )
        diagnostics_rate_value = float(
            self.get_parameter("diagnostics_rate_hz").value
        )
        diagnostics_rate_hz = (
            configured_update_rate_hz
            if diagnostics_rate_value <= 0.0
            else self._positive_float(
                diagnostics_rate_value, "diagnostics_rate_hz"
            )
        )

        self._state_lock = threading.RLock()
        self._measured_joint_positions: Optional[JointPositions] = None
        self._measured_received_at_s: Optional[float] = None
        self._active_goal = None
        self._goal_reserved = False
        self._state = "idle"
        self._last_detail = "waiting for a Cartesian planning goal"
        self._last_result = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        latched_preview_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._reference_path_publisher = self.create_publisher(
            Path, self._reference_path_topic, latched_preview_qos
        )
        self._joint_preview_publisher = self.create_publisher(
            RosJointTrajectory, self._joint_preview_topic, latched_preview_qos
        )
        self._diagnostics_publisher = self.create_publisher(
            DiagnosticArray, self._diagnostics_topic, 10
        )
        self._measured_joint_subscription = self.create_subscription(
            JointState,
            self._measured_joint_state_topic,
            self._measured_joint_state_callback,
            qos_profile_sensor_data,
        )
        self._action_callback_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            PlanCartesianTrajectory,
            self._action_name,
            execute_callback=self._execute_plan,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._action_callback_group,
        )
        self.create_timer(1.0 / diagnostics_rate_hz, self._publish_diagnostics)
        self.get_logger().info(
            "myarm_cartesian_trajectory is plan/preview only; action={}, "
            "feedback={}, preview={}".format(
                self._action_name,
                self._measured_joint_state_topic,
                self._joint_preview_topic,
            )
        )

    @staticmethod
    def _mapping(value: Any, name: str) -> Mapping[str, Any]:
        if not isinstance(value, dict):
            raise TypeError(f"{name} must be a mapping")
        return value

    @staticmethod
    def _mapping_or_empty(value: Any, name: str) -> Mapping[str, Any]:
        if value is None:
            return {}
        if not isinstance(value, dict):
            raise TypeError(f"{name} must be a mapping")
        return value

    @classmethod
    def _enabled_service_config(
        cls, services_config: Mapping[str, Any], name: str
    ) -> Mapping[str, Any]:
        services = cls._mapping(services_config.get("services"), "services")
        config = cls._mapping(services.get(name), f"services.{name}")
        if config.get("enabled") is not True:
            raise RuntimeError(f"{name} service is disabled in services.yaml")
        return config

    @staticmethod
    def _frame_name(value: Any, name: str) -> str:
        if not isinstance(value, str) or not value.strip() or value.startswith("/"):
            raise ValueError(f"{name} must be a non-empty relative TF frame")
        return value.strip()

    @staticmethod
    def _topic(
        configured_topics: Mapping[str, Any], name: str, default: str
    ) -> str:
        value = configured_topics.get(name, default)
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f"cartesian trajectory topic {name} must be non-empty")
        return value.strip()

    def _parameter_topic(self, parameter_name: str, configured_default: str) -> str:
        value = str(self.get_parameter(parameter_name).value).strip()
        return value or configured_default

    @staticmethod
    def _positive_float(value: Any, name: str) -> float:
        if isinstance(value, bool):
            raise TypeError(f"{name} must be numeric, not boolean")
        try:
            number = float(value)
        except (TypeError, ValueError) as error:
            raise TypeError(f"{name} must be numeric") from error
        if not math.isfinite(number) or number <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
        return number

    @staticmethod
    def _validate_joint_names(joint_names: Sequence[str]) -> None:
        names = tuple(joint_names)
        if len(names) != 6 or len(set(names)) != len(names):
            raise ValueError("MyArm Cartesian trajectories require six unique joints")
        if not all(isinstance(name, str) and name.strip() for name in names):
            raise ValueError("canonical joint names must be non-empty strings")

    def _measured_joint_state_callback(self, message: JointState) -> None:
        try:
            joints = self._canonical_joint_positions_from_message(message)
        except (TypeError, ValueError) as error:
            self._set_problem(f"invalid measured joint state: {error}")
            self.get_logger().warning(f"Measured joint state rejected: {error}")
            return
        with self._state_lock:
            self._measured_joint_positions = joints
            self._measured_received_at_s = time.monotonic()

    def _canonical_joint_positions_from_message(
        self, message: JointState
    ) -> JointPositions:
        if message.name:
            if len(message.name) != len(message.position):
                raise ValueError("JointState name and position lengths differ")
            if len(set(message.name)) != len(message.name):
                raise ValueError("JointState contains duplicate joint names")
            positions_by_name = dict(zip(message.name, message.position))
            missing = tuple(
                name for name in self._joint_names if name not in positions_by_name
            )
            if missing:
                raise ValueError(
                    "JointState is missing arm joints: {}".format(", ".join(missing))
                )
            return JointPositions(
                tuple(positions_by_name[name] for name in self._joint_names)
            )
        if len(message.position) != len(self._joint_names):
            raise ValueError(
                "unnamed JointState must contain exactly six canonical arm positions"
            )
        return JointPositions(message.position)

    def _fresh_measured_joint_positions(
        self, now_monotonic_s: float
    ) -> Tuple[Optional[JointPositions], Optional[float]]:
        with self._state_lock:
            positions = self._measured_joint_positions
            received_at_s = self._measured_received_at_s
        if positions is None or received_at_s is None:
            return None, None
        age_s = max(0.0, now_monotonic_s - received_at_s)
        if age_s > self._measured_state_max_age_s:
            return None, age_s
        return positions, age_s

    def _goal_callback(self, goal_request) -> GoalResponse:
        try:
            self._validate_goal(goal_request)
        except (TypeError, ValueError) as error:
            self.get_logger().warning(f"Cartesian goal rejected: {error}")
            return GoalResponse.REJECT
        with self._state_lock:
            if self._active_goal is not None or self._goal_reserved:
                self.get_logger().warning("Cartesian goal rejected: planner is busy")
                return GoalResponse.REJECT
            self._goal_reserved = True
            self._state = "planning"
            self._last_detail = "Cartesian goal accepted; waiting for fresh feedback"
            self._last_result = None
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        with self._state_lock:
            active = goal_handle is self._active_goal
        return CancelResponse.ACCEPT if active else CancelResponse.REJECT

    def _execute_plan(self, goal_handle):
        result_message = PlanCartesianTrajectory.Result()
        with self._state_lock:
            self._goal_reserved = False
            self._active_goal = goal_handle
            self._state = "planning"
        try:
            if goal_handle.is_cancel_requested:
                return self._cancelled_result(goal_handle, result_message)
            q_start, _ = self._fresh_measured_joint_positions(time.monotonic())
            if q_start is None:
                detail = "fresh measured joint state is required before Cartesian planning"
                return self._abort_result(
                    goal_handle, result_message, "stale_measured_state", detail
                )
            target_pose = self._target_pose_from_goal(goal_handle.request.target_pose)
            policy = self._policy_from_goal(goal_handle.request)
            self._publish_action_feedback(goal_handle, 0, 0, 0.0, "planning")
            planning_result = self._planner.plan_cartesian_motion(
                q_start=q_start,
                target_pose=target_pose,
                policy=policy,
            )
            if goal_handle.is_cancel_requested:
                return self._cancelled_result(goal_handle, result_message)

            self._copy_planning_result(result_message, planning_result)
            if not planning_result.succeeded or planning_result.trajectory is None:
                failure_reason = self._failure_reason(planning_result)
                detail = str(planning_result.detail)
                return self._abort_result(
                    goal_handle, result_message, failure_reason, detail,
                    planning_result=planning_result,
                )

            self._publish_preview(planning_result, q_start, target_pose)
            self._publish_action_feedback(
                goal_handle,
                int(getattr(planning_result, "waypoint_count", 0)),
                int(getattr(planning_result, "waypoint_count", 0)),
                1.0,
                "completed",
            )
            with self._state_lock:
                self._state = "succeeded"
                self._last_detail = str(planning_result.detail)
                self._last_result = planning_result
            goal_handle.succeed()
            return result_message
        except (TypeError, ValueError) as error:
            return self._abort_result(
                goal_handle, result_message, "invalid_goal", str(error)
            )
        except TransformException as error:
            return self._abort_result(
                goal_handle, result_message, "frame_transform_failed", str(error)
            )
        except Exception as error:  # noqa: BLE001 - report planner boundary failures.
            detail = f"Cartesian planning backend error: {error}"
            self.get_logger().error(detail)
            return self._abort_result(
                goal_handle, result_message, "planner_backend_error", detail
            )
        finally:
            with self._state_lock:
                if self._active_goal is goal_handle:
                    self._active_goal = None
                self._goal_reserved = False

    def _validate_goal(self, goal_request) -> None:
        frame_id = goal_request.target_pose.header.frame_id
        self._frame_name(frame_id, "target_pose.header.frame_id")
        self._sdk_pose_from_message(goal_request.target_pose)
        self._policy_from_goal(goal_request)

    def _target_pose_from_goal(self, source: PoseStamped) -> Pose:
        source_frame = self._frame_name(source.header.frame_id, "target pose frame")
        if source_frame == self._base_frame:
            return self._sdk_pose_from_message(source)
        target_time = Time()
        if source.header.stamp.sec != 0 or source.header.stamp.nanosec != 0:
            target_time = Time.from_msg(source.header.stamp)
        transform = self._tf_buffer.lookup_transform(
            self._base_frame,
            source_frame,
            target_time,
            timeout=Duration(seconds=self._tf_lookup_timeout_s),
        )
        # Foxy exposes the stamped helper separately; the un-stamped helper
        # accepts only ``geometry_msgs/Pose``.
        transformed = do_transform_pose_stamped(source, transform)
        transformed.header.frame_id = self._base_frame
        return self._sdk_pose_from_message(transformed)

    @staticmethod
    def _sdk_pose_from_message(message: PoseStamped) -> Pose:
        position = message.pose.position
        orientation = message.pose.orientation
        values = (
            position.x,
            position.y,
            position.z,
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("target pose contains non-finite values")
        return Pose(
            position=(position.x, position.y, position.z),
            orientation=(orientation.x, orientation.y, orientation.z, orientation.w),
        )

    def _policy_from_goal(self, goal_request) -> CartesianTrajectoryPolicy:
        default = self._planner.default_policy
        if goal_request.path_mode == PlanCartesianTrajectory.Goal.PATH_DEFAULT:
            path_mode = default.path_mode
        else:
            path_mode = {
                PlanCartesianTrajectory.Goal.PATH_LINEAR_TRANSLATION_SLERP: (
                    CartesianPathMode.LINEAR_TRANSLATION_SLERP
                ),
                PlanCartesianTrajectory.Goal.PATH_SE3_GEODESIC: (
                    CartesianPathMode.SE3_GEODESIC
                ),
            }.get(goal_request.path_mode)
            if path_mode is None:
                raise ValueError(
                    f"unsupported Cartesian path_mode: {goal_request.path_mode}"
                )
        if goal_request.task_mode == PlanCartesianTrajectory.Goal.TASK_DEFAULT:
            task_mode = default.ik_policy.task_mode
        else:
            task_mode = {
                PlanCartesianTrajectory.Goal.TASK_FULL_POSE: IKTaskMode.FULL_POSE,
                PlanCartesianTrajectory.Goal.TASK_POSITION_ONLY: (
                    IKTaskMode.POSITION_ONLY
                ),
            }.get(goal_request.task_mode)
            if task_mode is None:
                raise ValueError(f"unsupported IK task_mode: {goal_request.task_mode}")
        requested_duration_s = self._duration_seconds(goal_request.requested_duration)
        time_mode_value = goal_request.time_scaling_mode
        if time_mode_value == PlanCartesianTrajectory.Goal.TIME_DEFAULT:
            if requested_duration_s is not None or float(goal_request.speed_scale) != 0.0:
                raise ValueError(
                    "TIME_DEFAULT requires zero requested_duration and speed_scale"
                )
            time_scaling = default.time_scaling
        else:
            time_mode = {
                PlanCartesianTrajectory.Goal.TIME_AUTO_LIMITED: (
                    TimeScalingMode.AUTO_LIMITED
                ),
                PlanCartesianTrajectory.Goal.TIME_REQUESTED_DURATION_STRETCH: (
                    TimeScalingMode.REQUESTED_DURATION_STRETCH
                ),
                PlanCartesianTrajectory.Goal.TIME_REQUESTED_DURATION_STRICT: (
                    TimeScalingMode.REQUESTED_DURATION_STRICT
                ),
                PlanCartesianTrajectory.Goal.TIME_SPEED_SCALE: (
                    TimeScalingMode.SPEED_SCALE
                ),
            }.get(time_mode_value)
            if time_mode is None:
                raise ValueError(
                    f"unsupported time_scaling_mode: {time_mode_value}"
                )
            if time_mode in (
                TimeScalingMode.REQUESTED_DURATION_STRETCH,
                TimeScalingMode.REQUESTED_DURATION_STRICT,
            ) and requested_duration_s is None:
                raise ValueError(
                    "requested duration mode requires requested_duration > 0"
                )
            if time_mode in (
                TimeScalingMode.AUTO_LIMITED,
                TimeScalingMode.SPEED_SCALE,
            ):
                requested_duration_s = None
            speed_scale = float(goal_request.speed_scale)
            if time_mode is TimeScalingMode.SPEED_SCALE:
                if not math.isfinite(speed_scale) or not 0.0 < speed_scale <= 1.0:
                    raise ValueError("speed_scale mode requires 0 < speed_scale <= 1")
            else:
                speed_scale = None
            time_scaling = TimeScalingPolicy(
                mode=time_mode,
                requested_duration_s=requested_duration_s,
                speed_scale=speed_scale,
                sample_period_s=default.time_scaling.sample_period_s,
            )
        max_translation_step_m = self._optional_positive_override(
            goal_request.max_translation_step_m,
            default.max_translation_step_m,
            "max_translation_step_m",
        )
        max_rotation_step_rad = self._optional_positive_override(
            goal_request.max_rotation_step_rad,
            default.max_rotation_step_rad,
            "max_rotation_step_rad",
        )
        return replace(
            default,
            path_mode=path_mode,
            ik_policy=replace(default.ik_policy, task_mode=task_mode),
            time_scaling=time_scaling,
            max_translation_step_m=max_translation_step_m,
            max_rotation_step_rad=max_rotation_step_rad,
        )

    @staticmethod
    def _optional_positive_override(value: Any, default: float, name: str) -> float:
        number = float(value)
        if not math.isfinite(number):
            raise ValueError(f"{name} must be finite")
        if number <= 0.0:
            return default
        return number

    @staticmethod
    def _duration_seconds(duration) -> Optional[float]:
        seconds = int(duration.sec)
        nanoseconds = int(duration.nanosec)
        if seconds < 0 or nanoseconds < 0 or nanoseconds >= 1000000000:
            raise ValueError("requested_duration must be non-negative")
        value = float(seconds) + float(nanoseconds) * 1e-9
        if not math.isfinite(value):
            raise ValueError("requested_duration must be finite")
        return value if value > 0.0 else None

    def _publish_preview(
        self, planning_result, q_start: JointPositions, target_pose: Pose
    ) -> None:
        trajectory = planning_result.trajectory
        if trajectory is None:
            return
        self._joint_preview_publisher.publish(
            self._trajectory_to_ros(trajectory, stamp_now=True)
        )
        self._reference_path_publisher.publish(
            self._reference_path_from_result(planning_result, q_start, target_pose)
        )

    def _trajectory_to_ros(self, trajectory, *, stamp_now: bool = False) -> RosJointTrajectory:
        message = RosJointTrajectory()
        # Keep an action result executable by the existing
        # FollowJointTrajectory boundary, which deliberately accepts only a
        # zero header timestamp.  Published preview data, by contrast, is
        # stamped for visualization consumers.
        if stamp_now:
            message.header.stamp = self.get_clock().now().to_msg()
        message.joint_names = list(trajectory.joint_names)
        message.points = []
        for point in trajectory.points:
            ros_point = JointTrajectoryPoint()
            ros_point.positions = list(point.positions.values)
            ros_point.velocities = (
                list(point.velocities.values) if point.velocities is not None else []
            )
            ros_point.accelerations = (
                list(point.accelerations.values)
                if point.accelerations is not None
                else []
            )
            seconds, nanoseconds = self._seconds_to_duration(point.time_from_start_s)
            ros_point.time_from_start.sec = seconds
            ros_point.time_from_start.nanosec = nanoseconds
            message.points.append(ros_point)
        return message

    def _reference_path_from_result(
        self, planning_result, q_start: JointPositions, target_pose: Pose
    ) -> Path:
        """Publish planned reference poses when core exposes them.

        The first pycore implementation may expose the path as either
        ``reference_poses`` or ``reference_path``.  The fallback contains the
        target only rather than fabricating a path that was not validated.
        ``forward`` is intentionally optional to keep the ROS boundary free of
        adapter internals while allowing a planner service to expose FK later.
        """
        poses = tuple(getattr(planning_result, "reference_poses", ()) or ())
        if not poses:
            poses = tuple(getattr(planning_result, "reference_path", ()) or ())
        if not poses:
            forward = getattr(self._planner, "forward", None)
            if callable(forward):
                try:
                    trajectory = planning_result.trajectory
                    q_values = [q_start]
                    if trajectory is not None:
                        q_values.extend(point.positions for point in trajectory.points)
                    poses = tuple(forward(q) for q in q_values)
                except Exception as error:  # noqa: BLE001 - preview remains best-effort.
                    self.get_logger().warning(
                        f"Unable to construct Cartesian preview path from FK: {error}"
                    )
                    poses = ()
        if not poses:
            poses = (target_pose,)
        message = Path()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._base_frame
        for pose in poses:
            if not isinstance(pose, Pose):
                continue
            pose_message = PoseStamped()
            pose_message.header = message.header
            pose_message.pose.position.x, pose_message.pose.position.y, pose_message.pose.position.z = (
                pose.position
            )
            (
                pose_message.pose.orientation.x,
                pose_message.pose.orientation.y,
                pose_message.pose.orientation.z,
                pose_message.pose.orientation.w,
            ) = pose.orientation
            message.poses.append(pose_message)
        return message

    @staticmethod
    def _seconds_to_duration(value: float) -> Tuple[int, int]:
        if not math.isfinite(value) or value < 0.0:
            raise ValueError("duration must be finite and non-negative")
        seconds = math.floor(value)
        nanoseconds = round((value - seconds) * 1000000000.0)
        if nanoseconds == 1000000000:
            seconds += 1
            nanoseconds = 0
        return seconds, nanoseconds

    def _copy_planning_result(self, message, result) -> None:
        message.succeeded = bool(result.succeeded)
        message.trajectory = (
            self._trajectory_to_ros(result.trajectory)
            if result.trajectory is not None
            else RosJointTrajectory()
        )
        message.failure_reason = self._failure_reason(result)
        message.detail = str(result.detail)
        failed_index = getattr(result, "failed_waypoint_index", None)
        message.failed_waypoint_index = (
            self._UNKNOWN_WAYPOINT_INDEX
            if failed_index is None
            else max(0, int(failed_index))
        )
        message.resolved_duration_s = self._result_number(
            getattr(result, "resolved_duration_s", None)
        )
        message.minimum_joint_limit_margin_rad = self._result_number(
            getattr(result, "minimum_joint_limit_margin_rad", None)
        )
        message.minimum_singular_value = self._result_number(
            getattr(result, "minimum_singular_value", None)
        )
        message.maximum_position_residual_m = self._result_number(
            getattr(result, "maximum_position_residual_m", None)
        )
        message.maximum_orientation_residual_rad = self._result_number(
            getattr(result, "maximum_orientation_residual_rad", None)
        )

    @staticmethod
    def _result_number(value: Any) -> float:
        try:
            number = float(value)
        except (TypeError, ValueError):
            return 0.0
        return number if math.isfinite(number) else 0.0

    @staticmethod
    def _failure_reason(result) -> str:
        reason = getattr(result, "failure_reason", None)
        return reason.value if reason is not None and hasattr(reason, "value") else ""

    def _abort_result(
        self,
        goal_handle,
        result_message,
        failure_reason: str,
        detail: str,
        planning_result=None,
    ):
        if planning_result is not None:
            self._copy_planning_result(result_message, planning_result)
        else:
            result_message.failed_waypoint_index = self._UNKNOWN_WAYPOINT_INDEX
        result_message.succeeded = False
        result_message.failure_reason = failure_reason
        result_message.detail = detail
        with self._state_lock:
            self._state = "failed"
            self._last_detail = detail
            self._last_result = planning_result
        goal_handle.abort()
        return result_message

    def _cancelled_result(self, goal_handle, result_message):
        result_message.succeeded = False
        result_message.failure_reason = "canceled"
        result_message.detail = "Cartesian planning goal canceled"
        result_message.failed_waypoint_index = self._UNKNOWN_WAYPOINT_INDEX
        with self._state_lock:
            self._state = "canceled"
            self._last_detail = result_message.detail
            self._last_result = None
        goal_handle.canceled()
        return result_message

    @staticmethod
    def _publish_action_feedback(
        goal_handle, waypoint_index: int, waypoint_count: int, progress: float, stage: str
    ) -> None:
        feedback = PlanCartesianTrajectory.Feedback()
        feedback.waypoint_index = max(0, int(waypoint_index))
        feedback.waypoint_count = max(0, int(waypoint_count))
        feedback.progress = max(0.0, min(1.0, float(progress)))
        feedback.stage = stage
        goal_handle.publish_feedback(feedback)

    def _set_problem(self, detail: str) -> None:
        with self._state_lock:
            if self._state != "planning":
                self._state = "failed"
            self._last_detail = detail

    def _publish_diagnostics(self) -> None:
        now = time.monotonic()
        _, age_s = self._fresh_measured_joint_positions(now)
        with self._state_lock:
            state = self._state
            detail = self._last_detail
            planning_result = self._last_result
            active = self._active_goal is not None or self._goal_reserved
        level = DiagnosticStatus.OK
        if active or state == "planning":
            level = DiagnosticStatus.WARN
        elif state == "failed":
            level = DiagnosticStatus.ERROR
        elif state == "canceled":
            level = DiagnosticStatus.WARN
        if age_s is None:
            level = max(level, DiagnosticStatus.WARN)
        values = [
            KeyValue(key="state", value=state),
            KeyValue(key="active_goal", value=str(active)),
            KeyValue(key="base_frame", value=self._base_frame),
            KeyValue(key="tool_frame", value=self._tool_frame),
            KeyValue(key="measured_state_fresh", value=str(age_s is not None)),
            KeyValue(key="measured_state_age_s", value=self._format_number(age_s)),
            KeyValue(key="detail", value=detail),
        ]
        if planning_result is not None:
            values.extend([
                KeyValue(key="succeeded", value=str(planning_result.succeeded)),
                KeyValue(key="failure_reason", value=self._failure_reason(planning_result)),
                KeyValue(
                    key="waypoint_count",
                    value=str(getattr(planning_result, "waypoint_count", 0)),
                ),
                KeyValue(
                    key="resolved_duration_s",
                    value=self._format_number(
                        getattr(planning_result, "resolved_duration_s", None)
                    ),
                ),
                KeyValue(
                    key="minimum_joint_limit_margin_rad",
                    value=self._format_number(
                        getattr(
                            planning_result,
                            "minimum_joint_limit_margin_rad",
                            None,
                        )
                    ),
                ),
                KeyValue(
                    key="minimum_singular_value",
                    value=self._format_number(
                        getattr(planning_result, "minimum_singular_value", None)
                    ),
                ),
            ])
        diagnostic = DiagnosticArray()
        diagnostic.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.level = level
        status.name = "myarm/cartesian_trajectory"
        status.hardware_id = "myarm_m750"
        status.message = state
        status.values = values
        diagnostic.status = [status]
        self._diagnostics_publisher.publish(diagnostic)

    @staticmethod
    def _format_number(value: Any) -> str:
        if value is None:
            return ""
        try:
            number = float(value)
        except (TypeError, ValueError):
            return ""
        return f"{number:.9g}" if math.isfinite(number) else ""

    def destroy_node(self):
        self._action_server.destroy()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = MyArmCartesianTrajectoryNode()
        from rclpy.executors import MultiThreadedExecutor

        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
