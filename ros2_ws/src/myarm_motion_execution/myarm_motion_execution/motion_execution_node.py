"""ROS 2 boundary for validated MyArm joint motion planning and execution.

This node owns high-level joint-motion state only.  It reads measured joints,
plans a bounded minimum-jerk trajectory for a public ``joint_goal`` topic,
and samples that trajectory at a monotonic clock.  The resulting private
setpoints are forwarded to the separate robot-driver node, which remains the
sole owner of the fake or physical robot connection.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import replace
from pathlib import Path
from typing import Any, Mapping, Optional, Sequence, Tuple

import rclpy
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from myarm_interfaces.action import FollowCartesianTrajectory
from myarm_interfaces.msg import DriverJointSetpoint
from myarm_sdk.core import (
    CartesianPathMode,
    CartesianTrajectoryPolicy,
    IKTaskMode,
    JointPositions,
    JointTrajectory,
    MotionExecutionFailureReason,
    MotionExecutionState,
    Pose,
    TimeScalingMode,
    TimeScalingPolicy,
    TrajectoryPoint,
    load_sdk_yaml,
    load_urdf_joint_metadata,
)
from myarm_sdk.service import (
    CartesianTrajectoryPlannerService,
    JointTrajectoryPlannerService,
    MotionExecutionService,
)
from nav_msgs.msg import Path as RosPath
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory as RosJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from .joint_state_mapping import canonical_joint_positions_from_names


class MyArmMotionExecutionNode(Node):
    """Coordinate planner and executor services without owning robot I/O.

    Public joint goals are deliberately conservative: a new goal is planned
    only from fresh measured feedback, and it is rejected while a motion is
    active. ``FollowJointTrajectory`` accepts an explicit full joint
    trajectory. The optional fake-only ``FollowCartesianTrajectory`` action
    plans a TCP target, then uses this same executor path; neither action can
    bypass the driver safety gate or private setpoint boundary.
    """

    _SERVICES_CONFIG = "service/config/services.yaml"

    def __init__(self) -> None:
        super().__init__("myarm_motion_execution")
        self.declare_parameter("services_config", self._SERVICES_CONFIG)
        self.declare_parameter("enable_cartesian_execution", False)
        services_config = load_sdk_yaml(
            str(self.get_parameter("services_config").value)
        )
        self._robot_config = self._mapping(services_config.get("robot"), "robot")
        self._joint_trajectory_planner_config = self._enabled_service_config(
            services_config, "joint_trajectory_planner"
        )
        self._execution_config = self._enabled_service_config(
            services_config, "motion_execution"
        )
        self._cartesian_execution_enabled = bool(
            self.get_parameter("enable_cartesian_execution").value
        )
        self._topics = self._mapping(
            self._execution_config.get("topics"), "motion_execution topics"
        )

        joint_metadata = self._load_joint_metadata()
        self._joint_trajectory_planner = JointTrajectoryPlannerService.from_config(
            service_config=self._joint_trajectory_planner_config,
            joint_metadata=joint_metadata,
        )
        self._motion_execution = MotionExecutionService.from_config(
            service_config=self._execution_config
        )
        motion_limits = self._joint_trajectory_planner.motion_limits
        if motion_limits is None:
            raise RuntimeError(
                "JointTrajectoryPlannerService must expose motion limits"
            )
        self._joint_names = tuple(motion_limits.joint_names)
        self._validate_joint_names(self._joint_names)

        self._cartesian_planner = None
        self._cartesian_base_frame = ""
        self._cartesian_reference_path_publisher = None
        self._cartesian_tf_buffer = None
        self._cartesian_tf_listener = None
        self._active_cartesian_waypoint_count = 0
        if self._cartesian_execution_enabled:
            self._configure_fake_cartesian_execution(services_config)

        self._state_lock = threading.RLock()
        self._measured_joint_positions: Optional[JointPositions] = None
        self._measured_received_at_s: Optional[float] = None
        self._pending_joint_goal: Optional[JointPositions] = None
        self._active_action_goal = None
        self._action_goal_reserved = False
        self._action_completion = threading.Event()
        self._active_origin = ""
        self._last_event = None
        self._last_problem = ""
        self._last_planning_detail = ""
        self._driver_safety_state = "unknown"
        self._driver_safety_epoch = 0

        self._setpoint_publisher = self.create_publisher(
            DriverJointSetpoint, self._required_topic("internal_setpoint"), 10
        )
        self._preview_publisher = self.create_publisher(
            RosJointTrajectory, self._required_topic("preview"), 10
        )
        self._diagnostics_publisher = self.create_publisher(
            DiagnosticArray, self._required_topic("diagnostics"), 10
        )
        self._joint_goal_subscription = self.create_subscription(
            JointState,
            self._required_topic("joint_goal"),
            self._joint_goal_callback,
            10,
        )
        self._measured_joint_subscription = self.create_subscription(
            JointState,
            self._required_topic("measured_joint_state"),
            self._measured_joint_state_callback,
            qos_profile_sensor_data,
        )
        self._driver_safety_subscription = self.create_subscription(
            String,
            self._required_topic("driver_safety_state"),
            self._driver_safety_state_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self._driver_stop_client = self.create_client(
            Trigger, self._required_topic("driver_stop_service")
        )
        self._cancel_service = self.create_service(
            Trigger,
            self._required_topic("cancel_service"),
            self._cancel_service_callback,
        )
        self._reset_service = self.create_service(
            Trigger,
            self._required_topic("reset_service"),
            self._reset_service_callback,
        )

        self._action_callback_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            self._required_topic("follow_joint_trajectory"),
            execute_callback=self._execute_follow_joint_trajectory,
            goal_callback=self._follow_joint_trajectory_goal_callback,
            cancel_callback=self._follow_joint_trajectory_cancel_callback,
            callback_group=self._action_callback_group,
        )
        self._cartesian_action_server = None
        if self._cartesian_execution_enabled:
            self._cartesian_action_server = ActionServer(
                self,
                FollowCartesianTrajectory,
                self._required_topic("follow_cartesian_trajectory"),
                execute_callback=self._execute_follow_cartesian_trajectory,
                goal_callback=self._follow_cartesian_trajectory_goal_callback,
                cancel_callback=self._follow_cartesian_trajectory_cancel_callback,
                callback_group=self._action_callback_group,
            )

        update_rate_hz = self._positive_float(
            self._motion_execution.settings.update_rate_hz,
            "motion_execution update_rate_hz",
        )
        self.create_timer(1.0 / update_rate_hz, self._timer_callback)
        self.get_logger().info(
            "myarm_motion_execution is running at {} Hz; it plans fresh "
            "measured state on {} and streams private setpoints on {}.".format(
                self._format_number(update_rate_hz),
                self._required_topic("joint_goal"),
                self._required_topic("internal_setpoint"),
            )
        )
        if self._cartesian_execution_enabled:
            self.get_logger().warning(
                "FollowCartesianTrajectory is enabled for FakeRobotArm only; "
                "it uses the existing motion executor and driver safety gate."
            )

    @staticmethod
    def _mapping(value: Any, name: str) -> Mapping[str, Any]:
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
        if len(names) != 6:
            raise ValueError("MyArm M750 requires six canonical arm joint names")
        if not all(isinstance(name, str) and name for name in names):
            raise ValueError("canonical arm joint names must be non-empty strings")
        if len(set(names)) != len(names):
            raise ValueError("canonical arm joint names must be unique")

    def _required_topic(self, name: str) -> str:
        value = self._topics.get(name)
        if not isinstance(value, str) or not value.strip():
            raise ValueError(
                f"motion_execution topics.{name} must be a non-empty string"
            )
        return value.strip()

    def _load_joint_metadata(self):
        """Load canonical limits from the same configured URDF as other services."""
        joint_order = self._mapping(
            self._robot_config.get("joint_order"), "robot.joint_order"
        )
        if joint_order.get("source") != "urdf":
            raise ValueError("robot.joint_order.source must be 'urdf'")
        joint_names = tuple(joint_order.get("names", ()))
        self._validate_joint_names(joint_names)

        description = self._mapping(
            self._robot_config.get("robot_description"), "robot.robot_description"
        )
        package_name = description.get("package")
        relative_path = description.get("relative_path")
        if not isinstance(package_name, str) or not package_name.strip():
            raise ValueError("robot.robot_description.package must be non-empty")
        if not isinstance(relative_path, str) or not relative_path.strip():
            raise ValueError("robot.robot_description.relative_path must be non-empty")
        path = Path(relative_path)
        if path.is_absolute() or ".." in path.parts:
            raise ValueError("robot.robot_description.relative_path must stay in package")
        urdf_path = Path(get_package_share_directory(package_name)) / path
        return load_urdf_joint_metadata(urdf_path, joint_names)

    def _configure_fake_cartesian_execution(
        self, services_config: Mapping[str, Any]
    ) -> None:
        """Build the isolated Cartesian planner for the fake-only execution phase.

        The executor owns this capability because it owns execution state and
        private setpoint publication.  It intentionally constructs a separate
        Pinocchio instance instead of sharing mutable data with the plan-only
        Cartesian node.
        """
        services = self._mapping(services_config.get("services"), "services")
        robot_arm_config = self._mapping(
            services.get("robot_arm"), "services.robot_arm"
        )
        if robot_arm_config.get("plugin_adapter") != "fake_robot_arm":
            raise RuntimeError(
                "FollowCartesianTrajectory is restricted to "
                "services.robot_arm.plugin_adapter=fake_robot_arm in this phase"
            )
        cartesian_config = self._enabled_service_config(
            services_config, "cartesian_trajectory_planner"
        )
        kinematics_config = self._enabled_service_config(
            services_config, "kinematics"
        )
        planner = CartesianTrajectoryPlannerService.from_config(
            service_config=cartesian_config,
            kinematics_service_config=kinematics_config,
            package_share_directory=get_package_share_directory,
            robot_config=self._robot_config,
        )
        if tuple(planner.joint_names) != self._joint_names:
            raise RuntimeError(
                "Cartesian planner joint order must match motion execution"
            )
        self._cartesian_planner = planner
        self._cartesian_base_frame = self._frame_name(
            planner.base_frame, "cartesian base_frame"
        )
        reference_path_topic = self._required_topic("cartesian_reference_path")
        self._cartesian_reference_path_publisher = self.create_publisher(
            RosPath,
            reference_path_topic,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self._cartesian_tf_buffer = Buffer()
        self._cartesian_tf_listener = TransformListener(
            self._cartesian_tf_buffer, self, spin_thread=False
        )

    @staticmethod
    def _frame_name(value: Any, name: str) -> str:
        if not isinstance(value, str) or not value.strip() or value.startswith("/"):
            raise ValueError(f"{name} must be a non-empty relative TF frame")
        return value.strip()

    def _measured_joint_state_callback(self, message: JointState) -> None:
        try:
            positions = canonical_joint_positions_from_names(
                names=message.name,
                positions=message.position,
                canonical_joint_names=self._joint_names,
            )
        except (TypeError, ValueError) as error:
            self._record_problem(f"invalid measured joint state: {error}")
            self.get_logger().warning(f"Measured joint state rejected: {error}")
            return
        with self._state_lock:
            self._measured_joint_positions = positions
            self._measured_received_at_s = time.monotonic()

    def _driver_safety_state_callback(self, message: String) -> None:
        fields = {}
        for part in message.data.split(";"):
            key, separator, value = part.partition("=")
            if separator:
                fields[key] = value
        try:
            epoch = int(fields["epoch"])
        except (KeyError, TypeError, ValueError):
            self._record_problem("invalid driver safety-state message")
            return
        state = fields.get("state", "unknown")
        with self._state_lock:
            self._driver_safety_state = state
            self._driver_safety_epoch = epoch
        if state != "armed" and self._motion_execution.state == MotionExecutionState.EXECUTING:
            self._motion_execution.fault(
                MotionExecutionFailureReason.EXTERNAL_FAULT,
                detail=f"driver safety gate closed: {message.data}",
            )
            self._action_completion.set()

    def _joint_goal_callback(self, message: JointState) -> None:
        """Accept a public end-point goal only when no execution is active."""
        try:
            target = canonical_joint_positions_from_names(
                names=message.name,
                positions=message.position,
                canonical_joint_names=self._joint_names,
            )
        except (TypeError, ValueError) as error:
            self._record_problem(f"invalid joint goal: {error}")
            self.get_logger().warning(f"Joint goal rejected: {error}")
            return

        with self._state_lock:
            state = self._motion_execution.state
            if (
                self._active_action_goal is not None
                or self._action_goal_reserved
                or state == MotionExecutionState.EXECUTING
            ):
                detail = "joint goal rejected while an execution is active"
            elif state in (MotionExecutionState.HOLDING, MotionExecutionState.FAULT):
                detail = f"joint goal rejected; call reset after {state.value}"
            else:
                self._pending_joint_goal = target
                self._last_problem = ""
                self._last_planning_detail = "joint goal queued for fresh measured state"
                return
        self._record_problem(detail)
        self.get_logger().warning(detail)

    def _fresh_measured_joint_positions(
        self, now_monotonic_s: float
    ) -> Tuple[Optional[JointPositions], Optional[float]]:
        with self._state_lock:
            positions = self._measured_joint_positions
            received_at_s = self._measured_received_at_s
        if positions is None or received_at_s is None:
            return None, None
        age_s = max(0.0, now_monotonic_s - received_at_s)
        if age_s > self._motion_execution.settings.measured_state_max_age_s:
            return None, age_s
        return positions, age_s

    def _timer_callback(self) -> None:
        now_monotonic_s = time.monotonic()
        actual_positions, feedback_age_s = self._fresh_measured_joint_positions(
            now_monotonic_s
        )
        self._start_pending_joint_goal(actual_positions, now_monotonic_s)

        event = None
        if self._motion_execution.state == MotionExecutionState.EXECUTING:
            if actual_positions is None:
                detail = "fresh measured feedback lost during execution"
                self._motion_execution.fault(
                    MotionExecutionFailureReason.EXTERNAL_FAULT,
                    detail=detail,
                )
                self._request_driver_safe_stop()
                self._action_completion.set()
            else:
                try:
                    event = self._motion_execution.tick(
                        now_monotonic_s=now_monotonic_s,
                        actual_positions=actual_positions,
                    )
                except Exception as error:  # noqa: BLE001 - surface executor faults to ROS.
                    detail = f"motion-execution timer fault: {error}"
                    self._record_problem(detail)
                    self._motion_execution.fault(
                        MotionExecutionFailureReason.EXTERNAL_FAULT,
                        detail=detail,
                        hold_position=actual_positions,
                    )
                    self._request_driver_safe_stop()
                    event = self._motion_execution.tick(
                        now_monotonic_s=now_monotonic_s,
                        actual_positions=actual_positions,
                    )

        if event is not None:
            with self._state_lock:
                self._last_event = event
            if event.desired_setpoint is not None:
                self._publish_setpoint(event.desired_setpoint)
            self._publish_action_feedback(event, actual_positions)
            if event.state != MotionExecutionState.EXECUTING:
                self._action_completion.set()
        elif self._motion_execution.state != MotionExecutionState.EXECUTING:
            self._action_completion.set()

        self._publish_diagnostics(
            event,
            feedback_age_s,
            measured_state_fresh=actual_positions is not None,
        )

    def _start_pending_joint_goal(
        self,
        actual_positions: Optional[JointPositions],
        now_monotonic_s: float,
    ) -> None:
        if actual_positions is None:
            return
        with self._state_lock:
            target = self._pending_joint_goal
            action_active = (
                self._active_action_goal is not None or self._action_goal_reserved
            )
        if target is None or action_active:
            return

        state = self._motion_execution.state
        if state == MotionExecutionState.EXECUTING:
            return
        if state in (MotionExecutionState.HOLDING, MotionExecutionState.FAULT):
            with self._state_lock:
                self._pending_joint_goal = None
            self._record_problem(
                f"queued joint goal rejected; call reset after {state.value}"
            )
            return
        if state in (MotionExecutionState.SUCCEEDED, MotionExecutionState.CANCELED):
            reset = self._motion_execution.reset()
            if not reset.accepted:
                self._record_problem(f"unable to reset completed executor: {reset.detail}")
                return

        planning_result = self._joint_trajectory_planner.plan_joint_motion(
            q_start=actual_positions,
            q_goal=target,
        )
        with self._state_lock:
            self._pending_joint_goal = None
        if not planning_result.succeeded or planning_result.trajectory is None:
            detail = f"joint goal planning failed: {planning_result.detail}"
            self._record_problem(detail)
            self.get_logger().warning(detail)
            return

        started, detail = self._start_trajectory(
            trajectory=planning_result.trajectory,
            actual_positions=actual_positions,
            now_monotonic_s=now_monotonic_s,
            origin="joint_goal",
        )
        if not started:
            self._record_problem(f"joint goal execution rejected: {detail}")
            return
        with self._state_lock:
            self._last_planning_detail = (
                "minimum-jerk plan duration {} s{}".format(
                    self._format_number(planning_result.resolved_duration_s),
                    " (stretched to limits)" if planning_result.duration_adjusted else "",
                )
            )

    def _start_trajectory(
        self,
        trajectory: JointTrajectory,
        actual_positions: JointPositions,
        now_monotonic_s: float,
        origin: str,
    ) -> Tuple[bool, str]:
        """Start only from current feedback; never synthesize a jump to q0."""
        with self._state_lock:
            safety_state = self._driver_safety_state
        if safety_state != "armed":
            return False, f"driver safety gate is {safety_state}; operator re-arm is required"
        start_error_rad = self._maximum_position_error(
            trajectory.points[0].positions, actual_positions
        )
        tolerance_rad = self._motion_execution.settings.start_tolerance_rad
        if start_error_rad > tolerance_rad:
            return (
                False,
                f"trajectory q0 differs from fresh measured q by {self._format_number(start_error_rad)} rad (limit {self._format_number(tolerance_rad)} rad)",
            )

        state = self._motion_execution.state
        if state in (MotionExecutionState.SUCCEEDED, MotionExecutionState.CANCELED):
            reset = self._motion_execution.reset()
            if not reset.accepted:
                return False, f"unable to reset terminal executor: {reset.detail}"
            state = self._motion_execution.state
        if state != MotionExecutionState.IDLE:
            return False, f"executor is {state.value}"

        result = self._motion_execution.start(
            trajectory=trajectory,
            now_monotonic_s=now_monotonic_s,
        )
        if not result.accepted:
            return False, result.detail
        with self._state_lock:
            self._active_origin = origin
            self._last_problem = ""
            self._action_completion.clear()
        self._publish_preview(trajectory)
        return True, result.detail

    def _follow_joint_trajectory_goal_callback(self, goal_request):
        try:
            self._trajectory_from_ros(goal_request.trajectory)
        except (TypeError, ValueError) as error:
            self.get_logger().warning(f"FollowJointTrajectory goal rejected: {error}")
            return GoalResponse.REJECT

        with self._state_lock:
            state = self._motion_execution.state
            busy = (
                self._active_action_goal is not None
                or self._action_goal_reserved
                or self._pending_joint_goal is not None
                or state == MotionExecutionState.EXECUTING
            )
            blocked = state in (MotionExecutionState.HOLDING, MotionExecutionState.FAULT)
            if not busy and not blocked:
                self._action_goal_reserved = True
                return GoalResponse.ACCEPT
        reason = "executor is busy" if busy else f"call reset after {state.value}"
        self.get_logger().warning(f"FollowJointTrajectory goal rejected: {reason}")
        return GoalResponse.REJECT

    def _follow_joint_trajectory_cancel_callback(self, goal_handle):
        with self._state_lock:
            is_active = goal_handle is self._active_action_goal
        return CancelResponse.ACCEPT if is_active else CancelResponse.REJECT

    def _execute_follow_joint_trajectory(self, goal_handle):
        """Run one action without polling/sleeping in the ROS timer thread.

        The node's main entry point uses a multi-threaded executor.  This
        callback therefore waits on an event while the independent 5 Hz timer
        advances the pure executor and publishes setpoints.  In particular it
        does not use ``time.sleep`` or run a second timing loop.
        """
        result = FollowJointTrajectory.Result()
        with self._state_lock:
            self._action_goal_reserved = False
            self._active_action_goal = goal_handle
            self._action_completion.clear()
        try:
            trajectory = self._trajectory_from_ros(goal_handle.request.trajectory)
            now_monotonic_s = time.monotonic()
            actual_positions, _ = self._fresh_measured_joint_positions(
                now_monotonic_s
            )
            if actual_positions is None:
                return self._abort_action(
                    goal_handle,
                    result,
                    "fresh measured joint state is required before execution",
                    "INVALID_GOAL",
                )
            started, detail = self._start_trajectory(
                trajectory=trajectory,
                actual_positions=actual_positions,
                now_monotonic_s=now_monotonic_s,
                origin="follow_joint_trajectory",
            )
            if not started:
                return self._abort_action(goal_handle, result, detail, "INVALID_GOAL")

            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    current, _ = self._fresh_measured_joint_positions(time.monotonic())
                    self._motion_execution.cancel(hold_position=current)
                    self._request_driver_safe_stop()
                    self._action_completion.set()
                if self._action_completion.wait(timeout=0.05):
                    break
                if self._motion_execution.state != MotionExecutionState.EXECUTING:
                    break
            return self._complete_action(goal_handle, result)
        except (TypeError, ValueError) as error:
            return self._abort_action(goal_handle, result, str(error), "INVALID_GOAL")
        except Exception as error:  # noqa: BLE001 - return an action result, not a dead task.
            detail = f"motion-execution action failure: {error}"
            self._record_problem(detail)
            self._motion_execution.fault(
                MotionExecutionFailureReason.EXTERNAL_FAULT,
                detail=detail,
            )
            return self._abort_action(goal_handle, result, detail, "INVALID_GOAL")
        finally:
            with self._state_lock:
                if goal_handle is self._active_action_goal:
                    self._active_action_goal = None
                    self._active_origin = ""
                self._action_goal_reserved = False

    def _follow_cartesian_trajectory_goal_callback(self, goal_request):
        """Reserve the shared executor only for a syntactically valid TCP goal."""
        if not self._cartesian_execution_enabled or self._cartesian_planner is None:
            return GoalResponse.REJECT
        try:
            self._frame_name(
                goal_request.target_pose.header.frame_id,
                "target_pose.header.frame_id",
            )
            self._sdk_pose_from_message(goal_request.target_pose)
            self._cartesian_policy_from_goal(goal_request)
        except (TypeError, ValueError) as error:
            self.get_logger().warning(
                f"FollowCartesianTrajectory goal rejected: {error}"
            )
            return GoalResponse.REJECT
        with self._state_lock:
            state = self._motion_execution.state
            busy = (
                self._active_action_goal is not None
                or self._action_goal_reserved
                or self._pending_joint_goal is not None
                or state == MotionExecutionState.EXECUTING
            )
            blocked = state in (MotionExecutionState.HOLDING, MotionExecutionState.FAULT)
            if not busy and not blocked:
                self._action_goal_reserved = True
                return GoalResponse.ACCEPT
        reason = "executor is busy" if busy else f"call reset after {state.value}"
        self.get_logger().warning(
            f"FollowCartesianTrajectory goal rejected: {reason}"
        )
        return GoalResponse.REJECT

    def _follow_cartesian_trajectory_cancel_callback(self, goal_handle):
        with self._state_lock:
            is_active = goal_handle is self._active_action_goal
        return CancelResponse.ACCEPT if is_active else CancelResponse.REJECT

    def _execute_follow_cartesian_trajectory(self, goal_handle):
        """Plan one TCP goal then execute it through the existing FSM only."""
        result = FollowCartesianTrajectory.Result()
        with self._state_lock:
            self._action_goal_reserved = False
            self._active_action_goal = goal_handle
            self._active_cartesian_waypoint_count = 0
            self._action_completion.clear()
        try:
            self._publish_cartesian_action_feedback(goal_handle, 0, 0, 0.0, "validating")
            if goal_handle.is_cancel_requested:
                return self._cancel_cartesian_action(goal_handle, result)
            now_monotonic_s = time.monotonic()
            q_start, _ = self._fresh_measured_joint_positions(now_monotonic_s)
            if q_start is None:
                return self._abort_cartesian_action(
                    goal_handle,
                    result,
                    "stale_measured_state",
                    "fresh measured joint state is required before Cartesian execution",
                )
            target_pose = self._cartesian_target_pose_from_goal(
                goal_handle.request.target_pose
            )
            policy = self._cartesian_policy_from_goal(goal_handle.request)
            self._publish_cartesian_action_feedback(goal_handle, 0, 0, 0.0, "planning")
            planning_result = self._cartesian_planner.plan_cartesian_motion(
                q_start=q_start,
                target_pose=target_pose,
                policy=policy,
            )
            self._copy_cartesian_result(result, planning_result)
            if not planning_result.succeeded or planning_result.trajectory is None:
                return self._abort_cartesian_action(
                    goal_handle,
                    result,
                    self._cartesian_failure_reason(planning_result),
                    str(planning_result.detail),
                    keep_result_fields=True,
                )
            if goal_handle.is_cancel_requested:
                return self._cancel_cartesian_action(goal_handle, result)

            # The planner was intentionally pure. Re-read feedback immediately
            # before start so a delayed plan cannot introduce a q0 jump.
            actual_positions, _ = self._fresh_measured_joint_positions(
                time.monotonic()
            )
            if actual_positions is None:
                return self._abort_cartesian_action(
                    goal_handle,
                    result,
                    "stale_measured_state",
                    "measured joint state became stale during Cartesian planning",
                )
            with self._state_lock:
                self._active_cartesian_waypoint_count = planning_result.waypoint_count
                self._last_planning_detail = str(planning_result.detail)
            started, detail = self._start_trajectory(
                trajectory=planning_result.trajectory,
                actual_positions=actual_positions,
                now_monotonic_s=time.monotonic(),
                origin="follow_cartesian_trajectory",
            )
            if not started:
                return self._abort_cartesian_action(
                    goal_handle,
                    result,
                    "execution_preflight_failed",
                    detail,
                )
            self._publish_cartesian_reference_path(planning_result)
            self._publish_cartesian_action_feedback(
                goal_handle,
                0,
                planning_result.waypoint_count,
                0.0,
                "executing",
            )
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    current, _ = self._fresh_measured_joint_positions(time.monotonic())
                    self._motion_execution.cancel(hold_position=current)
                    self._request_driver_safe_stop()
                    self._action_completion.set()
                if self._action_completion.wait(timeout=0.05):
                    break
                if self._motion_execution.state != MotionExecutionState.EXECUTING:
                    break
            return self._complete_cartesian_action(goal_handle, result)
        except TransformException as error:
            return self._abort_cartesian_action(
                goal_handle, result, "frame_transform_failed", str(error)
            )
        except (TypeError, ValueError) as error:
            return self._abort_cartesian_action(
                goal_handle, result, "invalid_goal", str(error)
            )
        except Exception as error:  # noqa: BLE001 - never leave executor ownership unclear.
            detail = f"Cartesian execution action failure: {error}"
            self._motion_execution.fault(
                MotionExecutionFailureReason.EXTERNAL_FAULT,
                detail=detail,
            )
            self._request_driver_safe_stop()
            return self._abort_cartesian_action(
                goal_handle, result, "execution_backend_error", detail
            )
        finally:
            with self._state_lock:
                if goal_handle is self._active_action_goal:
                    self._active_action_goal = None
                    self._active_origin = ""
                self._active_cartesian_waypoint_count = 0
                self._action_goal_reserved = False

    def _cartesian_target_pose_from_goal(self, source: PoseStamped) -> Pose:
        source_frame = self._frame_name(
            source.header.frame_id, "target pose frame"
        )
        if source_frame == self._cartesian_base_frame:
            return self._sdk_pose_from_message(source)
        if self._cartesian_tf_buffer is None:
            raise RuntimeError("Cartesian TF buffer is unavailable")
        target_time = Time()
        if source.header.stamp.sec != 0 or source.header.stamp.nanosec != 0:
            target_time = Time.from_msg(source.header.stamp)
        transform = self._cartesian_tf_buffer.lookup_transform(
            self._cartesian_base_frame,
            source_frame,
            target_time,
            timeout=Duration(seconds=0.25),
        )
        transformed = do_transform_pose_stamped(source, transform)
        transformed.header.frame_id = self._cartesian_base_frame
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

    def _cartesian_policy_from_goal(self, goal_request) -> CartesianTrajectoryPolicy:
        if self._cartesian_planner is None:
            raise RuntimeError("Cartesian planner is unavailable")
        default = self._cartesian_planner.default_policy
        if goal_request.path_mode == FollowCartesianTrajectory.Goal.PATH_DEFAULT:
            path_mode = default.path_mode
        else:
            path_mode = {
                FollowCartesianTrajectory.Goal.PATH_LINEAR_TRANSLATION_SLERP: (
                    CartesianPathMode.LINEAR_TRANSLATION_SLERP
                ),
                FollowCartesianTrajectory.Goal.PATH_SE3_GEODESIC: (
                    CartesianPathMode.SE3_GEODESIC
                ),
            }.get(goal_request.path_mode)
            if path_mode is None:
                raise ValueError(f"unsupported Cartesian path_mode: {goal_request.path_mode}")
        if goal_request.task_mode == FollowCartesianTrajectory.Goal.TASK_DEFAULT:
            task_mode = default.ik_policy.task_mode
        else:
            task_mode = {
                FollowCartesianTrajectory.Goal.TASK_FULL_POSE: IKTaskMode.FULL_POSE,
                FollowCartesianTrajectory.Goal.TASK_POSITION_ONLY: (
                    IKTaskMode.POSITION_ONLY
                ),
            }.get(goal_request.task_mode)
            if task_mode is None:
                raise ValueError(f"unsupported IK task_mode: {goal_request.task_mode}")
        requested_duration_s = self._duration_to_seconds(goal_request.requested_duration)
        requested_duration_s = requested_duration_s if requested_duration_s > 0.0 else None
        if goal_request.time_scaling_mode == FollowCartesianTrajectory.Goal.TIME_DEFAULT:
            if requested_duration_s is not None or float(goal_request.speed_scale) != 0.0:
                raise ValueError(
                    "TIME_DEFAULT requires zero requested_duration and speed_scale"
                )
            time_scaling = default.time_scaling
        else:
            time_mode = {
                FollowCartesianTrajectory.Goal.TIME_AUTO_LIMITED: TimeScalingMode.AUTO_LIMITED,
                FollowCartesianTrajectory.Goal.TIME_REQUESTED_DURATION_STRETCH: (
                    TimeScalingMode.REQUESTED_DURATION_STRETCH
                ),
                FollowCartesianTrajectory.Goal.TIME_REQUESTED_DURATION_STRICT: (
                    TimeScalingMode.REQUESTED_DURATION_STRICT
                ),
                FollowCartesianTrajectory.Goal.TIME_SPEED_SCALE: TimeScalingMode.SPEED_SCALE,
            }.get(goal_request.time_scaling_mode)
            if time_mode is None:
                raise ValueError(
                    f"unsupported time_scaling_mode: {goal_request.time_scaling_mode}"
                )
            if time_mode in (
                TimeScalingMode.REQUESTED_DURATION_STRETCH,
                TimeScalingMode.REQUESTED_DURATION_STRICT,
            ) and requested_duration_s is None:
                raise ValueError("requested duration mode requires requested_duration > 0")
            if time_mode in (TimeScalingMode.AUTO_LIMITED, TimeScalingMode.SPEED_SCALE):
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
        return replace(
            default,
            path_mode=path_mode,
            ik_policy=replace(default.ik_policy, task_mode=task_mode),
            time_scaling=time_scaling,
            max_translation_step_m=self._optional_positive_override(
                goal_request.max_translation_step_m,
                default.max_translation_step_m,
                "max_translation_step_m",
            ),
            max_rotation_step_rad=self._optional_positive_override(
                goal_request.max_rotation_step_rad,
                default.max_rotation_step_rad,
                "max_rotation_step_rad",
            ),
        )

    @staticmethod
    def _optional_positive_override(value: Any, default: float, name: str) -> float:
        number = float(value)
        if not math.isfinite(number):
            raise ValueError(f"{name} must be finite")
        return default if number <= 0.0 else number

    def _copy_cartesian_result(self, message, planning_result) -> None:
        message.succeeded = bool(planning_result.succeeded)
        message.trajectory = (
            self._trajectory_to_ros(planning_result.trajectory)
            if planning_result.trajectory is not None
            else RosJointTrajectory()
        )
        message.failure_reason = self._cartesian_failure_reason(planning_result)
        message.detail = str(planning_result.detail)
        failed_index = planning_result.failed_waypoint_index
        message.failed_waypoint_index = (
            (1 << 32) - 1 if failed_index is None else max(0, int(failed_index))
        )
        message.resolved_duration_s = self._result_number(
            planning_result.resolved_duration_s
        )
        message.minimum_joint_limit_margin_rad = self._result_number(
            planning_result.minimum_joint_limit_margin_rad
        )
        message.minimum_singular_value = self._result_number(
            planning_result.minimum_singular_value
        )
        message.maximum_position_residual_m = self._result_number(
            planning_result.maximum_position_residual_m
        )
        message.maximum_orientation_residual_rad = self._result_number(
            planning_result.maximum_orientation_residual_rad
        )

    @staticmethod
    def _cartesian_failure_reason(planning_result) -> str:
        reason = planning_result.failure_reason
        return reason.value if reason is not None else ""

    @staticmethod
    def _result_number(value: Any) -> float:
        try:
            number = float(value)
        except (TypeError, ValueError):
            return 0.0
        return number if math.isfinite(number) else 0.0

    def _publish_cartesian_reference_path(self, planning_result) -> None:
        if self._cartesian_reference_path_publisher is None:
            return
        message = RosPath()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._cartesian_base_frame
        for pose in planning_result.reference_path:
            pose_message = PoseStamped()
            pose_message.header = message.header
            (
                pose_message.pose.position.x,
                pose_message.pose.position.y,
                pose_message.pose.position.z,
            ) = pose.position
            (
                pose_message.pose.orientation.x,
                pose_message.pose.orientation.y,
                pose_message.pose.orientation.z,
                pose_message.pose.orientation.w,
            ) = pose.orientation
            message.poses.append(pose_message)
        self._cartesian_reference_path_publisher.publish(message)

    def _publish_cartesian_action_feedback(
        self,
        goal_handle,
        waypoint_index: int,
        waypoint_count: int,
        progress: float,
        stage: str,
    ) -> None:
        feedback = FollowCartesianTrajectory.Feedback()
        feedback.waypoint_index = max(0, int(waypoint_index))
        feedback.waypoint_count = max(0, int(waypoint_count))
        feedback.progress = max(0.0, min(1.0, float(progress)))
        feedback.stage = stage
        try:
            goal_handle.publish_feedback(feedback)
        except Exception as error:  # noqa: BLE001 - action may complete concurrently.
            self.get_logger().warning(
                f"Unable to publish Cartesian action feedback: {error}"
            )

    def _abort_cartesian_action(
        self,
        goal_handle,
        result,
        failure_reason: str,
        detail: str,
        *,
        keep_result_fields: bool = False,
    ):
        if not keep_result_fields:
            result.trajectory = RosJointTrajectory()
            result.failed_waypoint_index = (1 << 32) - 1
        result.succeeded = False
        result.failure_reason = failure_reason
        result.detail = detail
        self._record_problem(detail)
        goal_handle.abort()
        return result

    def _cancel_cartesian_action(self, goal_handle, result):
        result.succeeded = False
        result.trajectory = RosJointTrajectory()
        result.failure_reason = "canceled"
        result.detail = "FollowCartesianTrajectory goal canceled"
        result.failed_waypoint_index = (1 << 32) - 1
        goal_handle.canceled()
        return result

    def _complete_cartesian_action(self, goal_handle, result):
        state = self._motion_execution.state
        with self._state_lock:
            event = self._last_event
        detail = event.detail if event is not None else ""
        if state == MotionExecutionState.SUCCEEDED:
            result.succeeded = True
            result.failure_reason = ""
            result.detail = detail or "Cartesian trajectory execution succeeded"
            goal_handle.succeed()
            return result
        result.succeeded = False
        result.trajectory = RosJointTrajectory()
        result.failed_waypoint_index = (1 << 32) - 1
        result.detail = detail or "Cartesian trajectory execution did not complete"
        if state == MotionExecutionState.CANCELED:
            result.failure_reason = "canceled"
            goal_handle.canceled()
        else:
            result.failure_reason = "execution_{}".format(state.value)
            goal_handle.abort()
        return result

    def _trajectory_from_ros(self, message: RosJointTrajectory) -> JointTrajectory:
        """Validate a full canonical action trajectory before it reaches pycore."""
        if message.header.stamp.sec != 0 or message.header.stamp.nanosec != 0:
            raise ValueError(
                "FollowJointTrajectory header.stamp must be zero; scheduled starts are unsupported"
            )
        names = tuple(message.joint_names)
        if len(names) != len(self._joint_names) or set(names) != set(self._joint_names):
            raise ValueError(
                "trajectory joint_names must contain exactly the six canonical arm joints"
            )
        if len(set(names)) != len(names):
            raise ValueError("trajectory joint_names contains duplicates")
        if not message.points:
            raise ValueError("trajectory must contain at least one point")

        points = []
        for index, point in enumerate(message.points):
            positions = self._reordered_joint_vector(
                names, point.positions, f"point {index} positions"
            )
            velocities = self._reordered_joint_vector(
                names, point.velocities, f"point {index} velocities"
            )
            accelerations = self._reordered_joint_vector(
                names, point.accelerations, f"point {index} accelerations"
            )
            points.append(
                TrajectoryPoint(
                    positions=positions,
                    velocities=velocities,
                    accelerations=accelerations,
                    time_from_start_s=self._duration_to_seconds(point.time_from_start),
                )
            )
        trajectory = JointTrajectory(self._joint_names, points)
        violations = self._joint_trajectory_planner.motion_limits.trajectory_violations(
            trajectory, require_derivatives=True
        )
        if violations:
            raise ValueError("trajectory violates motion limits: {}".format("; ".join(violations)))
        return trajectory

    def _reordered_joint_vector(
        self, names: Sequence[str], values: Sequence[float], label: str
    ) -> JointPositions:
        if len(values) != len(names):
            raise ValueError(f"{label} must contain exactly six values")
        values_by_name = dict(zip(names, values))
        return JointPositions(tuple(values_by_name[name] for name in self._joint_names))

    @staticmethod
    def _duration_to_seconds(duration) -> float:
        seconds = duration.sec
        nanoseconds = duration.nanosec
        if not isinstance(seconds, int) or not isinstance(nanoseconds, int):
            raise TypeError("trajectory time_from_start must use integer seconds")
        if nanoseconds < 0 or nanoseconds >= 1000000000:
            raise ValueError("trajectory time_from_start.nanosec is out of range")
        value = float(seconds) + float(nanoseconds) * 1e-9
        if not math.isfinite(value) or value < 0.0:
            raise ValueError("trajectory time_from_start must be non-negative")
        return value

    def _publish_setpoint(self, setpoint) -> None:
        """Publish one epoch-bound private driver setpoint."""
        with self._state_lock:
            safety_epoch = self._driver_safety_epoch
        message = DriverJointSetpoint()
        message.header.stamp = self.get_clock().now().to_msg()
        message.safety_epoch = safety_epoch
        message.joint_names = list(self._joint_names)
        message.positions = list(setpoint.positions.values)
        message.speed_scale = 0.5
        self._setpoint_publisher.publish(message)

    def _request_driver_safe_stop(self) -> None:
        """Ask the driver, the serial owner, to close its safety gate and stop."""
        if self._driver_stop_client.service_is_ready():
            self._driver_stop_client.call_async(Trigger.Request())

    def _publish_preview(self, trajectory: JointTrajectory) -> None:
        self._preview_publisher.publish(self._trajectory_to_ros(trajectory, stamp_now=True))

    def _trajectory_to_ros(
        self, trajectory: JointTrajectory, *, stamp_now: bool = False
    ) -> RosJointTrajectory:
        """Preserve derivative-complete trajectories for action and preview users."""
        message = RosJointTrajectory()
        if stamp_now:
            message.header.stamp = self.get_clock().now().to_msg()
        message.joint_names = list(trajectory.joint_names)
        ros_points = []
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
            ros_points.append(ros_point)
        message.points = ros_points
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

    def _publish_action_feedback(self, event, actual_positions: Optional[JointPositions]) -> None:
        with self._state_lock:
            goal_handle = self._active_action_goal
            active_origin = self._active_origin
            waypoint_count = self._active_cartesian_waypoint_count
        if goal_handle is None or event.desired_setpoint is None:
            return
        if active_origin == "follow_cartesian_trajectory":
            self._publish_cartesian_action_feedback(
                goal_handle,
                waypoint_count,
                waypoint_count,
                event.progress,
                "executing" if event.state == MotionExecutionState.EXECUTING else event.state.value,
            )
            return
        if active_origin != "follow_joint_trajectory" or actual_positions is None:
            return
        feedback = FollowJointTrajectory.Feedback()
        feedback.header.stamp = self.get_clock().now().to_msg()
        feedback.joint_names = list(self._joint_names)
        feedback.desired.positions = list(event.desired_setpoint.positions.values)
        feedback.desired.velocities = list(event.desired_setpoint.velocities.values)
        feedback.desired.accelerations = list(
            event.desired_setpoint.accelerations.values
        )
        feedback.actual.positions = list(actual_positions.values)
        feedback.error.positions = [
            desired - actual
            for desired, actual in zip(
                event.desired_setpoint.positions.values, actual_positions.values
            )
        ]
        try:
            goal_handle.publish_feedback(feedback)
        except Exception as error:  # noqa: BLE001 - action may complete concurrently.
            self.get_logger().warning(f"Unable to publish action feedback: {error}")

    def _publish_diagnostics(
        self,
        event,
        feedback_age_s: Optional[float],
        measured_state_fresh: bool,
    ) -> None:
        state = self._motion_execution.state
        with self._state_lock:
            pending_goal = self._pending_joint_goal is not None
            active_origin = self._active_origin
            problem = self._last_problem
            planning_detail = self._last_planning_detail
        level, summary = self._diagnostic_level_and_summary(
            state=state,
            measured_state_fresh=measured_state_fresh,
            problem=problem,
        )
        values = [
            KeyValue(key="state", value=state.value),
            KeyValue(key="active_origin", value=active_origin),
            KeyValue(key="pending_joint_goal", value=str(pending_goal)),
            KeyValue(
                key="measured_state_fresh",
                value=str(measured_state_fresh),
            ),
            KeyValue(key="measured_state_age_s", value=self._format_number(feedback_age_s)),
            KeyValue(key="planner", value=planning_detail),
            KeyValue(key="detail", value=problem or (event.detail if event else "")),
        ]
        if event is not None:
            values.extend([
                KeyValue(key="progress", value=self._format_number(event.progress)),
                KeyValue(key="elapsed_s", value=self._format_number(event.elapsed_s)),
                KeyValue(key="duration_s", value=self._format_number(event.duration_s)),
                KeyValue(
                    key="max_tracking_error_rad",
                    value=self._format_number(event.max_tracking_error_rad),
                ),
                KeyValue(key="tick_lag_s", value=self._format_number(event.tick_lag_s)),
                KeyValue(key="lagged", value=str(event.lagged)),
                KeyValue(key="timed_out", value=str(event.timed_out)),
                KeyValue(
                    key="reason",
                    value=event.reason.value if event.reason is not None else "",
                ),
            ])
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.level = level
        status.name = "myarm/motion_execution"
        status.hardware_id = "myarm_m750"
        status.message = summary
        status.values = values
        message.status = [status]
        self._diagnostics_publisher.publish(message)

    @staticmethod
    def _diagnostic_level_and_summary(
        state: MotionExecutionState,
        measured_state_fresh: bool,
        problem: str,
    ) -> Tuple[int, str]:
        if state == MotionExecutionState.FAULT:
            return DiagnosticStatus.ERROR, "motion_execution_fault"
        if problem:
            return DiagnosticStatus.ERROR, "motion_execution_error"
        if state == MotionExecutionState.HOLDING:
            return DiagnosticStatus.WARN, "motion_execution_holding"
        if state == MotionExecutionState.CANCELED:
            return DiagnosticStatus.WARN, "motion_execution_canceled"
        if not measured_state_fresh:
            return DiagnosticStatus.WARN, "waiting_for_fresh_measured_state"
        if state == MotionExecutionState.EXECUTING:
            return DiagnosticStatus.OK, "motion_execution_active"
        if state == MotionExecutionState.SUCCEEDED:
            return DiagnosticStatus.OK, "motion_execution_succeeded"
        return DiagnosticStatus.OK, "motion_execution_idle"

    def _cancel_service_callback(self, request, response):
        del request
        actual_positions, _ = self._fresh_measured_joint_positions(time.monotonic())
        result = self._motion_execution.cancel(hold_position=actual_positions)
        response.success = result.accepted
        response.message = result.detail
        if result.accepted:
            self._request_driver_safe_stop()
            self._action_completion.set()
        else:
            self._record_problem(f"cancel rejected: {result.detail}")
        return response

    def _reset_service_callback(self, request, response):
        del request
        with self._state_lock:
            action_active = self._active_action_goal is not None
        if action_active:
            response.success = False
            response.message = "cannot reset while a motion action is active"
            return response
        result = self._motion_execution.reset()
        response.success = result.accepted
        response.message = result.detail
        if result.accepted:
            with self._state_lock:
                self._last_problem = ""
                self._last_planning_detail = ""
                self._active_origin = ""
        else:
            self._record_problem(f"reset rejected: {result.detail}")
        return response

    def _complete_action(self, goal_handle, result):
        state = self._motion_execution.state
        with self._state_lock:
            event = self._last_event
        detail = event.detail if event is not None else ""
        if state == MotionExecutionState.SUCCEEDED:
            goal_handle.succeed()
            result.error_code = self._action_result_code("SUCCESSFUL", 0)
            result.error_string = detail or "trajectory execution succeeded"
            return result
        if state == MotionExecutionState.CANCELED:
            goal_handle.canceled()
            result.error_code = self._action_result_code("SUCCESSFUL", 0)
            result.error_string = detail or "trajectory execution canceled"
            return result
        goal_handle.abort()
        result.error_code = self._action_result_code("GOAL_TOLERANCE_VIOLATED", -5)
        result.error_string = detail or "trajectory execution did not complete"
        return result

    def _abort_action(self, goal_handle, result, detail: str, code_name: str):
        self._record_problem(detail)
        goal_handle.abort()
        result.error_code = self._action_result_code(code_name, -1)
        result.error_string = detail
        return result

    @staticmethod
    def _action_result_code(name: str, fallback: int) -> int:
        return getattr(FollowJointTrajectory.Result, name, fallback)

    @staticmethod
    def _maximum_position_error(
        desired_positions: JointPositions, actual_positions: JointPositions
    ) -> float:
        return max(
            abs(desired - actual)
            for desired, actual in zip(desired_positions.values, actual_positions.values)
        )

    def _record_problem(self, detail: str) -> None:
        with self._state_lock:
            self._last_problem = detail

    @staticmethod
    def _format_number(value: Any) -> str:
        if value is None:
            return ""
        number = float(value)
        return "nan" if not math.isfinite(number) else f"{number:.9g}"

    def destroy_node(self):
        """Unblock any action worker; no robot transport is owned here."""
        self._action_completion.set()
        if self._motion_execution.state == MotionExecutionState.EXECUTING:
            self._motion_execution.cancel()
            self._request_driver_safe_stop()
        self._action_server.destroy()
        if self._cartesian_action_server is not None:
            self._cartesian_action_server.destroy()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = MyArmMotionExecutionNode()
        # Motion action callbacks wait on a completion Event while the timer
        # progresses execution, so they must not share a single worker with
        # that timer. Three workers also leave one for cancel requests and
        # measured-state callbacks.
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if executor is not None:
            executor.shutdown()
        rclpy.shutdown()
