"""One-shot, fake-only NeuGrasp trial coordinator.

This node intentionally owns the *application sequence* only.  It never
opens a camera, runs a model, publishes driver setpoints, or recreates the
scan implementation.  A single worker executes one bounded fake trial:

``READY -> INIT_HOME -> SCAN -> PREDICT_ARTIFACT -> SELECT_PREFLIGHT ->
PREGRASP -> GRASP -> CLOSE -> LIFT -> COMPLETE``.

The model-volume tensors are always interpreted in the current
``neugrasp_volume`` frame and are transformed into ``base_link`` using the
live TF tree.  This is deliberately different from the legacy run artifacts,
which can contain historical calibration/base relations.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Optional, Sequence, Tuple

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, Pose, PoseStamped
from myarm_interfaces.action import ScanWorkspace
from myarm_sdk.core import JointPositions, Pose as SdkPose, load_sdk_yaml
from myarm_sdk.service import KinematicsService, JointTrajectoryPlannerService
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker

from .artifact_pipeline import (
    ArtifactBundle,
    ArtifactCandidate,
    ArtifactSettings,
    GRIPPER_EDGES,
    gripper_wireframe_points,
    load_artifacts,
)
from .artifact_visualization import ArtifactVisualizationPublisher
from .math3d import RigidTransform, compose, normalize_quaternion, rotate_vector


_SNAPSHOT_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
_IDENTITY = (0.0, 0.0, 0.0, 1.0)


class TrialFailure(RuntimeError):
    """An expected fail-closed trial error, safe to show in status/logs."""


@dataclass(frozen=True)
class TrialFrames:
    base: str
    volume: str
    tool: str


@dataclass(frozen=True)
class PrimitivePolicy:
    pregrasp_distance_m: float
    lift_distance_m: float
    top_down_max_angle_rad: float


@dataclass(frozen=True)
class ExecutionPolicy:
    gripper_opening_m: float
    gripper_closed_m: float
    readiness_timeout_s: float
    action_timeout_s: float
    gripper_timeout_s: float
    gripper_tolerance_m: float
    measured_joint_max_age_s: float


@dataclass(frozen=True)
class TimingPolicy:
    scan_view_settle_s: float
    after_phase_s: Mapping[str, float]


@dataclass(frozen=True)
class VisualizationPolicy:
    tsdf_z_index_range: Tuple[int, int]


@dataclass(frozen=True)
class TrialConfig:
    frames: TrialFrames
    grasp_to_tool: RigidTransform
    primitive: PrimitivePolicy
    execution: ExecutionPolicy
    timing: TimingPolicy
    visualization: VisualizationPolicy
    artifacts: ArtifactSettings


@dataclass(frozen=True)
class CandidateTargets:
    candidate: ArtifactCandidate
    # Motion/IK targets are tool0 poses.  The wireframe geometry below has a
    # different semantic origin: it is authored in the NeuGrasp grasp frame.
    grasp_tool: RigidTransform
    pregrasp_tool: RigidTransform
    lift_tool: RigidTransform
    grasp_frame: RigidTransform
    pregrasp_frame: RigidTransform
    lift_frame: RigidTransform
    preflight_final_joint_positions: JointPositions


@dataclass(frozen=True)
class PrimitiveTargets:
    """Paired NeuGrasp-frame wireframe poses and tool0 motion poses."""

    grasp_tool: RigidTransform
    pregrasp_tool: RigidTransform
    lift_tool: RigidTransform
    grasp_frame: RigidTransform
    pregrasp_frame: RigidTransform
    lift_frame: RigidTransform


class NeugraspTrialNode(Node):
    """Run exactly one fake NeuGrasp primitive and then remain observable.

    A worker thread is intentional: action futures are completed by the ROS
    executor while the worker waits on them.  ``main`` therefore uses a
    :class:`MultiThreadedExecutor`; no callback is blocked by an action wait.
    """

    def __init__(self) -> None:
        super().__init__("neugrasp_trial")
        self.declare_parameter("run_dir", "")
        self.declare_parameter("trial_config", "")
        self.declare_parameter("services_config", "service/config/services.yaml")
        # These are explicitly carried by the dedicated launch so a caller
        # cannot accidentally pair a trial profile with a different scene/TF
        # contract.  Frame definitions remain owned by trial_config.
        self.declare_parameter("scan_config", "")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("volume_frame", "neugrasp_volume")
        self.declare_parameter("tool_frame", "tool0")
        self.declare_parameter("scan_profile_id", "neugrasp_simulation_views_16_19")
        self.declare_parameter("scan_action", "/neugrasp/scan_workspace")
        self.declare_parameter(
            "follow_joint_trajectory_action", "/myarm/follow_joint_trajectory"
        )
        self.declare_parameter("motion_cancel_service", "/myarm/motion_execution/cancel")
        self.declare_parameter("gripper_command_topic", "/myarm/gripper/command")
        self.declare_parameter("gripper_state_topic", "/myarm/gripper/state")
        self.declare_parameter("joint_state_topic", "/myarm/state/joint_state")

        run_dir = str(self.get_parameter("run_dir").value).strip()
        if not run_dir:
            raise ValueError("run_dir is required and must contain inference/*.npy")
        self._run_dir = Path(run_dir).expanduser()
        if not self._run_dir.is_dir():
            raise ValueError(f"run_dir does not exist: {self._run_dir}")
        config_value = str(self.get_parameter("trial_config").value).strip()
        if not config_value:
            raise ValueError("trial_config is required for the fake grasp-to-tool mapping")
        self._config = self._load_trial_config(Path(config_value).expanduser())
        self._validate_launch_frame(
            self.get_parameter("base_frame").value, self._config.frames.base, "base_frame"
        )
        self._validate_launch_frame(
            self.get_parameter("volume_frame").value,
            self._config.frames.volume,
            "volume_frame",
        )
        self._validate_launch_frame(
            self.get_parameter("tool_frame").value, self._config.frames.tool, "tool_frame"
        )
        scan_config = str(self.get_parameter("scan_config").value).strip()
        if scan_config and not Path(scan_config).expanduser().is_file():
            raise ValueError(f"scan_config does not exist: {scan_config}")
        self._scan_profile_id = self._nonempty(
            self.get_parameter("scan_profile_id").value, "scan_profile_id"
        )

        self._state_lock = threading.RLock()
        self._planning_lock = threading.Lock()
        self._latest_joints: Optional[JointPositions] = None
        self._latest_joints_at_s: Optional[float] = None
        self._latest_gripper_opening_m: Optional[float] = None
        self._latest_gripper_at_s: Optional[float] = None
        self._active_action_goal = None
        self._started = False
        self._terminal = False
        self._stop_requested = threading.Event()
        self._worker: Optional[threading.Thread] = None

        services_document = load_sdk_yaml(
            str(self.get_parameter("services_config").value)
        )
        self._configure_services(services_document)

        callback_group = ReentrantCallbackGroup()
        self._scan_client = ActionClient(
            self,
            ScanWorkspace,
            self._nonempty(self.get_parameter("scan_action").value, "scan_action"),
            callback_group=callback_group,
        )
        self._trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            self._nonempty(
                self.get_parameter("follow_joint_trajectory_action").value,
                "follow_joint_trajectory_action",
            ),
            callback_group=callback_group,
        )
        self._motion_cancel_client = self.create_client(
            Trigger,
            self._nonempty(
                self.get_parameter("motion_cancel_service").value,
                "motion_cancel_service",
            ),
            callback_group=callback_group,
        )
        self._gripper_publisher = self.create_publisher(
            Float64,
            self._nonempty(
                self.get_parameter("gripper_command_topic").value,
                "gripper_command_topic",
            ),
            10,
        )
        self.create_subscription(
            JointState,
            self._nonempty(self.get_parameter("joint_state_topic").value, "joint_state_topic"),
            self._joint_state_callback,
            10,
        )
        self.create_subscription(
            JointState,
            self._nonempty(self.get_parameter("gripper_state_topic").value, "gripper_state_topic"),
            self._gripper_state_callback,
            10,
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        self._phase_publisher = self.create_publisher(
            String, "/neugrasp/trial/phase", _SNAPSHOT_QOS
        )
        self._selected_pose_publisher = self.create_publisher(
            PoseStamped, "/neugrasp/selected_grasp", _SNAPSHOT_QOS
        )
        self._selected_marker_publisher = self.create_publisher(
            Marker, "/neugrasp/selected_grasp_marker", _SNAPSHOT_QOS
        )
        self._pregrasp_marker_publisher = self.create_publisher(
            Marker, "/neugrasp/pregrasp_marker", _SNAPSHOT_QOS
        )
        self._lift_marker_publisher = self.create_publisher(
            Marker, "/neugrasp/lift_marker", _SNAPSHOT_QOS
        )
        self._artifact_visualizer = ArtifactVisualizationPublisher(
            self,
            self._config.frames.volume,
            self._config.artifacts,
            tsdf_z_index_range=self._config.visualization.tsdf_z_index_range,
        )
        self._publish_phase("READY")
        # Defer startup a fraction so DDS discovery and TF/RSP can come up.
        self._start_timer = self.create_timer(0.10, self._start_once)
        self.get_logger().info(
            "Configured one fake-only NeuGrasp trial; waiting for joint state, "
            "gripper state, TF and action servers."
        )

    # ----- configuration -------------------------------------------------

    def _configure_services(self, document: Mapping[str, Any]) -> None:
        if not isinstance(document, Mapping):
            raise TypeError("services_config must be a YAML mapping")
        services = self._mapping(document.get("services"), "services")
        robot = self._mapping(document.get("robot"), "robot")
        robot_arm = self._mapping(services.get("robot_arm"), "services.robot_arm")
        if robot_arm.get("plugin_adapter") != "fake_robot_arm":
            raise ValueError(
                "neugrasp_trial_node is fake-only and requires "
                "services.robot_arm.plugin_adapter=fake_robot_arm"
            )
        kinematics_config = self._mapping(
            services.get("kinematics"), "services.kinematics"
        )
        planner_config = self._mapping(
            services.get("joint_trajectory_planner"),
            "services.joint_trajectory_planner",
        )
        self._live_ik = KinematicsService.from_config(
            service_config=kinematics_config,
            package_share_directory=get_package_share_directory,
            robot_config=robot,
        )
        self._scratch_ik = KinematicsService.from_config(
            service_config=kinematics_config,
            package_share_directory=get_package_share_directory,
            robot_config=robot,
        )
        self._joint_planner = JointTrajectoryPlannerService.from_config(
            service_config=planner_config,
            joint_metadata=self._live_ik.joint_metadata,
        )
        if self._live_ik.base_frame != self._config.frames.base:
            raise ValueError(
                "trial base frame {!r} does not match kinematics base frame {!r}".format(
                    self._config.frames.base, self._live_ik.base_frame
                )
            )
        if self._live_ik.tool_frame != self._config.frames.tool:
            raise ValueError(
                "trial tool frame {!r} does not match kinematics tool frame {!r}".format(
                    self._config.frames.tool, self._live_ik.tool_frame
                )
            )
        named_poses = self._mapping(robot.get("named_poses"), "robot.named_poses")
        home = self._mapping(named_poses.get("home"), "robot.named_poses.home")
        self._home_joint_positions = JointPositions(
            self._numeric_vector(home.get("positions_rad"), "robot.named_poses.home.positions_rad", 6)
        )

    @classmethod
    def _load_trial_config(cls, path: Path) -> TrialConfig:
        if not path.is_file():
            raise ValueError(f"trial_config does not exist: {path}")
        try:
            with path.open("r", encoding="utf-8") as stream:
                document = yaml.safe_load(stream)
        except yaml.YAMLError as error:
            raise ValueError(f"trial_config is invalid YAML: {path}") from error
        if not isinstance(document, dict):
            raise TypeError("trial_config must be a mapping")
        if document.get("schema_version") != 1:
            raise ValueError("trial_config schema_version must be 1")
        # ``status`` is the explicit profile safety label.  ``mode`` is
        # accepted as a compatibility alias so an early local profile cannot
        # silently turn this fake-only coordinator into a hardware path.
        status = document.get("status", document.get("mode"))
        if status != "FAKE":
            raise ValueError("trial_config status must be exactly FAKE")
        if "mode" in document and document["mode"] != "FAKE":
            raise ValueError("trial_config mode, when present, must be FAKE")
        frames = cls._mapping(document.get("frames"), "frames")
        parsed_frames = TrialFrames(
            base=cls._frame(frames.get("base"), "frames.base"),
            volume=cls._frame(frames.get("volume"), "frames.volume"),
            tool=cls._frame(frames.get("tool"), "frames.tool"),
        )
        mapping = cls._mapping(document.get("fake_grasp_to_tool0"), "fake_grasp_to_tool0")
        if mapping.get("status") != "FAKE":
            raise ValueError("fake_grasp_to_tool0.status must be FAKE")
        grasp_to_tool = RigidTransform(
            translation=cls._numeric_vector(
                mapping.get("translation_m"), "fake_grasp_to_tool0.translation_m", 3
            ),
            rotation=normalize_quaternion(
                cls._numeric_vector(
                    mapping.get("rotation_xyzw"),
                    "fake_grasp_to_tool0.rotation_xyzw",
                    4,
                )
            ),
        )
        primitive = cls._mapping(document.get("primitive"), "primitive")
        primitive_policy = PrimitivePolicy(
            pregrasp_distance_m=cls._positive_float(
                primitive.get("pregrasp_distance_m"), "primitive.pregrasp_distance_m"
            ),
            lift_distance_m=cls._positive_float(
                primitive.get("lift_distance_m"), "primitive.lift_distance_m"
            ),
            top_down_max_angle_rad=math.radians(cls._positive_float(
                primitive.get("top_down_max_angle_deg"), "primitive.top_down_max_angle_deg"
            )),
        )
        if primitive_policy.top_down_max_angle_rad > math.pi:
            raise ValueError("primitive.top_down_max_angle_deg must not exceed 180")
        execution = cls._mapping(document.get("execution"), "execution")
        execution_policy = ExecutionPolicy(
            gripper_opening_m=cls._nonnegative_float(
                execution.get("gripper_opening_m"), "execution.gripper_opening_m"
            ),
            gripper_closed_m=cls._nonnegative_float(
                execution.get("gripper_closed_m"), "execution.gripper_closed_m"
            ),
            readiness_timeout_s=cls._positive_float(
                execution.get("readiness_timeout_s", 20.0), "execution.readiness_timeout_s"
            ),
            action_timeout_s=cls._positive_float(
                execution.get("action_timeout_s", 180.0), "execution.action_timeout_s"
            ),
            gripper_timeout_s=cls._positive_float(
                execution.get("gripper_timeout_s", 5.0), "execution.gripper_timeout_s"
            ),
            gripper_tolerance_m=cls._positive_float(
                execution.get("gripper_tolerance_m", 0.002), "execution.gripper_tolerance_m"
            ),
            measured_joint_max_age_s=cls._positive_float(
                execution.get("measured_joint_max_age_s", 0.5), "execution.measured_joint_max_age_s"
            ),
        )
        if execution_policy.gripper_opening_m > 0.08 + 1e-9:
            raise ValueError("execution.gripper_opening_m must not exceed fake gripper limit 0.08")
        if execution_policy.gripper_closed_m > execution_policy.gripper_opening_m:
            raise ValueError("execution.gripper_closed_m must not exceed gripper_opening_m")
        timing = cls._mapping(document.get("timing"), "timing")
        after_phase = cls._mapping(timing.get("after_phase_s"), "timing.after_phase_s")
        required_phase_names = (
            "init_home", "scan", "predict_artifact", "select_preflight",
            "pregrasp", "grasp", "close", "lift",
        )
        if set(after_phase) != set(required_phase_names):
            raise ValueError("timing.after_phase_s must define exactly: " + ", ".join(required_phase_names))
        timing_policy = TimingPolicy(
            scan_view_settle_s=cls._nonnegative_float(
                timing.get("scan_view_settle_s"), "timing.scan_view_settle_s"
            ),
            after_phase_s={
                name: cls._nonnegative_float(after_phase[name], "timing.after_phase_s." + name)
                for name in required_phase_names
            },
        )
        visualization = cls._mapping(document.get("visualization"), "visualization")
        z_index_range_values = cls._numeric_vector(
            visualization.get("tsdf_z_index_range"), "visualization.tsdf_z_index_range", 2
        )
        if any(value < 0.0 or int(value) != value for value in z_index_range_values):
            raise ValueError("visualization.tsdf_z_index_range must contain non-negative integer indices")
        tsdf_z_index_range = tuple(int(value) for value in z_index_range_values)
        artifact_document = document.get("artifacts", {})
        if artifact_document is None:
            artifact_document = {}
        if not isinstance(artifact_document, dict):
            raise TypeError("artifacts must be a mapping")
        # The shared pure pipeline owns validation/defaults and is used by both
        # replay and trial.  Its public mapping constructor avoids duplicated
        # processing constants at this ROS boundary.
        artifacts = ArtifactSettings.from_mapping(artifact_document)
        z_min, z_max = tsdf_z_index_range
        if z_max < z_min or z_max >= artifacts.volume_resolution:
            raise ValueError("visualization.tsdf_z_index_range must be a valid inclusive volume Z range")
        return TrialConfig(
            frames=parsed_frames,
            grasp_to_tool=grasp_to_tool,
            primitive=primitive_policy,
            execution=execution_policy,
            timing=timing_policy,
            visualization=VisualizationPolicy(tsdf_z_index_range=tsdf_z_index_range),
            artifacts=artifacts,
        )

    # ----- ROS feedback and lifecycle -----------------------------------

    def _joint_state_callback(self, message: JointState) -> None:
        try:
            positions = self._canonical_joint_positions(message, self._live_ik.joint_names)
            now_s = time.monotonic()
            with self._state_lock:
                self._latest_joints = positions
                self._latest_joints_at_s = now_s
                self._live_ik.update_measured_joint_positions(
                    positions, received_at_monotonic_s=now_s
                )
        except (TypeError, ValueError) as error:
            self.get_logger().warning(f"Trial rejected joint feedback: {error}")

    def _gripper_state_callback(self, message: JointState) -> None:
        try:
            if len(message.name) != len(message.position):
                raise ValueError("gripper JointState names and positions have different lengths")
            if "left_gripper_joint" not in message.name:
                raise ValueError("gripper JointState is missing left_gripper_joint")
            left = float(message.position[message.name.index("left_gripper_joint")])
            if not math.isfinite(left) or left < -1e-9:
                raise ValueError("gripper JointState has invalid left jaw opening")
            with self._state_lock:
                # Driver contract: gripper state is one jaw; public command is
                # total fingertip opening.
                self._latest_gripper_opening_m = 2.0 * left
                self._latest_gripper_at_s = time.monotonic()
        except ValueError as error:
            self.get_logger().warning(f"Trial rejected gripper feedback: {error}")

    def _start_once(self) -> None:
        if self._started:
            return
        self._started = True
        self._start_timer.cancel()
        self._worker = threading.Thread(
            target=self._run_one_trial,
            name="neugrasp_fake_trial",
            daemon=True,
        )
        self._worker.start()

    def _run_one_trial(self) -> None:
        try:
            self._wait_ready()
            self._publish_phase("INIT_HOME")
            self._command_gripper(self._config.execution.gripper_opening_m)
            self._move_home()
            self._settle_after("init_home")

            self._publish_phase("SCAN")
            self._run_scan()
            self._settle_after("scan")

            self._publish_phase("PREDICT_ARTIFACT")
            artifacts = load_artifacts(self._run_dir, self._config.artifacts)
            rendered_points = self._artifact_visualizer.publish_snapshot(artifacts)
            self.get_logger().info(
                "Loaded artifact prediction in current volume frame: "
                f"tsdf_points={len(artifacts.surface_indices)} rendered_tsdf_points={rendered_points} "
                f"candidates={len(artifacts.candidates)}"
            )
            if not artifacts.candidates:
                raise TrialFailure("artifact postprocess produced no grasp candidates")
            self._settle_after("predict_artifact")

            self._publish_phase("SELECT_PREFLIGHT")
            base_volume = self._lookup_transform(
                self._config.frames.base, self._config.frames.volume
            )
            selected = self._select_feasible_candidate(artifacts, base_volume)
            self._publish_selected_targets(selected)
            self._settle_after("select_preflight")

            self._publish_phase("PREGRASP")
            self._move_to_transform(selected.pregrasp_tool, "pregrasp")
            self._settle_after("pregrasp")
            self._publish_phase("GRASP")
            self._move_to_transform(selected.grasp_tool, "grasp")
            self._settle_after("grasp")
            self._publish_phase("CLOSE")
            self._command_gripper(self._config.execution.gripper_closed_m)
            self._settle_after("close")
            self._publish_phase("LIFT")
            self._move_to_transform(selected.lift_tool, "lift")
            self._settle_after("lift")
            self._publish_phase("COMPLETE")
            self.get_logger().info(
                "Fake trial COMPLETE. This confirms only the motion sequence; "
                "FakeRobotArm has no contact, collision, or object-attachment validation."
            )
        except TrialFailure as error:
            self._fail_closed(str(error))
        except Exception as error:  # noqa: BLE001 - ROS boundary stays fail-closed.
            self._fail_closed(f"unexpected trial error: {error}")

    def _wait_ready(self) -> None:
        deadline = time.monotonic() + self._config.execution.readiness_timeout_s
        missing = ""
        while rclpy.ok() and not self._stop_requested.is_set() and time.monotonic() < deadline:
            missing_parts = []
            if self._fresh_joints() is None:
                missing_parts.append("fresh /myarm/state/joint_state")
            if not self._fresh_gripper_opening():
                missing_parts.append("fresh /myarm/gripper/state")
            try:
                self._lookup_transform(self._config.frames.base, self._config.frames.volume)
            except TransformException:
                missing_parts.append(
                    f"TF {self._config.frames.base} <- {self._config.frames.volume}"
                )
            if not self._scan_client.wait_for_server(timeout_sec=0.05):
                missing_parts.append("ScanWorkspace action server")
            if not self._trajectory_client.wait_for_server(timeout_sec=0.05):
                missing_parts.append("FollowJointTrajectory action server")
            if not missing_parts:
                return
            missing = ", ".join(missing_parts)
            self._interruptible_wait(0.05)
        if self._stop_requested.is_set() or not rclpy.ok():
            raise TrialFailure("trial stopped while waiting for readiness")
        raise TrialFailure(f"readiness timeout waiting for: {missing}")

    # ----- phase operations ---------------------------------------------

    def _run_scan(self) -> None:
        goal = ScanWorkspace.Goal()
        goal.profile_id = self._scan_profile_id
        goal.execute_motion = True
        goal.capture_enabled = False
        goal.settle_time_s = self._config.timing.scan_view_settle_s
        handle = self._send_action_goal(self._scan_client, goal, "ScanWorkspace")
        result_wrap = self._await_future(
            handle.get_result_async(), "ScanWorkspace result", self._config.execution.action_timeout_s
        )
        self._clear_active_action(handle)
        if result_wrap is None or result_wrap.result is None:
            raise TrialFailure("ScanWorkspace returned no result")
        result = result_wrap.result
        if not result.succeeded:
            raise TrialFailure(
                "scan failed: {}: {}".format(result.failure_reason, result.detail)
            )
        self.get_logger().info(
            "Scan complete: profile={} views={}".format(
                self._scan_profile_id, result.completed_view_count
            )
        )

    def _move_home(self) -> None:
        self._execute_joint_motion(self._home_joint_positions, "home")

    def _move_to_transform(self, target: RigidTransform, stage: str) -> None:
        current = self._require_fresh_joints(stage)
        goal = self._solve_live_ik(target, current, stage)
        self._execute_joint_motion(goal, stage, start=current)

    def _execute_joint_motion(
        self,
        goal_positions: JointPositions,
        stage: str,
        start: Optional[JointPositions] = None,
    ) -> None:
        current = start or self._require_fresh_joints(stage)
        try:
            plan = self._joint_planner.plan_joint_motion(
                q_start=current, q_goal=goal_positions
            )
        except (TypeError, ValueError) as error:
            raise TrialFailure(f"{stage}: joint plan construction failed: {error}") from error
        if not plan.succeeded or plan.trajectory is None:
            reason = plan.failure_reason.value if plan.failure_reason is not None else "unknown"
            raise TrialFailure(f"{stage}: joint planning failed ({reason}): {plan.detail}")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = self._joint_trajectory_to_ros(plan.trajectory)
        handle = self._send_action_goal(self._trajectory_client, goal, f"{stage} trajectory")
        result_wrap = self._await_future(
            handle.get_result_async(),
            f"{stage} trajectory result",
            self._config.execution.action_timeout_s,
        )
        self._clear_active_action(handle)
        if result_wrap is None or result_wrap.result is None:
            raise TrialFailure(f"{stage}: FollowJointTrajectory returned no result")
        result = result_wrap.result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise TrialFailure(
                f"{stage}: FollowJointTrajectory failed: {result.error_string}"
            )

    def _command_gripper(self, opening_m: float) -> None:
        command = Float64()
        command.data = opening_m
        self._gripper_publisher.publish(command)
        self.get_logger().info("Gripper command: {:.4f} m total opening".format(opening_m))
        deadline = time.monotonic() + self._config.execution.gripper_timeout_s
        while rclpy.ok() and not self._stop_requested.is_set() and time.monotonic() < deadline:
            with self._state_lock:
                measured = self._latest_gripper_opening_m
                measured_at_s = self._latest_gripper_at_s
            if (
                measured is not None
                and measured_at_s is not None
                and time.monotonic() - measured_at_s <= self._config.execution.measured_joint_max_age_s
                and abs(measured - opening_m) <= self._config.execution.gripper_tolerance_m
            ):
                self.get_logger().info(
                    "Gripper feedback reached {:.4f} m total opening".format(measured)
                )
                return
            self._interruptible_wait(0.02)
        if self._stop_requested.is_set() or not rclpy.ok():
            raise TrialFailure("trial stopped while waiting for gripper state")
        raise TrialFailure(
            "gripper did not reach {:.4f} m within {:.1f} s".format(
                opening_m, self._config.execution.gripper_timeout_s
            )
        )

    # ----- artifact pose contract / preflight --------------------------

    def _select_feasible_candidate(
        self, artifacts: ArtifactBundle, base_volume: RigidTransform
    ) -> CandidateTargets:
        seed = self._require_fresh_joints("candidate preflight")
        rejected = []
        for candidate in artifacts.candidates:
            try:
                targets = self._candidate_targets(candidate, base_volume)
                final_q = self._preflight_targets(targets, seed)
            except TrialFailure as error:
                rejected.append(
                    "score={:.4f} index={}: {}".format(
                        candidate.score, list(candidate.index), error
                    )
                )
                continue
            self.get_logger().info(
                "Selected feasible artifact candidate score={:.4f} index={} width_m={:.4f}".format(
                    candidate.score, list(candidate.index), candidate.width_m
                )
            )
            return CandidateTargets(
                candidate=candidate,
                grasp_tool=targets.grasp_tool,
                pregrasp_tool=targets.pregrasp_tool,
                lift_tool=targets.lift_tool,
                grasp_frame=targets.grasp_frame,
                pregrasp_frame=targets.pregrasp_frame,
                lift_frame=targets.lift_frame,
                preflight_final_joint_positions=final_q,
            )
        detail = "; ".join(rejected[:5]) or "no candidates"
        raise TrialFailure("no candidate is feasible through pregrasp/grasp/lift: " + detail)

    def _candidate_targets(
        self, candidate: ArtifactCandidate, base_volume: RigidTransform
    ) -> PrimitiveTargets:
        # Contract: T_base_tool0 = T_base_volume * T_volume_grasp *
        # T_grasp_tool0.  The config intentionally carries this direct
        # grasp-to-tool0 relation for the FakeRobotArm profile.
        volume_grasp = RigidTransform(
            translation=tuple(candidate.position_m),
            rotation=tuple(candidate.rotation_xyzw),
        )
        base_grasp = compose(base_volume, volume_grasp)
        grasp_tool = compose(base_grasp, self._config.grasp_to_tool)
        pregrasp_grasp = compose(
            base_grasp,
            RigidTransform(
                translation=(0.0, 0.0, -self._config.primitive.pregrasp_distance_m),
                rotation=_IDENTITY,
            ),
        )
        pregrasp_tool = compose(pregrasp_grasp, self._config.grasp_to_tool)

        # PyBullet policy: local -Z retreat for top-down grasps; otherwise
        # translate +Z in the base frame.  Predicted local +Z is the approach
        # direction in the NeuGrasp convention.
        approach_base = rotate_vector(base_grasp.rotation, (0.0, 0.0, 1.0))
        dot_down = max(-1.0, min(1.0, -approach_base[2]))
        angle_to_world_down = math.acos(dot_down)
        if angle_to_world_down <= self._config.primitive.top_down_max_angle_rad:
            lift_grasp = compose(
                base_grasp,
                RigidTransform(
                    translation=(0.0, 0.0, -self._config.primitive.lift_distance_m),
                    rotation=_IDENTITY,
                ),
            )
        else:
            lift_grasp = RigidTransform(
                translation=(
                    base_grasp.translation[0],
                    base_grasp.translation[1],
                    base_grasp.translation[2] + self._config.primitive.lift_distance_m,
                ),
                rotation=base_grasp.rotation,
            )
        lift_tool = compose(lift_grasp, self._config.grasp_to_tool)
        return PrimitiveTargets(
            grasp_tool=grasp_tool,
            pregrasp_tool=pregrasp_tool,
            lift_tool=lift_tool,
            grasp_frame=base_grasp,
            pregrasp_frame=pregrasp_grasp,
            lift_frame=lift_grasp,
        )

    def _preflight_targets(
        self,
        targets: PrimitiveTargets,
        seed: JointPositions,
    ) -> JointPositions:
        # Preflight ordering is physical ordering, even though the return
        # tuple is (grasp, pregrasp, lift) for publication convenience.
        q = seed
        for stage, target in (
            ("pregrasp", targets.pregrasp_tool),
            ("grasp", targets.grasp_tool),
            ("lift", targets.lift_tool),
        ):
            q_next = self._solve_scratch_ik(target, q, stage)
            try:
                plan = self._joint_planner.plan_joint_motion(q_start=q, q_goal=q_next)
            except (TypeError, ValueError) as error:
                raise TrialFailure(f"{stage} preflight planning invalid: {error}") from error
            if not plan.succeeded or plan.trajectory is None:
                reason = plan.failure_reason.value if plan.failure_reason is not None else "unknown"
                raise TrialFailure(
                    f"{stage} preflight joint plan failed ({reason}): {plan.detail}"
                )
            q = q_next
        return q

    def _solve_scratch_ik(
        self, transform: RigidTransform, seed: JointPositions, stage: str
    ) -> JointPositions:
        with self._planning_lock:
            self._scratch_ik.set_target_pose(self._sdk_pose(transform), seed=seed)
            step = self._scratch_ik.step(now_monotonic_s=time.monotonic())
        return self._validated_ik_goal(step, f"{stage} preflight")

    def _solve_live_ik(
        self, transform: RigidTransform, seed: JointPositions, stage: str
    ) -> JointPositions:
        # Explicit fresh feedback seed is used on every real endpoint.  This
        # deliberately does not reuse the scratch solution selected earlier.
        with self._planning_lock:
            self._live_ik.set_target_pose(self._sdk_pose(transform), seed=seed)
            step = self._live_ik.step(now_monotonic_s=time.monotonic())
        return self._validated_ik_goal(step, stage)

    @staticmethod
    def _validated_ik_goal(step, stage: str) -> JointPositions:
        result = step.ik_result
        if result is None or not result.converged or step.joint_goal is None:
            reason = (
                result.failure_reason.value
                if result is not None and result.failure_reason is not None
                else "ik_failed"
            )
            detail = result.detail if result is not None else "one-shot IK returned no result"
            raise TrialFailure(f"{stage}: IK failed ({reason}): {detail}")
        return step.joint_goal

    def _publish_selected_targets(self, selected: CandidateTargets) -> None:
        self._selected_pose_publisher.publish(
            self._pose_stamped(self._config.frames.base, selected.grasp_tool)
        )
        self._selected_marker_publisher.publish(
            self._target_marker(
                "selected_grasp", selected.grasp_frame, selected.candidate.width_m,
                (0.10, 1.0, 0.20, 1.0), 0,
            )
        )
        self._pregrasp_marker_publisher.publish(
            self._target_marker(
                "pregrasp", selected.pregrasp_frame, selected.candidate.width_m,
                (0.0, 0.90, 1.0, 1.0), 0,
            )
        )
        self._lift_marker_publisher.publish(
            self._target_marker(
                "lift", selected.lift_frame, selected.candidate.width_m,
                (0.70, 0.25, 1.0, 1.0), 0,
            )
        )

    # ----- action / TF helpers ------------------------------------------

    def _send_action_goal(self, client, goal, label: str):
        if not client.wait_for_server(timeout_sec=self._config.execution.readiness_timeout_s):
            raise TrialFailure(f"{label} action server is unavailable")
        future = client.send_goal_async(goal)
        handle = self._await_future(
            future, f"{label} goal response", self._config.execution.action_timeout_s
        )
        if handle is None or not handle.accepted:
            raise TrialFailure(f"{label} goal was rejected")
        with self._state_lock:
            self._active_action_goal = handle
        return handle

    def _await_future(self, future, label: str, timeout_s: float):
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and not self._stop_requested.is_set() and time.monotonic() < deadline:
            if future.done():
                try:
                    return future.result()
                except Exception as error:  # noqa: BLE001
                    raise TrialFailure(f"{label} transport failed: {error}") from error
            self._interruptible_wait(0.02)
        if self._stop_requested.is_set() or not rclpy.ok():
            raise TrialFailure(f"{label} stopped")
        raise TrialFailure(f"{label} timed out after {timeout_s:.1f} s")

    def _clear_active_action(self, handle) -> None:
        with self._state_lock:
            if self._active_action_goal is handle:
                self._active_action_goal = None

    def _lookup_transform(self, target: str, source: str) -> RigidTransform:
        transform = self._tf_buffer.lookup_transform(target, source, Time())
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return RigidTransform(
            translation=(translation.x, translation.y, translation.z),
            rotation=(rotation.x, rotation.y, rotation.z, rotation.w),
        )

    def _require_fresh_joints(self, stage: str) -> JointPositions:
        current = self._fresh_joints()
        if current is None:
            raise TrialFailure(f"{stage}: fresh joint state is unavailable")
        return current

    def _fresh_joints(self) -> Optional[JointPositions]:
        with self._state_lock:
            joints = self._latest_joints
            received_at_s = self._latest_joints_at_s
        if (
            joints is None
            or received_at_s is None
            or time.monotonic() - received_at_s > self._config.execution.measured_joint_max_age_s
        ):
            return None
        return joints

    def _fresh_gripper_opening(self) -> bool:
        with self._state_lock:
            received_at_s = self._latest_gripper_at_s
        return (
            received_at_s is not None
            and time.monotonic() - received_at_s <= self._config.execution.measured_joint_max_age_s
        )

    def _fail_closed(self, detail: str) -> None:
        self._stop_requested.set()
        with self._state_lock:
            active = self._active_action_goal
        if active is not None:
            try:
                active.cancel_goal_async()
            except Exception as error:  # noqa: BLE001 - executor cancel still follows.
                self.get_logger().warning(f"Unable to cancel active trial action: {error}")
        if self._motion_cancel_client.service_is_ready():
            try:
                self._motion_cancel_client.call_async(Trigger.Request())
            except Exception as error:  # noqa: BLE001
                self.get_logger().warning(f"Unable to request motion executor cancel: {error}")
        self._terminal = True
        self._publish_phase("FAILED")
        self.get_logger().error("Fake trial FAILED closed: " + detail)

    def _settle_after(self, phase_name: str) -> None:
        duration_s = self._config.timing.after_phase_s[phase_name]
        if duration_s <= 0.0:
            return
        self.get_logger().info(
            "Settling after {} for {:.1f} s".format(phase_name, duration_s)
        )
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline and not self._stop_requested.is_set() and rclpy.ok():
            self._interruptible_wait(min(0.02, deadline - time.monotonic()))
        if self._stop_requested.is_set() or not rclpy.ok():
            raise TrialFailure("trial stopped during {} settle".format(phase_name))

    def _interruptible_wait(self, seconds: float) -> None:
        self._stop_requested.wait(max(0.0, min(seconds, 0.05)))

    # ----- visualization -------------------------------------------------

    def _target_marker(
        self,
        namespace: str,
        transform: RigidTransform,
        width_m: float,
        rgba: Tuple[float, float, float, float],
        marker_id: int,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._config.frames.base
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.004
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba
        for first, second in GRIPPER_EDGES:
            local = gripper_wireframe_points(width_m)
            for index in (first, second):
                rotated = rotate_vector(transform.rotation, tuple(local[index]))
                marker.points.append(
                    self._point((
                        transform.translation[0] + rotated[0],
                        transform.translation[1] + rotated[1],
                        transform.translation[2] + rotated[2],
                    ))
                )
        return marker

    def _publish_phase(self, phase: str) -> None:
        message = String()
        message.data = phase
        self._phase_publisher.publish(message)
        self.get_logger().info("Trial phase: " + phase)

    def _pose_stamped(self, frame_id: str, transform: RigidTransform) -> PoseStamped:
        message = PoseStamped()
        message.header.frame_id = frame_id
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose = self._pose(transform)
        return message

    @staticmethod
    def _pose(transform: RigidTransform) -> Pose:
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = transform.translation
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = transform.rotation
        return pose

    @staticmethod
    def _point(values: Sequence[float]) -> Point:
        message = Point()
        message.x, message.y, message.z = (float(value) for value in values)
        return message

    def _sdk_pose(self, transform: RigidTransform) -> SdkPose:
        return SdkPose(position=transform.translation, orientation=transform.rotation)

    @staticmethod
    def _joint_trajectory_to_ros(trajectory) -> JointTrajectory:
        message = JointTrajectory()
        message.joint_names = list(trajectory.joint_names)
        for point in trajectory.points:
            ros_point = JointTrajectoryPoint()
            ros_point.positions = list(point.positions.values)
            ros_point.velocities = list(point.velocities.values)
            ros_point.accelerations = list(point.accelerations.values)
            seconds = int(point.time_from_start_s)
            nanoseconds = int(round((point.time_from_start_s - seconds) * 1_000_000_000))
            if nanoseconds == 1_000_000_000:
                seconds += 1
                nanoseconds = 0
            ros_point.time_from_start.sec = seconds
            ros_point.time_from_start.nanosec = nanoseconds
            message.points.append(ros_point)
        return message

    # ----- compact validation helpers -----------------------------------

    @staticmethod
    def _mapping(value: Any, name: str) -> Mapping[str, Any]:
        if not isinstance(value, dict):
            raise TypeError(f"{name} must be a mapping")
        return value

    @staticmethod
    def _frame(value: Any, name: str) -> str:
        if not isinstance(value, str) or not value.strip() or value.startswith("/"):
            raise ValueError(f"{name} must be a non-empty relative TF frame")
        return value.strip()

    @classmethod
    def _validate_launch_frame(cls, value: Any, expected: str, name: str) -> None:
        supplied = cls._frame(value, name)
        if supplied != expected:
            raise ValueError(
                f"{name}={supplied!r} does not match fake trial config frame {expected!r}"
            )

    @staticmethod
    def _nonempty(value: Any, name: str) -> str:
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f"{name} must be a non-empty name")
        return value.strip()

    @staticmethod
    def _numeric_vector(value: Any, name: str, length: int) -> Tuple[float, ...]:
        if not isinstance(value, (list, tuple)) or len(value) != length:
            raise ValueError(f"{name} must contain {length} values")
        try:
            values = tuple(float(item) for item in value)
        except (TypeError, ValueError) as error:
            raise TypeError(f"{name} must be numeric") from error
        if not all(math.isfinite(item) for item in values):
            raise ValueError(f"{name} must be finite")
        return values

    @classmethod
    def _positive_float(cls, value: Any, name: str) -> float:
        numeric = cls._nonnegative_float(value, name)
        if numeric <= 0.0:
            raise ValueError(f"{name} must be positive")
        return numeric

    @staticmethod
    def _nonnegative_float(value: Any, name: str) -> float:
        if isinstance(value, bool):
            raise TypeError(f"{name} must be numeric")
        try:
            numeric = float(value)
        except (TypeError, ValueError) as error:
            raise TypeError(f"{name} must be numeric") from error
        if not math.isfinite(numeric) or numeric < 0.0:
            raise ValueError(f"{name} must be finite and non-negative")
        return numeric

    @staticmethod
    def _nonnegative_int(value: Any, name: str) -> int:
        if isinstance(value, bool):
            raise TypeError(f"{name} must be an integer")
        try:
            numeric = int(value)
        except (TypeError, ValueError) as error:
            raise TypeError(f"{name} must be an integer") from error
        if numeric < 0 or numeric != value:
            raise ValueError(f"{name} must be a non-negative integer")
        return numeric

    @staticmethod
    def _canonical_joint_positions(
        message: JointState, joint_names: Sequence[str]
    ) -> JointPositions:
        if message.name:
            if len(message.name) != len(message.position):
                raise ValueError("JointState name and position lengths differ")
            if len(set(message.name)) != len(message.name):
                raise ValueError("JointState contains duplicate names")
            by_name = dict(zip(message.name, message.position))
            missing = [name for name in joint_names if name not in by_name]
            if missing:
                raise ValueError("JointState is missing arm joints: " + ", ".join(missing))
            return JointPositions(tuple(float(by_name[name]) for name in joint_names))
        if len(message.position) != len(joint_names):
            raise ValueError(
                "unnamed JointState must contain exactly {} arm positions".format(
                    len(joint_names)
                )
            )
        return JointPositions(tuple(float(value) for value in message.position))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = NeugraspTrialNode()
        executor = MultiThreadedExecutor(num_threads=4)
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
