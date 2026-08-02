"""Sequential NeuGrasp scan coordinator.

The node owns the application-level ScanWorkspace action, profile expansion
and visualization messages. It never publishes a driver setpoint or opens a
robot connection. Motion is delegated one view at a time either to the
fake-only Cartesian action or to one-shot IK plus FollowJointTrajectory.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped
from myarm_interfaces.action import FollowCartesianTrajectory, ScanWorkspace
from myarm_sdk.core import JointPositions, Pose as SdkPose, load_sdk_yaml
from myarm_sdk.service import KinematicsService, JointTrajectoryPlannerService
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray

from .math3d import RigidTransform, compose, finite_vector, inverse, optical_look_at


_SNAPSHOT_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


@dataclass(frozen=True)
class ScanFrames:
    base: str
    workspace: str
    tool: str
    camera_optical: str


@dataclass(frozen=True)
class ScanView:
    key: str
    source_view_id: str
    camera_workspace: RigidTransform


@dataclass(frozen=True)
class ScanProfile:
    profile_id: str
    version: int
    views: Tuple[ScanView, ...]
    capture_order: Tuple[str, ...]
    model_input_order: Tuple[str, ...]
    query_view_key: str
    default_settle_time_s: float


@dataclass(frozen=True)
class ScanMotionResult:
    """Normalized result returned by either delegated motion path."""

    succeeded: bool
    failure_reason: str
    detail: str


class NeugraspScanNode(Node):
    """Create visual scan targets and execute them serially against fake arm."""

    def __init__(self) -> None:
        super().__init__("neugrasp_scan")
        self.declare_parameter("scan_config", "")
        self.declare_parameter("action_name", "/neugrasp/scan_workspace")
        self.declare_parameter(
            "follow_cartesian_action", "/myarm/follow_cartesian_trajectory"
        )
        self.declare_parameter(
            "follow_joint_trajectory_action", "/myarm/follow_joint_trajectory"
        )
        self.declare_parameter("services_config", "service/config/services.yaml")
        self.declare_parameter("motion_planner", "")
        self.declare_parameter(
            "motion_cancel_service", "/myarm/motion_execution/cancel"
        )
        self.declare_parameter("default_profile", "")
        self.declare_parameter("server_wait_timeout_s", 5.0)

        config_path = Path(str(self.get_parameter("scan_config").value)).expanduser()
        self._frames, self._profiles = self._load_config(config_path)
        configured_motion_planner = self._load_motion_planner(config_path)
        requested_motion_planner = str(
            self.get_parameter("motion_planner").value
        ).strip()
        self._motion_planner = self._validate_motion_planner(
            requested_motion_planner or configured_motion_planner
        )
        requested_default = str(self.get_parameter("default_profile").value).strip()
        self._default_profile_id = requested_default or next(iter(self._profiles))
        if self._default_profile_id not in self._profiles:
            raise ValueError(
                f"default_profile is not present in scan config: {self._default_profile_id}"
            )
        self._server_wait_timeout_s = self._positive_float(
            self.get_parameter("server_wait_timeout_s").value,
            "server_wait_timeout_s",
        )

        self._state_lock = threading.RLock()
        self._goal_reserved = False
        self._active_goal = None
        self._active_child_goal = None
        self._joint_planning_lock = threading.Lock()

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        self._scan_views_publisher = self.create_publisher(
            PoseArray, "/neugrasp/scan_views", _SNAPSHOT_QOS
        )
        self._planned_poses_publisher = self.create_publisher(
            PoseArray, "/neugrasp/planned_camera_poses", _SNAPSHOT_QOS
        )
        self._measured_poses_publisher = self.create_publisher(
            PoseArray, "/neugrasp/measured_camera_poses", _SNAPSHOT_QOS
        )
        self._scan_markers_publisher = self.create_publisher(
            MarkerArray, "/neugrasp/scan_view_markers", _SNAPSHOT_QOS
        )

        callback_group = ReentrantCallbackGroup()
        self._cartesian_client = None
        self._joint_client = None
        self._one_shot_ik = None
        self._joint_trajectory_planner = None
        self._latest_measured_joint_positions = None
        self._latest_measured_at_s = None
        if self._motion_planner == "cartesian_trajectory":
            self._cartesian_client = ActionClient(
                self,
                FollowCartesianTrajectory,
                str(self.get_parameter("follow_cartesian_action").value),
                callback_group=callback_group,
            )
        else:
            self._configure_one_shot_joint_path()
            self._joint_client = ActionClient(
                self,
                FollowJointTrajectory,
                str(self.get_parameter("follow_joint_trajectory_action").value),
                callback_group=callback_group,
            )
        self._motion_cancel_client = self.create_client(
            Trigger,
            str(self.get_parameter("motion_cancel_service").value),
            callback_group=callback_group,
        )
        self._scan_action = ActionServer(
            self,
            ScanWorkspace,
            str(self.get_parameter("action_name").value),
            execute_callback=self._execute_scan,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=callback_group,
        )
        self.get_logger().info(
            "Neugrasp scan coordinator ready: profile={}, views={}, planner={}, action={}".format(
                self._default_profile_id,
                len(self._profiles[self._default_profile_id].views),
                self._motion_planner,
                self.get_parameter("action_name").value,
            )
        )

    @staticmethod
    def _positive_float(value: Any, name: str) -> float:
        try:
            normalized = float(value)
        except (TypeError, ValueError) as error:
            raise TypeError(f"{name} must be numeric") from error
        if not math.isfinite(normalized) or normalized <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
        return normalized

    @classmethod
    def _load_motion_planner(cls, path: Path) -> str:
        try:
            with path.open("r", encoding="utf-8") as stream:
                document = yaml.safe_load(stream)
        except OSError as error:
            raise ValueError(f"scan_config cannot be opened: {path}") from error
        except yaml.YAMLError as error:
            raise ValueError(f"scan_config is invalid YAML: {path}") from error
        if not isinstance(document, dict):
            raise TypeError("scan_config must be a mapping")
        execution = cls._mapping_or_empty(document.get("execution"), "execution")
        return str(execution.get("motion_planner", "cartesian_trajectory")).strip()

    @staticmethod
    def _validate_motion_planner(value: str) -> str:
        supported = {"cartesian_trajectory", "one_shot_ik_joint_trajectory"}
        if value not in supported:
            raise ValueError(
                "motion_planner must be one of {}, got {!r}".format(
                    sorted(supported), value
                )
            )
        return value

    def _configure_one_shot_joint_path(self) -> None:
        """Build private IK and joint-planning services for one scan view.

        The services are pure and use the same SDK config/URDF metadata as the
        robot-side nodes. The coordinator subscribes only to canonical measured
        feedback and delegates the resulting full trajectory to the public
        FollowJointTrajectory executor action.
        """
        services_document = load_sdk_yaml(
            str(self.get_parameter("services_config").value)
        )
        services = self._mapping(services_document.get("services"), "services")
        robot = self._mapping(services_document.get("robot"), "robot")
        kinematics_config = self._mapping(
            services.get("kinematics"), "services.kinematics"
        )
        planner_config = self._mapping(
            services.get("joint_trajectory_planner"),
            "services.joint_trajectory_planner",
        )
        self._one_shot_ik = KinematicsService.from_config(
            service_config=kinematics_config,
            package_share_directory=get_package_share_directory,
            robot_config=robot,
        )
        self._joint_trajectory_planner = JointTrajectoryPlannerService.from_config(
            service_config=planner_config,
            joint_metadata=self._one_shot_ik.joint_metadata,
        )
        if self._one_shot_ik.base_frame != self._frames.base:
            raise ValueError(
                "one-shot IK base frame {!r} does not match scan base frame {!r}".format(
                    self._one_shot_ik.base_frame, self._frames.base
                )
            )
        if self._one_shot_ik.tool_frame != self._frames.tool:
            raise ValueError(
                "one-shot IK tool frame {!r} does not match scan tool frame {!r}".format(
                    self._one_shot_ik.tool_frame, self._frames.tool
                )
            )
        measured_topic = self._topic(
            self._mapping(
                kinematics_config.get("topics"), "services.kinematics.topics"
            ).get("measured_joint_state"),
            "services.kinematics.topics.measured_joint_state",
        )
        self.create_subscription(
            JointState,
            measured_topic,
            self._one_shot_measured_joint_callback,
            10,
        )

    def _one_shot_measured_joint_callback(self, message: JointState) -> None:
        if self._one_shot_ik is None:
            return
        try:
            joints = self._canonical_joint_positions_from_message(
                message, self._one_shot_ik.joint_names
            )
            now_s = time.monotonic()
            with self._state_lock:
                self._one_shot_ik.update_measured_joint_positions(
                    joints, received_at_monotonic_s=now_s
                )
                self._latest_measured_joint_positions = joints
                self._latest_measured_at_s = now_s
        except ValueError as error:
            self.get_logger().warning(f"One-shot IK feedback rejected: {error}")

    @staticmethod
    def _canonical_joint_positions_from_message(
        message: JointState, joint_names: Sequence[str]
    ) -> JointPositions:
        if message.name:
            if len(message.name) != len(message.position):
                raise ValueError("measured JointState name and position lengths differ")
            if len(set(message.name)) != len(message.name):
                raise ValueError("measured JointState contains duplicate joint names")
            by_name = dict(zip(message.name, message.position))
            missing = [name for name in joint_names if name not in by_name]
            if missing:
                raise ValueError(
                    "measured JointState is missing arm joints: {}".format(
                        ", ".join(missing)
                    )
                )
            return JointPositions(tuple(by_name[name] for name in joint_names))
        if len(message.position) != len(joint_names):
            raise ValueError(
                "unnamed measured JointState must contain exactly {} arm positions".format(
                    len(joint_names)
                )
            )
        return JointPositions(message.position)

    @staticmethod
    def _frame(value: Any, name: str) -> str:
        if not isinstance(value, str) or not value.strip() or value.startswith("/"):
            raise ValueError(f"{name} must be a non-empty relative TF frame")
        return value.strip()

    @staticmethod
    def _topic(value: Any, name: str) -> str:
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f"{name} must be a non-empty topic name")
        return value.strip()

    @classmethod
    def _load_config(cls, path: Path) -> Tuple[ScanFrames, Dict[str, ScanProfile]]:
        if not path.is_file():
            raise ValueError(f"scan_config does not exist: {path}")
        try:
            with path.open("r", encoding="utf-8") as stream:
                document = yaml.safe_load(stream)
        except yaml.YAMLError as error:
            raise ValueError(f"scan_config is invalid YAML: {path}") from error
        if not isinstance(document, dict):
            raise TypeError("scan_config must be a mapping")
        frames_document = cls._mapping(document.get("frames"), "frames")
        frames = ScanFrames(
            base=cls._frame(frames_document.get("base"), "frames.base"),
            workspace=cls._frame(frames_document.get("workspace"), "frames.workspace"),
            tool=cls._frame(frames_document.get("tool"), "frames.tool"),
            camera_optical=cls._frame(
                frames_document.get("camera_optical"), "frames.camera_optical"
            ),
        )
        trajectory = cls._mapping(document.get("trajectory"), "trajectory")
        cls._validate_ros_spherical_convention(
            cls._mapping(
                trajectory.get("spherical_convention"),
                "trajectory.spherical_convention",
            )
        )
        profiles_document = cls._mapping(trajectory.get("profiles"), "trajectory.profiles")
        default_settle = cls._nonnegative_float(
            cls._mapping_or_empty(document.get("execution"), "execution").get(
                "settle_time_s", 0.25
            ),
            "execution.settle_time_s",
        )
        profiles = {
            profile_id: cls._parse_profile(
                profile_id, profile_document, default_settle
            )
            for profile_id, profile_document in profiles_document.items()
        }
        if not profiles:
            raise ValueError("trajectory.profiles must contain at least one profile")
        if any(not isinstance(profile_id, str) or not profile_id for profile_id in profiles):
            raise ValueError("trajectory profile names must be non-empty strings")
        active_profile = trajectory.get("active_profile")
        if active_profile is not None and active_profile not in profiles:
            raise ValueError("trajectory.active_profile is not present in trajectory.profiles")
        if active_profile is not None:
            ordered_profiles = {str(active_profile): profiles.pop(str(active_profile))}
            ordered_profiles.update(profiles)
            profiles = ordered_profiles
        return frames, profiles

    @staticmethod
    def _validate_ros_spherical_convention(convention: Mapping[str, Any]) -> None:
        """Reject a config whose declared azimuth convention disagrees with ROS.

        Scan positions live in a REP-103 workspace frame (+X forward, +Y left,
        +Z up). Positive yaw about +Z therefore follows the right-hand rule:
        +X toward +Y, counter-clockwise when viewed from +Z. This node has no
        alternate convention switch; accepting one here would silently mirror
        every camera target.
        """
        expected = {
            "azimuth_zero_axis": "+x",
            "azimuth_positive_direction": "counter_clockwise_about_positive_z",
            "polar_zero_axis": "+z",
            "angle_unit": "degree",
        }
        for field_name, expected_value in expected.items():
            if convention.get(field_name) != expected_value:
                raise ValueError(
                    "trajectory.spherical_convention.{} must be {!r} for the "
                    "ROS right-handed workspace convention".format(
                        field_name, expected_value
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
        return NeugraspScanNode._mapping(value, name)

    @staticmethod
    def _nonnegative_float(value: Any, name: str) -> float:
        try:
            normalized = float(value)
        except (TypeError, ValueError) as error:
            raise TypeError(f"{name} must be numeric") from error
        if not math.isfinite(normalized) or normalized < 0.0:
            raise ValueError(f"{name} must be finite and non-negative")
        return normalized

    @classmethod
    def _parse_profile(
        cls, profile_id: Any, document: Any, default_settle: float
    ) -> ScanProfile:
        if not isinstance(profile_id, str) or not profile_id:
            raise ValueError("trajectory profile id must be non-empty")
        profile = cls._mapping(document, f"trajectory.profiles.{profile_id}")
        profile_type = profile.get("type")
        if profile_type == "paper_spiral":
            views = cls._paper_spiral_views(profile_id, profile)
        elif profile_type == "fixed_poses":
            views = cls._fixed_pose_views(profile_id, profile)
        else:
            raise ValueError(
                f"trajectory profile {profile_id} has unsupported type: {profile_type!r}; "
                "use paper_spiral or fixed_poses"
            )
        by_key = {view.key: view for view in views}
        if len(by_key) != len(views):
            raise ValueError(f"trajectory profile {profile_id} has duplicate view_key")
        capture_order = cls._ordered_view_keys(
            profile.get("capture_order"), views, profile_id, "capture_order"
        )
        model_order = cls._ordered_view_keys(
            profile.get("model_input_order"), views, profile_id, "model_input_order"
        )
        if set(capture_order) != set(by_key) or len(capture_order) != len(views):
            raise ValueError(f"trajectory profile {profile_id}.capture_order must cover every view")
        if set(model_order) != set(by_key) or len(model_order) != len(views):
            raise ValueError(
                f"trajectory profile {profile_id}.model_input_order must cover every view"
            )
        query_reference = profile.get("query_view_key")
        if not isinstance(query_reference, (str, int)) or isinstance(query_reference, bool):
            raise ValueError(
                f"trajectory profile {profile_id}.query_view_key must name one view"
            )
        query_token = str(query_reference).strip()
        if query_token in by_key:
            query_view_key = query_token
        elif query_token in {view.source_view_id for view in views}:
            query_view_key = next(
                view.key for view in views if view.source_view_id == query_token
            )
        else:
            raise ValueError(
                f"trajectory profile {profile_id}.query_view_key references unknown view: "
                f"{query_reference!r}"
            )
        version = profile.get("profile_version", 1)
        if not isinstance(version, int) or version <= 0:
            raise ValueError(f"trajectory profile {profile_id}.profile_version must be positive")
        settle = cls._nonnegative_float(
            profile.get("settle_time_s", default_settle),
            f"trajectory profile {profile_id}.settle_time_s",
        )
        return ScanProfile(
            profile_id=profile_id,
            version=version,
            views=tuple(views),
            capture_order=tuple(capture_order),
            model_input_order=tuple(model_order),
            query_view_key=query_view_key,
            default_settle_time_s=settle,
        )

    @classmethod
    def _paper_spiral_views(
        cls, profile_id: str, profile: Mapping[str, Any]
    ) -> List[ScanView]:
        polar = cls._float_list(profile.get("polar_deg"), f"{profile_id}.polar_deg")
        azimuth = cls._float_list(profile.get("azimuth_deg"), f"{profile_id}.azimuth_deg")
        radii_value = profile.get("radius_m")
        if isinstance(radii_value, (int, float)) and not isinstance(radii_value, bool):
            radii = [float(radii_value)] * len(polar)
        else:
            radii = cls._float_list(radii_value, f"{profile_id}.radius_m")
        if not polar or len(polar) != len(azimuth) or len(polar) != len(radii):
            raise ValueError(
                f"trajectory profile {profile_id} polar_deg, azimuth_deg and radius_m must have equal non-zero lengths"
            )
        if any(radius <= 0.0 for radius in radii):
            raise ValueError(f"trajectory profile {profile_id}.radius_m values must be positive")
        look_at = finite_vector(profile.get("look_at_m", [0.0, 0.0, 0.0]), "look_at_m", 3)
        source_ids = profile.get("source_view_ids", list(range(len(polar))))
        if not isinstance(source_ids, list) or len(source_ids) != len(polar):
            raise ValueError(f"trajectory profile {profile_id}.source_view_ids length must match views")
        raw_keys = profile.get("view_keys")
        if raw_keys is None:
            view_keys = [f"view_{index:02d}" for index in range(len(polar))]
        else:
            if not isinstance(raw_keys, list) or len(raw_keys) != len(polar):
                raise ValueError(
                    f"trajectory profile {profile_id}.view_keys length must match views"
                )
            if not all(isinstance(key, str) and key for key in raw_keys):
                raise ValueError(
                    f"trajectory profile {profile_id}.view_keys must be non-empty strings"
                )
            if len(set(raw_keys)) != len(raw_keys):
                raise ValueError(
                    f"trajectory profile {profile_id}.view_keys must be unique"
                )
            view_keys = list(raw_keys)
        views = []
        for index, (radius, theta_deg, phi_deg) in enumerate(zip(radii, polar, azimuth)):
            theta = math.radians(theta_deg)
            phi = math.radians(phi_deg)
            # Workspace follows REP-103: +X forward, +Y left, +Z up. Positive
            # azimuth follows the right-hand rule about +Z: +X -> +Y.
            position = (
                radius * math.sin(theta) * math.cos(phi),
                radius * math.sin(theta) * math.sin(phi),
                radius * math.cos(theta),
            )
            views.append(
                ScanView(
                    key=view_keys[index],
                    source_view_id=str(source_ids[index]),
                    camera_workspace=RigidTransform(
                        translation=position,
                        rotation=optical_look_at(position, look_at),
                    ),
                )
            )
        return views

    @classmethod
    def _fixed_pose_views(
        cls, profile_id: str, profile: Mapping[str, Any]
    ) -> List[ScanView]:
        values = profile.get("views")
        if not isinstance(values, list) or not values:
            raise ValueError(f"trajectory profile {profile_id}.views must be a non-empty list")
        views = []
        for index, item in enumerate(values):
            document = cls._mapping(item, f"trajectory profile {profile_id}.views[{index}]")
            key = document.get("view_key", f"view_{index:02d}")
            if not isinstance(key, str) or not key:
                raise ValueError(f"trajectory profile {profile_id}.views[{index}].view_key is invalid")
            position = finite_vector(
                document.get("position_m"),
                f"trajectory profile {profile_id}.views[{index}].position_m",
                3,
            )
            orientation = finite_vector(
                document.get("orientation_xyzw"),
                f"trajectory profile {profile_id}.views[{index}].orientation_xyzw",
                4,
            )
            views.append(
                ScanView(
                    key=key,
                    source_view_id=str(document.get("source_view_id", index)),
                    camera_workspace=RigidTransform(position, orientation),
                )
            )
        return views

    @staticmethod
    def _float_list(value: Any, name: str) -> List[float]:
        if not isinstance(value, list):
            raise TypeError(f"{name} must be a list")
        try:
            normalized = [float(item) for item in value]
        except (TypeError, ValueError) as error:
            raise TypeError(f"{name} must be numeric") from error
        if not all(math.isfinite(item) for item in normalized):
            raise ValueError(f"{name} must be finite")
        return normalized

    @staticmethod
    def _ordered_view_keys(
        raw_order: Any,
        views: Sequence[ScanView],
        profile_id: str,
        name: str,
    ) -> List[str]:
        if raw_order is None:
            return [view.key for view in views]
        if not isinstance(raw_order, list):
            raise TypeError(f"trajectory profile {profile_id}.{name} must be a list")
        by_index = {index: view.key for index, view in enumerate(views)}
        by_source = {view.source_view_id: view.key for view in views}
        by_key = {view.key: view.key for view in views}
        result = []
        for item in raw_order:
            if isinstance(item, int) and item in by_index:
                result.append(by_index[item])
                continue
            token = str(item)
            if token in by_key:
                result.append(by_key[token])
            elif token in by_source:
                result.append(by_source[token])
            else:
                raise ValueError(
                    f"trajectory profile {profile_id}.{name} references unknown view: {item!r}"
                )
        return result

    def _goal_callback(self, request):
        profile_id = str(request.profile_id).strip() or self._default_profile_id
        with self._state_lock:
            if self._goal_reserved or self._active_goal is not None:
                self.get_logger().warning("Scan goal rejected: another scan is active")
                return GoalResponse.REJECT
            if profile_id not in self._profiles:
                self.get_logger().warning(f"Scan goal rejected: unknown profile {profile_id}")
                return GoalResponse.REJECT
            self._goal_reserved = True
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        with self._state_lock:
            # A cancel can arrive in the tiny interval after the goal callback
            # accepted the sole scan but before execute_callback records its
            # handle.  There is only one reserved goal, so accept it as well.
            active_or_reserved = goal_handle is self._active_goal or self._goal_reserved
        return CancelResponse.ACCEPT if active_or_reserved else CancelResponse.REJECT

    def _execute_scan(self, goal_handle):
        result = ScanWorkspace.Result()
        profile_id = str(goal_handle.request.profile_id).strip() or self._default_profile_id
        profile = self._profiles[profile_id]
        self._publish_profile_visualization(profile)
        planned = self._pose_array(self._frames.base, [])
        measured = self._pose_array(self._frames.base, [])
        completed = []
        with self._state_lock:
            self._goal_reserved = False
            self._active_goal = goal_handle
        try:
            settle_time_s = float(goal_handle.request.settle_time_s)
            if settle_time_s < 0.0:
                settle_time_s = profile.default_settle_time_s
            elif not math.isfinite(settle_time_s):
                return self._abort(goal_handle, result, "invalid_goal", "settle_time_s must be finite")
            if goal_handle.request.execute_motion:
                if not self._wait_for_motion_server(goal_handle):
                    return self._finish_canceled_or_abort(
                        goal_handle,
                        result,
                        "motion_server_unavailable",
                        "{} action server is unavailable".format(
                            "FollowJointTrajectory"
                            if self._motion_planner == "one_shot_ik_joint_trajectory"
                            else "FollowCartesianTrajectory"
                        ),
                    )
            workspace_to_base = self._lookup_transform(
                self._frames.base, self._frames.workspace
            )
            tool_to_camera = self._lookup_transform(
                self._frames.tool, self._frames.camera_optical
            )
            view_by_key = {view.key: view for view in profile.views}
            total = len(profile.capture_order)
            for capture_index, view_key in enumerate(profile.capture_order):
                if goal_handle.is_cancel_requested:
                    return self._cancel(goal_handle, result, planned, measured, completed)
                view = view_by_key[view_key]
                base_camera = compose(workspace_to_base, view.camera_workspace)
                base_tool = compose(base_camera, inverse(tool_to_camera))
                planned.poses.append(self._pose_from_transform(base_camera))
                self._planned_poses_publisher.publish(planned)
                planned_camera = self._pose_stamped(self._frames.base, base_camera)
                planned_tool = self._pose_stamped(self._frames.base, base_tool)
                self._feedback(
                    goal_handle,
                    view.key,
                    capture_index,
                    total,
                    "planned",
                    planned_camera,
                    planned_tool,
                    PoseStamped(),
                )
                if goal_handle.request.execute_motion:
                    child_result = self._run_motion_view(
                        goal_handle, planned_tool, view, capture_index, total
                    )
                    if child_result is None:
                        return self._cancel(goal_handle, result, planned, measured, completed)
                    if not child_result.succeeded:
                        detail = "{}: {}".format(
                            child_result.failure_reason, child_result.detail
                        )
                        return self._abort(
                            goal_handle,
                            result,
                            "cartesian_execution_failed",
                            f"view {view.key} failed: {detail}",
                            planned,
                            measured,
                            completed,
                        )
                    if not self._wait_settle(goal_handle, settle_time_s):
                        return self._cancel(goal_handle, result, planned, measured, completed)
                    try:
                        measured_camera = self._lookup_transform(
                            self._frames.base, self._frames.camera_optical
                        )
                    except TransformException as error:
                        return self._abort(
                            goal_handle,
                            result,
                            "measured_camera_tf_unavailable",
                            f"view {view.key}: {error}",
                            planned,
                            measured,
                            completed,
                        )
                    measured_pose = self._pose_stamped(self._frames.base, measured_camera)
                    measured.poses.append(measured_pose.pose)
                    self._measured_poses_publisher.publish(measured)
                else:
                    measured_pose = PoseStamped()
                completed.append(view.key)
                self._feedback(
                    goal_handle,
                    view.key,
                    capture_index,
                    total,
                    "captured_placeholder" if goal_handle.request.capture_enabled else "complete",
                    planned_camera,
                    planned_tool,
                    measured_pose,
                )
            result.succeeded = True
            result.failure_reason = ""
            result.detail = (
                f"profile {profile.profile_id} v{profile.version} completed "
                f"{len(completed)} views; capture is a placeholder until a camera node is integrated"
                if goal_handle.request.capture_enabled
                else f"profile {profile.profile_id} v{profile.version} completed {len(completed)} views"
            )
            result.completed_view_count = len(completed)
            result.completed_view_keys = completed
            result.planned_camera_poses = planned
            result.measured_camera_poses = measured
            goal_handle.succeed()
            return result
        except TransformException as error:
            return self._abort(
                goal_handle,
                result,
                "frame_transform_failed",
                str(error),
                planned,
                measured,
                completed,
            )
        except (TypeError, ValueError, KeyError) as error:
            return self._abort(
                goal_handle,
                result,
                "scan_configuration_error",
                str(error),
                planned,
                measured,
                completed,
            )
        except Exception as error:  # noqa: BLE001 - leave action ownership explicit.
            self._request_motion_cancel()
            return self._abort(
                goal_handle,
                result,
                "scan_runtime_error",
                str(error),
                planned,
                measured,
                completed,
            )
        finally:
            # Scan targets are phase-local visualization, never a persistent
            # scene relation. Clear retained markers before trial Predict.
            self._clear_profile_visualization()
            with self._state_lock:
                if goal_handle is self._active_goal:
                    self._active_goal = None
                self._active_child_goal = None
                self._goal_reserved = False

    def _wait_for_motion_server(self, goal_handle) -> bool:
        deadline = time.monotonic() + self._server_wait_timeout_s
        client = (
            self._joint_client
            if self._motion_planner == "one_shot_ik_joint_trajectory"
            else self._cartesian_client
        )
        if client is None:
            return False
        while rclpy.ok() and time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                return False
            if client.wait_for_server(timeout_sec=0.1):
                return True
        return False

    def _run_motion_view(
        self,
        scan_goal,
        target_pose: PoseStamped,
        view: ScanView,
        capture_index: int,
        total: int,
    ):
        if self._motion_planner == "one_shot_ik_joint_trajectory":
            return self._run_one_shot_joint_view(
                scan_goal, target_pose, view, capture_index, total
            )
        return self._run_cartesian_view(
            scan_goal, target_pose, view, capture_index, total
        )

    def _run_one_shot_joint_view(
        self,
        scan_goal,
        target_pose: PoseStamped,
        view: ScanView,
        capture_index: int,
        total: int,
    ):
        """Solve one endpoint IK, plan in joint space, then execute it.

        Unlike CartesianSequentialCLIK, this path never asks the IK solver to
        converge at intermediate Cartesian waypoints. It still rejects an
        endpoint that is outside hard limits or the configured safety margin.
        """
        if self._one_shot_ik is None or self._joint_trajectory_planner is None:
            return ScanMotionResult(
                False, "joint_path_unavailable", "one-shot joint path is not configured"
            )
        if self._joint_client is None:
            return ScanMotionResult(
                False, "joint_action_unavailable", "FollowJointTrajectory client is unavailable"
            )
        try:
            with self._joint_planning_lock:
                sdk_target = self._sdk_pose_from_message(target_pose)
                self._one_shot_ik.set_target_pose(sdk_target)
                step = self._one_shot_ik.step(now_monotonic_s=time.monotonic())
                ik_result = step.ik_result
                if (
                    ik_result is None
                    or not ik_result.converged
                    or step.joint_goal is None
                    or step.measured_joint_positions is None
                    or not step.measured_state_fresh
                ):
                    reason = (
                        ik_result.failure_reason.value
                        if ik_result is not None and ik_result.failure_reason is not None
                        else "ik_failed"
                    )
                    detail = (
                        ik_result.detail
                        if ik_result is not None
                        else "one-shot IK returned no result"
                    )
                    if ik_result is not None:
                        detail = (
                            "{}; active_joint_limits={}; "
                            "minimum_joint_limit_margin_rad={:.9f}; "
                            "position_residual_m={:.9f}; "
                            "orientation_residual_rad={:.9f}"
                        ).format(
                            detail,
                            ",".join(ik_result.active_joint_limits) or "none",
                            ik_result.minimum_joint_limit_margin_rad,
                            ik_result.position_residual_m,
                            ik_result.orientation_residual_rad,
                        )
                    return ScanMotionResult(False, reason, detail)
                plan = self._joint_trajectory_planner.plan_joint_motion(
                    q_start=step.measured_joint_positions,
                    q_goal=step.joint_goal,
                )
            if not plan.succeeded or plan.trajectory is None:
                reason = plan.failure_reason.value if plan.failure_reason is not None else "joint_plan_failed"
                return ScanMotionResult(False, reason, plan.detail)
            child_goal = FollowJointTrajectory.Goal()
            child_goal.trajectory = self._joint_trajectory_to_ros(plan.trajectory)
        except (TypeError, ValueError) as error:
            return ScanMotionResult(False, "one_shot_ik_invalid", str(error))

        send_future = self._joint_client.send_goal_async(child_goal)
        send_future.add_done_callback(
            lambda future: self._cancel_late_child_if_parent_canceled(future, scan_goal)
        )
        if not self._wait_future(send_future, scan_goal):
            self._cancel_late_child_if_parent_canceled(send_future, scan_goal)
            return None
        child_handle = send_future.result()
        if child_handle is None or not child_handle.accepted:
            return ScanMotionResult(
                False, "goal_rejected", "FollowJointTrajectory rejected the view"
            )
        with self._state_lock:
            self._active_child_goal = child_handle
        self._feedback(
            scan_goal,
            view.key,
            capture_index,
            total,
            "executing_joint_trajectory",
            PoseStamped(),
            target_pose,
            PoseStamped(),
        )
        result_future = child_handle.get_result_async()
        if not self._wait_future(result_future, scan_goal):
            return None
        wrapped = result_future.result()
        with self._state_lock:
            self._active_child_goal = None
        if wrapped is None:
            return ScanMotionResult(
                False, "joint_action_no_result", "FollowJointTrajectory returned no result"
            )
        result = wrapped.result
        succeeded = result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        return ScanMotionResult(
            succeeded,
            "" if succeeded else "follow_joint_trajectory_failed",
            str(result.error_string),
        )

    def _sdk_pose_from_message(self, message: PoseStamped) -> SdkPose:
        if message.header.frame_id != self._frames.base:
            raise ValueError(
                "one-shot IK target must be expressed in {!r}, got {!r}".format(
                    self._frames.base, message.header.frame_id
                )
            )
        values = (
            message.pose.position.x,
            message.pose.position.y,
            message.pose.position.z,
            message.pose.orientation.x,
            message.pose.orientation.y,
            message.pose.orientation.z,
            message.pose.orientation.w,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("one-shot IK target contains non-finite values")
        return SdkPose(
            position=values[:3], orientation=values[3:]
        )

    @staticmethod
    def _joint_trajectory_to_ros(trajectory):
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

    def _run_cartesian_view(
        self,
        scan_goal,
        target_pose: PoseStamped,
        view: ScanView,
        capture_index: int,
        total: int,
    ):
        child_goal = FollowCartesianTrajectory.Goal()
        child_goal.target_pose = target_pose
        child_goal.requested_duration = Duration()
        child_goal.path_mode = FollowCartesianTrajectory.Goal.PATH_DEFAULT
        child_goal.task_mode = FollowCartesianTrajectory.Goal.TASK_DEFAULT
        child_goal.time_scaling_mode = FollowCartesianTrajectory.Goal.TIME_DEFAULT
        child_goal.speed_scale = 0.0
        child_goal.max_translation_step_m = 0.0
        child_goal.max_rotation_step_rad = 0.0
        send_future = self._cartesian_client.send_goal_async(child_goal)
        # The parent may be canceled before the action server answers the goal
        # request.  Keep a completion callback in addition to the synchronous
        # path below: otherwise a late accepted child could start moving after
        # ScanWorkspace has already returned canceled.
        send_future.add_done_callback(
            lambda future: self._cancel_late_child_if_parent_canceled(future, scan_goal)
        )
        if not self._wait_future(send_future, scan_goal):
            self._cancel_late_child_if_parent_canceled(send_future, scan_goal)
            return None
        child_handle = send_future.result()
        if child_handle is None or not child_handle.accepted:
            class RejectedResult:
                succeeded = False
                failure_reason = "goal_rejected"
                detail = "FollowCartesianTrajectory rejected the view"
            return RejectedResult()
        with self._state_lock:
            self._active_child_goal = child_handle
        self._feedback(
            scan_goal,
            view.key,
            capture_index,
            total,
            "executing",
            PoseStamped(),
            target_pose,
            PoseStamped(),
        )
        result_future = child_handle.get_result_async()
        if not self._wait_future(result_future, scan_goal):
            return None
        wrapped = result_future.result()
        with self._state_lock:
            self._active_child_goal = None
        return wrapped.result if wrapped is not None else None

    def _cancel_late_child_if_parent_canceled(self, future, scan_goal) -> None:
        """Cancel an accepted child that arrived after parent cancellation."""
        if not future.done() or not scan_goal.is_cancel_requested:
            return
        try:
            child_handle = future.result()
        except Exception as error:  # noqa: BLE001 - action transport failed already.
            self.get_logger().warning(
                f"Unable to inspect late motion child goal response: {error}"
            )
            return
        if child_handle is None or not child_handle.accepted:
            return
        try:
            child_handle.cancel_goal_async()
        except Exception as error:  # noqa: BLE001 - executor cancel remains a fallback.
            self.get_logger().warning(f"Unable to cancel late motion child goal: {error}")
        self._request_motion_cancel()

    def _wait_future(self, future, scan_goal) -> bool:
        completed = threading.Event()
        future.add_done_callback(lambda _: completed.set())
        cancellation_requested = False
        while rclpy.ok() and not completed.wait(0.05):
            if scan_goal.is_cancel_requested and not cancellation_requested:
                cancellation_requested = True
                self._cancel_active_child()
        if scan_goal.is_cancel_requested:
            self._cancel_active_child()
            return False
        return completed.is_set()

    def _cancel_active_child(self) -> None:
        with self._state_lock:
            child_goal = self._active_child_goal
        if child_goal is not None:
            try:
                child_goal.cancel_goal_async()
            except Exception as error:  # noqa: BLE001 - fallback is still required.
                self.get_logger().warning(f"Unable to cancel motion child action: {error}")
        self._request_motion_cancel()

    def _request_motion_cancel(self) -> None:
        if self._motion_cancel_client.service_is_ready():
            try:
                self._motion_cancel_client.call_async(Trigger.Request())
            except Exception as error:  # noqa: BLE001
                self.get_logger().warning(f"Unable to request motion cancel: {error}")

    @staticmethod
    def _wait_settle(goal_handle, settle_time_s: float) -> bool:
        deadline = time.monotonic() + settle_time_s
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                return False
            threading.Event().wait(min(0.05, deadline - time.monotonic()))
        return not goal_handle.is_cancel_requested

    def _lookup_transform(self, target_frame: str, source_frame: str) -> RigidTransform:
        transform = self._tf_buffer.lookup_transform(target_frame, source_frame, Time())
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return RigidTransform(
            translation=(translation.x, translation.y, translation.z),
            rotation=(rotation.x, rotation.y, rotation.z, rotation.w),
        )

    def _publish_profile_visualization(self, profile: ScanProfile) -> None:
        ordered = {view.key: view for view in profile.views}
        views = [ordered[key] for key in profile.capture_order]
        pose_array = self._pose_array(
            self._frames.workspace,
            [self._pose_from_transform(view.camera_workspace) for view in views],
        )
        self._scan_views_publisher.publish(pose_array)
        self._scan_markers_publisher.publish(self._scan_markers(profile, views))

    def _scan_markers(
        self, profile: ScanProfile, views: Sequence[ScanView]
    ) -> MarkerArray:
        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        route = self._marker("scan_route", 0, Marker.LINE_STRIP, stamp)
        route.scale.x = 0.006
        route.color.r = 0.20
        route.color.g = 0.85
        route.color.b = 1.0
        route.color.a = 0.95
        route.points = [self._point(view.camera_workspace.translation) for view in views]
        marker_array.markers.append(route)
        for index, view in enumerate(views):
            frustum = self._marker("scan_frusta", index, Marker.LINE_LIST, stamp)
            frustum.scale.x = 0.004
            frustum.color.r = 0.95
            frustum.color.g = 0.25
            frustum.color.b = 0.95
            frustum.color.a = 0.9
            frustum.points = self._frustum_points(view.camera_workspace)
            marker_array.markers.append(frustum)
            text = self._marker("scan_labels", index, Marker.TEXT_VIEW_FACING, stamp)
            text.pose.position.x = view.camera_workspace.translation[0]
            text.pose.position.y = view.camera_workspace.translation[1]
            text.pose.position.z = view.camera_workspace.translation[2] + 0.025
            text.pose.orientation.w = 1.0
            text.scale.z = 0.028
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            text.text = f"{index}: {view.key} ({view.source_view_id})"
            marker_array.markers.append(text)
            marker_array.markers.append(self._view_axes_marker(index, view.camera_workspace, stamp))
        return marker_array

    def _view_axes_marker(self, marker_id: int, pose: RigidTransform, stamp) -> Marker:
        """Draw REP-103 optical-frame axes at one planned scan camera pose."""
        from .math3d import rotate_vector

        marker = self._marker("scan_view_axes", marker_id, Marker.LINE_LIST, stamp)
        marker.scale.x = 0.0035
        origin = self._point(pose.translation)
        axes = (
            ((1.0, 0.0, 0.0), (1.0, 0.0, 0.0, 1.0)),
            ((0.0, 1.0, 0.0), (0.0, 1.0, 0.0, 1.0)),
            ((0.0, 0.0, 1.0), (0.0, 0.35, 1.0, 1.0)),
        )
        for axis, rgba in axes:
            rotated = rotate_vector(pose.rotation, axis)
            marker.points.extend((origin, self._point((
                pose.translation[0] + 0.04 * rotated[0],
                pose.translation[1] + 0.04 * rotated[1],
                pose.translation[2] + 0.04 * rotated[2],
            ))))
            marker.colors.extend((self._color(rgba), self._color(rgba)))
        return marker

    def _clear_profile_visualization(self) -> None:
        self._scan_views_publisher.publish(self._pose_array(self._frames.workspace, []))
        clear = Marker()
        clear.action = Marker.DELETEALL
        self._scan_markers_publisher.publish(MarkerArray(markers=[clear]))

    def _marker(self, namespace: str, marker_id: int, marker_type: int, stamp):
        marker = Marker()
        marker.header.frame_id = self._frames.workspace
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    @staticmethod
    def _point(values: Sequence[float]) -> Point:
        point = Point()
        point.x, point.y, point.z = values
        return point

    @staticmethod
    def _color(values: Sequence[float]) -> ColorRGBA:
        color = ColorRGBA()
        color.r, color.g, color.b, color.a = values
        return color

    def _frustum_points(self, pose: RigidTransform) -> List[Point]:
        depth = 0.06
        half_width = 0.028
        corners_camera = (
            (-half_width, -half_width, depth),
            (half_width, -half_width, depth),
            (half_width, half_width, depth),
            (-half_width, half_width, depth),
        )
        from .math3d import rotate_vector

        origin = self._point(pose.translation)
        corners = []
        for corner in corners_camera:
            rotated = rotate_vector(pose.rotation, corner)
            corners.append(
                self._point(
                    (
                        pose.translation[0] + rotated[0],
                        pose.translation[1] + rotated[1],
                        pose.translation[2] + rotated[2],
                    )
                )
            )
        points = []
        for corner in corners:
            points.extend((origin, corner))
        for index, corner in enumerate(corners):
            points.extend((corner, corners[(index + 1) % len(corners)]))
        return points

    def _feedback(
        self,
        goal_handle,
        view_key: str,
        capture_index: int,
        total: int,
        stage: str,
        planned_camera: PoseStamped,
        planned_tool: PoseStamped,
        measured_camera: PoseStamped,
    ) -> None:
        feedback = ScanWorkspace.Feedback()
        feedback.view_key = view_key
        feedback.capture_index = capture_index
        feedback.view_count = total
        feedback.stage = stage
        feedback.progress = float(capture_index) / float(total) if total else 0.0
        feedback.planned_camera_pose = planned_camera
        feedback.planned_tool_pose = planned_tool
        feedback.measured_camera_pose = measured_camera
        goal_handle.publish_feedback(feedback)

    def _abort(
        self,
        goal_handle,
        result,
        reason: str,
        detail: str,
        planned: Optional[PoseArray] = None,
        measured: Optional[PoseArray] = None,
        completed: Optional[Iterable[str]] = None,
    ):
        result.succeeded = False
        result.failure_reason = reason
        result.detail = detail
        result.completed_view_keys = list(completed or [])
        result.completed_view_count = len(result.completed_view_keys)
        result.planned_camera_poses = planned or self._pose_array(self._frames.base, [])
        result.measured_camera_poses = measured or self._pose_array(self._frames.base, [])
        goal_handle.abort()
        return result

    def _cancel(self, goal_handle, result, planned, measured, completed):
        self._cancel_active_child()
        result.succeeded = False
        result.failure_reason = "canceled"
        result.detail = "ScanWorkspace canceled"
        result.completed_view_keys = list(completed)
        result.completed_view_count = len(completed)
        result.planned_camera_poses = planned
        result.measured_camera_poses = measured
        goal_handle.canceled()
        return result

    def _finish_canceled_or_abort(self, goal_handle, result, reason: str, detail: str):
        if goal_handle.is_cancel_requested:
            return self._cancel(
                goal_handle,
                result,
                self._pose_array(self._frames.base, []),
                self._pose_array(self._frames.base, []),
                [],
            )
        return self._abort(goal_handle, result, reason, detail)

    def _pose_array(self, frame_id: str, poses: Sequence[Pose]) -> PoseArray:
        message = PoseArray()
        message.header.frame_id = frame_id
        message.header.stamp = self.get_clock().now().to_msg()
        message.poses = list(poses)
        return message

    @staticmethod
    def _pose_from_transform(transform: RigidTransform) -> Pose:
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = transform.translation
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = transform.rotation
        return pose

    def _pose_stamped(self, frame_id: str, transform: RigidTransform) -> PoseStamped:
        message = PoseStamped()
        message.header.frame_id = frame_id
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose = self._pose_from_transform(transform)
        return message


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = NeugraspScanNode()
        executor = MultiThreadedExecutor()
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
