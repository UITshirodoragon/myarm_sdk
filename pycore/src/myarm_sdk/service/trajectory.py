"""Joint trajectory-planning capability."""

from __future__ import annotations

from typing import Any, Mapping, Optional, Sequence

from myarm_sdk.core import (
    JointMetadata,
    JointMotionLimits,
    JointPositions,
    TimeScalingPolicy,
    TrajectoryPlanningRequest,
    TrajectoryPlanningResult,
    load_sdk_yaml,
)
from myarm_sdk.core.validation import require_enabled
from myarm_sdk.plugin_adapter.trajectory import MinimumJerkJointTrajectoryAdapter
from myarm_sdk.port_interface import TrajectoryPlannerInterface


class TrajectoryPlannerService:
    """Expose one pure trajectory-planning port to application/ROS callers."""

    def __init__(
        self,
        planner: TrajectoryPlannerInterface,
        motion_limits: Optional[JointMotionLimits] = None,
        default_time_scaling: Optional[TimeScalingPolicy] = None,
    ) -> None:
        self._planner = planner
        self._motion_limits = motion_limits
        self._default_time_scaling = default_time_scaling or TimeScalingPolicy()

    @classmethod
    def from_config(
        cls,
        service_config: Mapping[str, Any],
        joint_metadata: Sequence[JointMetadata],
    ) -> TrajectoryPlannerService:
        """Create the configured minimum-jerk planner without ROS imports.

        The shared runtime config selects the adapter and this method loads its
        module-local profile.  Joint metadata remains injected from the
        authoritative robot URDF rather than being duplicated in YAML.
        """
        require_enabled(service_config, "trajectory_planner")
        if service_config.get("plugin_adapter") != "minimum_jerk_joint":
            raise ValueError(
                "Only the minimum_jerk_joint trajectory plugin adapter is available"
            )
        adapter_config = load_sdk_yaml(str(service_config["plugin_config"]))
        if adapter_config.get("plugin_adapter") != "minimum_jerk_joint":
            raise ValueError(
                "Trajectory plugin config must select minimum_jerk_joint"
            )
        motion_limits = JointMotionLimits(
            joint_metadata=joint_metadata,
            acceleration_limits_rad_s2=cls._acceleration_limits(
                adapter_config.get("acceleration_limits_rad_s2"), joint_metadata
            ),
        )
        default_time_scaling_config = adapter_config.get(
            "default_time_scaling", {}
        )
        if not isinstance(default_time_scaling_config, dict):
            raise TypeError("default_time_scaling must be a mapping")
        default_time_scaling = TimeScalingPolicy(
            mode=default_time_scaling_config.get(
                "mode", TimeScalingPolicy().mode.value
            ),
            requested_duration_s=default_time_scaling_config.get(
                "requested_duration_s"
            ),
            speed_scale=default_time_scaling_config.get("speed_scale"),
            sample_period_s=float(adapter_config["sample_period_s"]),
        )
        return cls(
            planner=MinimumJerkJointTrajectoryAdapter(),
            motion_limits=motion_limits,
            default_time_scaling=default_time_scaling,
        )

    @property
    def motion_limits(self) -> Optional[JointMotionLimits]:
        """Return configured limits, or ``None`` for manually injected services."""
        return self._motion_limits

    @property
    def default_time_scaling(self) -> TimeScalingPolicy:
        """Return the service-level policy used by convenience planning calls."""
        return self._default_time_scaling

    def plan(self, request: TrajectoryPlanningRequest) -> TrajectoryPlanningResult:
        """Plan from a caller-owned fresh measured state to a joint goal."""
        return self._planner.plan(request)

    def plan_joint_motion(
        self,
        q_start: JointPositions,
        q_goal: JointPositions,
        time_scaling: Optional[TimeScalingPolicy] = None,
        motion_limits: Optional[JointMotionLimits] = None,
    ) -> TrajectoryPlanningResult:
        """Convenience form for callers that do not need a request object."""
        resolved_motion_limits = motion_limits or self._motion_limits
        if resolved_motion_limits is None:
            raise ValueError(
                "motion_limits must be supplied when the service is not configured"
            )
        return self.plan(
            TrajectoryPlanningRequest(
                q_start=q_start,
                q_goal=q_goal,
                motion_limits=resolved_motion_limits,
                time_scaling=time_scaling or self._default_time_scaling,
            )
        )

    @staticmethod
    def _acceleration_limits(
        value: Any, joint_metadata: Sequence[JointMetadata]
    ) -> Sequence[float]:
        """Read named acceleration values in the canonical URDF joint order."""
        if not isinstance(value, dict):
            raise TypeError("acceleration_limits_rad_s2 must be a mapping")
        names = tuple(metadata.name for metadata in joint_metadata)
        missing = tuple(name for name in names if name not in value)
        unexpected = tuple(name for name in value if name not in names)
        if missing or unexpected:
            details = []
            if missing:
                details.append("missing {}".format(", ".join(missing)))
            if unexpected:
                details.append("unexpected {}".format(", ".join(unexpected)))
            raise ValueError(
                "acceleration_limits_rad_s2 names must match joint metadata: {}".format(
                    "; ".join(details)
                )
            )
        return tuple(float(value[name]) for name in names)
