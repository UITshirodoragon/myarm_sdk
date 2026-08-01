# MyArm motion execution

`myarm_motion_execution` is the ROS 2 boundary for the SDK's
`JointTrajectoryPlannerService` and `MotionExecutionService`. It owns neither a
serial connection nor a robot adapter.

```text
/myarm/command/joint_goal + fresh /myarm/state/joint_state
  → minimum-jerk planner
  → validated q/qdot/qddot preview
  → monotonic-time executor
  → /myarm/internal/driver_joint_setpoint
  → myarm_robot_driver
```

The driver is the only fake/physical robot owner. This node never calls
`pymycobot`, never creates `RobotArmService`, and never publishes `/joint_states`.

Default interfaces:

- `/myarm/command/joint_goal` (`sensor_msgs/msg/JointState`): a six-joint
  endpoint goal in canonical URDF order. The node plans it from fresh measured
  feedback and rejects it while another motion is active.
- `/myarm/follow_joint_trajectory` (`control_msgs/action/FollowJointTrajectory`):
  full externally supplied trajectory. It must use exactly the canonical joint
  names, begin at `t=0`, include q/qdot/qddot at every point, satisfy all
  limits, and start close to the fresh measured q.
- `/myarm/follow_cartesian_trajectory`
  (`myarm_interfaces/action/FollowCartesianTrajectory`): optional action
  available only when launch passes `enable_cartesian_execution:=true`. It is
  restricted to `FakeRobotArm` in the current phase. The executor transforms
  the target to `base_link`, requires fresh measured q, invokes the isolated
  sequential-CLIK planner, rechecks q0, then executes the resulting trajectory
  through the same FSM and private driver topic as `FollowJointTrajectory`.
- `/myarm/cartesian_execution/reference_path` (`nav_msgs/msg/Path`): latched
  reference path published only after Cartesian preflight accepts execution.
- `/myarm/joint_trajectory/preview` (`trajectory_msgs/msg/JointTrajectory`): the
  complete validated trajectory generated for a `joint_goal` or accepted from
  the action.
- `/myarm/internal/driver_joint_setpoint` (`sensor_msgs/msg/JointState`):
  private desired q/qdot stream for the driver. `JointState.effort` is not used
  to carry acceleration.
- `/myarm/motion_execution/diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`):
  planner result, state, feedback freshness, progress, timing and tracking
  information.
- `/myarm/motion_execution/cancel` and `/myarm/motion_execution/reset`
  (`std_srvs/srv/Trigger`): explicit controls for topic-driven motion. Reset is
  required after `holding` or `fault` before a new public goal is accepted.

The node runs at 5 Hz by default. Both motion actions wait on a completion
event while a separate timer advances the executor in a multi-threaded ROS
executor; neither uses `time.sleep` or owns a duplicate control loop.

`FollowCartesianTrajectory` is deliberately unavailable by default,
unavailable with `MyArmM750RobotArm`, and has no collision checking. It is an
integration path for `myarm_cartesian_fake_execution.launch.py`, not
authorization for physical Cartesian motion.

The module-local profile
`plugin_adapter/trajectory/config/minimum_jerk_joint_trajectory_planner.yaml` contains
the acceleration limits and default time-scaling mode. URDF remains the source
of truth for joint order and position/velocity limits.
