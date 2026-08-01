# MyArm Cartesian trajectory

`myarm_cartesian_trajectory` turns a Cartesian TCP target into a fully
validated joint trajectory using the SDK's sequential CLIK planner. It is
strictly plan/preview-only: it owns no robot adapter, has no driver setpoint
publisher, and does not submit trajectories to motion execution.

The action is `/myarm/plan_cartesian_trajectory`
(`myarm_interfaces/action/PlanCartesianTrajectory`). Its target is a
`geometry_msgs/PoseStamped`; a pose in another TF frame is transformed to the
configured `base_link`. Before accepting a plan, the node requires fresh
canonical feedback on `/myarm/state/joint_state`.

The zero-valued `PATH_DEFAULT`, `TASK_DEFAULT`, and `TIME_DEFAULT` fields keep
the selected Cartesian adapter profile; explicit action values are overrides.

Successful plans publish:

- `/myarm/cartesian_trajectory/reference_path` (`nav_msgs/Path`) for RViz.
- `/myarm/cartesian_trajectory/joint_preview`
  (`trajectory_msgs/JointTrajectory`) for preview or an explicit later
  application handoff.
- `/myarm/cartesian_trajectory/diagnostics`.

The optional preview-player subscribes to the configured
`services.cartesian_trajectory_planner.topics.joint_preview` topic unless its
`joint_preview_topic` parameter is explicitly overridden. It samples the same
SDK interpolation kernel at 5 Hz, publishes continuous position and velocity
state (the standard `JointState` message has no acceleration field), and keeps
republishing the final synthetic state for late-starting
`robot_state_publisher`/RViz consumers. The output adds a closed
`left_gripper_joint = 0.0` solely to complete the baseline URDF; it does not
issue a gripper command. Its default output is private so it cannot contend
with a real driver. Only the dedicated `*_cartesian_preview.launch.py`
launches route that output to `/joint_states`, after remapping the driver
visualisation stream away.

All Cartesian nodes accept the same SDK-relative `services_config` parameter
(default `service/config/services.yaml`). The Cartesian launch passes the same
empty-or-explicit `joint_preview_topic` override to both planner and player,
so a runtime-topic override cannot disconnect preview playback from the plan
publisher.

There is no collision checking in this phase. A successful Cartesian plan is
not authorization to execute physical motion.

For integration with the existing executor, use
`myarm_bringup myarm_cartesian_fake_execution.launch.py` with the checked-in
`FakeRobotArm` configuration. The plan action still never handoffs
automatically. That launch additionally enables the executor-owned
`/myarm/follow_cartesian_trajectory` action, which performs its own fresh-state
plan + preflight + execution through the normal private driver boundary. It is
restricted to the fake adapter in this phase.
