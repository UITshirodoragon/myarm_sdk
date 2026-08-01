# MyArm M750 robot driver

`myarm_robot_driver` is the ROS 2 boundary for
`myarm_sdk.service.RobotArmService`.  It does not import `pymycobot` or any
other robot-vendor SDK.

The service configuration is read from the SDK's single
`service/config/services.yaml` file.  Its `services.robot_arm` section selects
the fake or physical adapter and configures all ROS names.

The executable is:

```bash
ros2 run myarm_robot_driver myarm_robot_driver_node
```

Topics and services are configured rather than hard-coded.  With the standard
configuration they are:

- `/myarm/internal/driver_joint_setpoint` (`sensor_msgs/msg/JointState`): a
  private, already time-parameterized setpoint stream from
  `myarm_motion_execution`. It is not a public direct-joint command API.
  Named inputs are remapped into the canonical URDF order; malformed,
  duplicate, or incomplete inputs are rejected.
- `/myarm/state/joint_state` (`sensor_msgs/msg/JointState`): authoritative
  feedback in canonical model-space radians.
- `/joint_states` (`sensor_msgs/msg/JointState`): optional actual-feedback
  message for `robot_state_publisher` and RViz. It contains the six physical
  arm joints plus a virtual `left_gripper_joint=0.0`; the right gripper is a
  URDF mimic joint. Neither gripper value enters `RobotArmService`.
- `/myarm/robot/diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`):
  connection, power, feedback freshness and command information.
- `/myarm/robot/stop`, `/myarm/robot/power_on`, `/myarm/robot/power_off`
  (`std_srvs/srv/Trigger`): explicit lifecycle actions.

The node connects once on startup but never powers on or commands motion
implicitly. `RobotArmService.accepts_execution_setpoints` decides whether the
node creates the internal subscription. In the physical profile this remains
false until the explicit `transport.allow_physical_motion: true` opt-in;
feedback and RViz publication continue to work when motion is off.

The node does not plan trajectories, queue public goals or own action
preemption. It polls `read_feedback()` and forwards at most the newest private
setpoint through `send_joint_setpoint()`. This keeps exactly one serial owner
for both `FakeRobotArm` and `MyArmM750RobotArm`.
