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

- `/myarm/internal/driver_joint_setpoint`
  (`myarm_interfaces/msg/DriverJointSetpoint`): private, epoch-bound
  setpoints from `myarm_motion_execution`, never a public direct-joint API.
  A stop, stale feedback or transport fault increments the driver safety epoch,
  so delayed messages from the previous epoch are rejected.
- `/myarm/state/joint_state` (`sensor_msgs/msg/JointState`): authoritative
  feedback in canonical model-space radians.
- `/joint_states` (`sensor_msgs/msg/JointState`): optional actual-feedback
  message for `robot_state_publisher` and RViz. When gripper feedback is
  available it includes `left_gripper_joint = opening_width_m / 2`; the right
  jaw remains a URDF mimic joint.
- `/myarm/gripper/command` (`std_msgs/msg/Float64`): total fingertip opening
  in metres in `[0.0, 0.08]`, handled by the same `RobotArmService` and serial
  owner as the arm. `/myarm/gripper/state` publishes the equivalent left-jaw
  coordinate for visualization.
- `/myarm/robot/diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`):
  connection, power, feedback freshness and command information.
- `/myarm/robot/stop`, `/myarm/robot/rearm`, `/myarm/robot/power_on`,
  `/myarm/robot/power_off` (`std_srvs/srv/Trigger`): explicit lifecycle and
  safety actions. A physical driver starts **disarmed**; re-arm requires fresh
  connected feedback. `/myarm/robot/safety_state` reports the latched state
  and safety epoch.

The node connects once on startup but never powers on or commands arm motion
implicitly. `RobotArmService.accepts_execution_setpoints` decides whether the
node creates the internal subscription. In the physical profile this remains
false until the explicit `transport.allow_physical_motion: true` opt-in; even
then an operator must re-arm after fresh feedback. Feedback and RViz continue
to work when motion is off. The vendor API has no verified independent
gripper-stop operation, so physical gripper actuation has its own default-false
`gripper.allow_physical_actuation` gate and must be hardware-tested before use.

## Gripper commands

`/myarm/gripper/command` receives the **total opening between the two
fingertips**, expressed in metres. The valid inclusive range is `0.0` to
`0.08`; it is not the position of one URDF jaw.

Open the gripper fully:

```bash
ros2 topic pub --once /myarm/gripper/command std_msgs/msg/Float64 "{data: 0.08}"
```

Close the gripper:

```bash
ros2 topic pub --once /myarm/gripper/command std_msgs/msg/Float64 "{data: 0.0}"
```

For a physical robot, the command is accepted only after enabling both
`transport.allow_physical_motion: true` and
`gripper.allow_physical_actuation: true`, then powering on and calling
`/myarm/robot/rearm`. The checked-in fake profile accepts the command directly.

The node does not plan trajectories, queue public goals or own action
preemption. It polls `read_feedback()` and forwards at most the newest private
setpoint through `send_joint_setpoint()`. This keeps exactly one serial owner
for both `FakeRobotArm` and `MyArmM750RobotArm`.
