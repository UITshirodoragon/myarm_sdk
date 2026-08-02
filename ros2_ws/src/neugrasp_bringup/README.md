# neugrasp_bringup

NeuGrasp composition for MyArm M750. The canonical plain
`myarm_m750_poe_v3_2.urdf` remains unchanged for pycore and Pinocchio. This
package uses a separate Xacro profile only to append fixed wrist-camera frames.

The runtime is deliberately phase-oriented:

```text
scene_debug  -> inspect workspace and generated camera targets
fake_scan    -> execute targets one-at-a-time against FakeRobotArm
replay       -> render saved tensors in the current scene/robot frame config
fake_trial   -> one automatic tensor-only scan/pick sequence against FakeRobotArm
```

Physical NeuGrasp motion is not enabled by this package.

## Frame ownership

```text
robot_state_publisher (NeuGrasp Xacro)
  base_link -> arm -> flange_link -> tool0
                              └-> gripper_base_link
                                   -> logitech_c925_wrist_mount_link  (C925 profile)
                                   -> wrist_camera_link
                                   -> wrist_camera_optical_frame

neugrasp_static_scene_frames
  world -> base_link                 optional, one owner only
  base_link -> neugrasp_workspace    cell calibration
  workspace -> neugrasp_volume       derived from bbox.min

myarm_neugrasp scan node
  PoseArray + MarkerArray for runtime scan targets
```

Scan targets, selected grasps, point clouds and candidate grasps are not URDF
or static-scene TF. The selected profile produces them at runtime, so changing
trajectory parameters/calibration does not leave stale TF in the graph.

## Build

From `myarm_sdk/ros2_ws`:

```bash
colcon build --packages-up-to myarm_interfaces myarm_neugrasp neugrasp_bringup myarm_rviz2
source install/setup.bash
```

## Wrist-camera calibration

The robot-description profile is:

```text
myarm_description/urdf/myarm_m750_neugrasp.urdf.xacro
```

It only adds fixed links below `gripper_base_link`; it never changes the arm,
`tool0`, limits or baseline dynamic model.

## Camera model profiles and fixed frames

The camera subtree is composed as **robot + named mount + camera model**. The
current measured profile is `logitech_c925_wrist_v1`:

```text
gripper_base_link
  -> logitech_c925_wrist_mount_link
  -> wrist_camera_link
  -> wrist_camera_optical_frame
```

Its runtime transforms are fixed in Xacro, not published by a second static
TF node:

```text
T_gripper_base_mount   = xyz[-0.0215, 0, -0.0214], rpy[0, -pi/2, 0]
T_mount_camera_body    = xyz[0.00209, 0, 0.025], rpy[0, 0, 0]
T_camera_body_optical  = xyz[0.04, 0, 0.03], rpy[-pi/2, 0, -pi/2]
```

`wrist_camera_link -> wrist_camera_optical_frame` is therefore an explicit
measured fixed joint. Future camera models add their own named mount and
profile under `myarm_description/urdf/xacro/sensors/{mounts,profiles}/`, while
keeping the semantic body/optical frame names when used by NeuGrasp.

When `use_wrist_camera:=true`, `neugrasp_system.launch.py` requires one
calibration YAML. Start from:

```text
myarm_description/config/neugrasp_wrist_camera_calibration.template.yaml
```

The launch validates schema, frame names, numeric transforms, ID/hash and
status. For `status: CALIBRATED`, `calibration_sha256` must equal the SHA-256
of canonical JSON for the complete YAML mapping except that field itself,
prefixed by `sha256:`. `status: FAKE` is accepted only with
`required_robot_arm_plugin_adapter:=fake_robot_arm`. A real record must be
`CALIBRATED`. Do not reuse legacy `T_eef_camera` until its parent frame is
verified and converted to the explicit `gripper_base_link -> camera optical`
contract.

Regenerate the digest after every real-calibration edit:

```bash
python3 - /absolute/path/to/calibration.yaml <<'PY'
import hashlib, json, sys, yaml
path = sys.argv[1]
with open(path, encoding="utf-8") as stream:
    record = yaml.safe_load(stream)
record.pop("calibration_sha256", None)
payload = json.dumps(record, ensure_ascii=False, sort_keys=True,
                     separators=(",", ":"), allow_nan=False)
print("sha256:" + hashlib.sha256(payload.encode("utf-8")).hexdigest())
PY
```

Every calibration record names `camera_profile`. `generic` consumes `mount`,
`camera_body` and `camera_optical` from YAML. `logitech_c925_wrist_v1` selects
the fixed model Xacro; the same values remain in YAML for provenance and must
stay synchronized. The checked-in C925 measurement record is deliberately
`UNVERIFIED`; it must not be marked `CALIBRATED` until physical TF validation
has passed.

`neugrasp_fake_wrist_camera_calibration.yaml` uses C925 geometry only for
fake-arm visualization. Its `FAKE` status makes it unusable with a physical
adapter.

### Change camera geometry safely

Treat mount, camera body and optical-frame values as one calibration change.
Never add a second `static_transform_publisher` for these frames.

1. For C925, edit the named mount/profile Xacros and mirror the measurement in
   `config/camera_profiles/logitech_c925_wrist_v1.measurement.yaml`. For a new
   model, add a new mount Xacro and camera-profile Xacro instead of changing
   the C925 profile.
2. Update the calibration record. A physical record requires
   `status: CALIBRATED` and a regenerated `calibration_sha256`.
3. Reinstall packages, generate the URDF explicitly, then validate it:

   ```bash
   cd /home/ktmt-agx-xv/Data/khoanhd/MyArmM750_Controller_Lab/myarm_sdk/ros2_ws
   source /opt/ros/foxy/setup.bash
   colcon build --packages-select myarm_description neugrasp_bringup myarm_neugrasp myarm_rviz2
   source install/setup.bash

   xacro "$(ros2 pkg prefix myarm_description)/share/myarm_description/urdf/myarm_m750_neugrasp.urdf.xacro" \
     use_wrist_camera:=true wrist_camera_profile:=logitech_c925_wrist_v1 \
     > /tmp/myarm_m750_neugrasp_c925.urdf
   check_urdf /tmp/myarm_m750_neugrasp_c925.urdf
   ```

4. Start scene debug and confirm every camera frame has one parent before the
   scan action:

   ```bash
   ros2 launch neugrasp_bringup neugrasp_scene_debug.launch.py start_rviz:=true
   ```

`colcon build` reinstalls Xacro/YAML; it does not change canonical v3.2
kinematics. The `xacro ... > /tmp/*.urdf` command is the explicit URDF
generation step and `check_urdf` validates the result.

## Scene debug: no motion

This starts fake feedback, application Xacro, static workspace/volume TF and
the scan-profile visualizer. It does not expose a motion executor:

```bash
ros2 launch neugrasp_bringup neugrasp_scene_debug.launch.py
```

Use `start_rviz:=true` for a local viewer, or start the passive remote viewer
separately on the host PC:

```bash
ros2 launch myarm_rviz2 neugrasp_rviz_remote.launch.py
```

The viewer never starts RSP, TF, driver, planner or replay nodes.

## Fake sequential scan: one-shot IK + joint trajectory

The default fake launch starts one RSP, FakeRobotArm and the normal motion
executor. The default scan path is `one_shot_ik_joint_trajectory`; it does not
use `FollowCartesianTrajectory`. The scan node stays idle until a
`ScanWorkspace` action goal arrives; it never moves on launch.

```bash
ros2 launch neugrasp_bringup neugrasp_fake_scan.launch.py
```

First inspect all four targets without motion:

```bash
ros2 action send_goal --feedback /neugrasp/scan_workspace \
  myarm_interfaces/action/ScanWorkspace \
  "{profile_id: neugrasp_simulation_views_16_19, execute_motion: false, capture_enabled: false, settle_time_s: -1.0}"
```

Then execute the same sequence against the fake adapter:

```bash
ros2 action send_goal --feedback /neugrasp/scan_workspace \
  myarm_interfaces/action/ScanWorkspace \
  "{profile_id: neugrasp_simulation_views_16_19, execute_motion: true, capture_enabled: false, settle_time_s: -1.0}"
```

For each view the coordinator computes:

```text
workspace camera target
  -> base camera target
  -> base tool0 target using TF(tool0, wrist_camera_optical_frame)
  -> one-shot IK from fresh /myarm/state/joint_state
  -> validated minimum-jerk joint trajectory
  -> /myarm/follow_joint_trajectory
  -> settle -> measured base->camera snapshot
```

This eliminates the intermediate Cartesian waypoints that were producing
`branch_discontinuity` or `joint_limit_blocked` in sequential CLIK. It cannot
make an unreachable endpoint valid: endpoint IK still respects URDF hard
limits and the configured safety margin. The coordinator never publishes a
driver setpoint, opens no serial device, or calls legacy `robot.py`.
Cancellation cancels the active child action and requests the executor's normal
cancel service.

`execution.motion_planner` in `neugrasp_scan_profiles.yaml` selects the path:

```yaml
motion_planner: one_shot_ik_joint_trajectory  # default
# motion_planner: cartesian_trajectory         # fake-only CLIK diagnosis
```

For the Cartesian diagnostic path, change this field, rebuild/source the
workspace, then relaunch fake scan. That path requires the fake-only
`FollowCartesianTrajectory` server already exposed by the fake launch. The
joint path uses the same canonical feedback, URDF limits, minimum-jerk planner
and motion-execution safety gates as ordinary joint motion.

When a joint scan fails, its `ScanWorkspace` action detail reports the endpoint
IK failure, active joint limits, minimum margin and residual. Do not reduce the
URDF limits or safety margin merely to make a scan pass; instead adjust the
scan radius/alignment/workspace calibration or use a validated fixed
camera-pose profile.

`capture_enabled` currently records a phase boundary/feedback only; no camera
is faked. This permits TF, trajectory and replay validation before live sensor
integration.

`neugrasp_simulation_views_16_19` is the default deterministic scan profile. It preserves
simulation-evaluation identities `view_16` through `view_19` from NeuGrasp
`render_utils.py`, then rotates the complete source arc by `-90°` around +Z so
its center is at robot-facing azimuth `180°`. Azimuth is ROS-standard
counter-clockwise around +Z (`+X -> +Y`); its values are `[150, 170, 190, 210]`
degrees, beta is `[45, 35, 25, 15]` degrees, and all live trial views use
`r=0.40 m`. The source simulation evaluation uses `r=0.50 m`, so this profile
must not be called an exact simulation reproduction. Its query view is
`view_17`, the second view in the four-view set. Add a calibrated `fixed_poses`
profile only after reachability and camera/workspace validation. Do not import legacy
`planned_scan_trajectory.json` as a live profile: its azimuth and
`T_eef_camera` convention differ.

`neugrasp_tranning_views_16_19` is the separate nominal packed-training
profile. It preserves packed-bank IDs `16` through `19`, uses `r=0.45 m`, beta
`[28.5, 14, 43, 28.5]` degrees and ROS-CCW azimuth
`[157.5, 180, 180, 202.5]` degrees after its common `-112.5°` alignment. The
packed generator randomizes radius, beta and pose noise per scene; use that
scene's recorded `camera_pose.npy` whenever exact training-scene reproduction
is needed.

## Add a calibrated fixed profile

Keep scan targets in the profile YAML, never as URDF joints or static TF. A
`fixed_poses` target is a **camera optical** pose expressed in
`neugrasp_workspace`; the coordinator derives the matching `tool0` target
from TF at runtime. Add four validated entries in the same `profiles` mapping:

```yaml
custom_fixed:
  type: fixed_poses
  profile_version: 1
  views:
    - view_key: view_00
      source_view_id: "0"
      position_m: [measured_x, measured_y, measured_z]
      orientation_xyzw: [measured_qx, measured_qy, measured_qz, measured_qw]
    # Add view_01 .. view_03 after their optical poses are measured.
  capture_order: [view_00, view_01, view_02, view_03]
  model_input_order: [view_00, view_01, view_02, view_03]
  query_view_key: view_01
```

`capture_order` and `model_input_order` are independent but must each cover
every view exactly once. `query_view_key` is validated as a stable view key
(or source view ID), not a hard-coded model-batch index.

For a `paper_spiral` profile, optional `view_keys` preserves external
simulator identifiers. Both `neugrasp_simulation_views_16_19` and
`neugrasp_tranning_views_16_19` use this to publish `view_16` rather
than renumbering that source view to `view_00` in feedback and metadata.

## Replay a completed run

Replay starts no driver, camera or NeuGrasp inference model. It starts the
current robot description and, by default, the current `scene_config`; these
are the only owners of robot/camera/workspace/volume frame relationships.

Replay is a tensor visualization phase. It reads exactly the four raw model
outputs below:

```text
inference/tsdf_vol.npy
inference/qual_vol_raw.npy
inference/rot_vol_raw.npy
inference/width_vol_raw.npy
```

It does not read either legacy `.ply`, a JSON candidate, calibration, TF
snapshot, scan metadata, image, or inference manifest from the run.

```bash
ros2 launch neugrasp_bringup neugrasp_replay.launch.py \
  run_dir:=/home/ktmt-agx-xv/Data/khoanhd/Octo_Lab/NeuGrasp_real_runtime_v0_1_3/NeuGrasp/real_runtime/runs/neugrasp_real_20260626_105154
```

Append `start_rviz:=true` to use the supplied NeuGrasp RViz profile.

The tensors are an unframed lattice. The replay node maps their indices
directly into the current local volume frame; it performs no TF lookup and
does not consume a transform or calibration from the run. With the standard
NeuGrasp volume (`0.30 m`, resolution `40`), the voxel size is `0.0075 m`.
The default `voxel_reference:=center` publishes voxel centres
`(index + 0.5) * voxel_size`, so RViz boxes of size `0.0075 m` fill exactly
the configured `neugrasp_volume` bounds.

Both ROS headers are therefore `neugrasp_volume` by default:

```text
/neugrasp/tsdf_cloud          sensor_msgs/PointCloud2, generated packed RGB
/neugrasp/grasp_wireframes    visualization_msgs/Marker, LINE_LIST
```

`/neugrasp/tsdf_cloud` selects `-0.85 < TSDF < 0.0`; its `RGB8` colour is a
generated near-surface colour map, not a model RGB output. The grasp marker
reproduces the legacy NeuGrasp postprocess: Gaussian quality smoothing,
TSDF/width validity mask, score threshold, and local-maximum suppression. It
then uses the corresponding quaternion and width tensor to draw the standard
four-edge gripper wireframe. Widths above the actual MyArm opening `0.08 m`
are rejected. The default RViz profile contains no image, manifest, candidate
JSON or historical run-frame display; scan route/pose markers are published
only while a `ScanWorkspace` action is active.

If your current scene uses a different volume frame, override the node
contract. `source_frame` remains accepted only for backwards-compatible launch
commands and is ignored by tensor replay:

```bash
ros2 launch neugrasp_bringup neugrasp_replay.launch.py \
  run_dir:=/path/to/run target_frame:=neugrasp_volume
```

To include the current wrist-camera Xacro profile in that TF tree, opt in with
the current named calibration (never a YAML copied from `run_dir`):

```bash
ros2 launch neugrasp_bringup neugrasp_replay.launch.py \
  run_dir:=/path/to/run use_wrist_camera:=true \
  camera_calibration:=/path/to/current_camera_calibration.yaml
```

Replay is visualization-only: neither topic is a motion command.

## Fake autonomous one-trial path

`neugrasp_fake_trial.launch.py` is a separate **FakeRobotArm-only** path. It
does not include either `neugrasp_replay.launch.py` or
`neugrasp_fake_scan.launch.py`, so there is exactly one robot-state publisher,
 static scene-frame node, fake driver, motion executor, scan node and trial
 coordinator in the graph. The trial coordinator itself creates the retained
 artifact snapshot only in its Predict phase. It must never be used with a physical
adapter.

The required `run_dir` supplies only the four raw tensors below. No camera
node, images, old run TF/calibration, JSON/PLY artifact, inference manifest or
NeuGrasp inference model is started or consumed.

```text
inference/tsdf_vol.npy
inference/qual_vol_raw.npy
inference/rot_vol_raw.npy
inference/width_vol_raw.npy
```

Start one automatic fake trial with a completed run:

```bash
ros2 launch neugrasp_bringup neugrasp_fake_trial.launch.py \
  run_dir:=/home/ktmt-agx-xv/Data/khoanhd/Octo_Lab/NeuGrasp_real_runtime_v0_1_3/NeuGrasp/real_runtime/runs/neugrasp_real_20260603_194238 \
  start_rviz:=true
```

The launch passes `neugrasp_simulation_views_16_19` directly to the
`ScanWorkspace` action; it does not depend on `trajectory.active_profile`.
Override it only with another fake-validated profile:

```bash
ros2 launch neugrasp_bringup neugrasp_fake_trial.launch.py \
  run_dir:=/path/to/completed/run \
  scan_profile_id:=neugrasp_simulation_views_16_19
```

The coordinator runs exactly once and publishes its current phase on
`/neugrasp/trial/phase`:

```text
READY -> INIT/HOME -> SCAN -> PREDICT artifact -> SELECT/PREFLIGHT
      -> PREGRASP -> GRASP -> CLOSE -> LIFT -> COMPLETE
```

It first waits for fresh feedback, current `base_link <- neugrasp_volume` TF,
the scan/executor actions and gripper feedback. It opens the fake gripper to
the actual MyArm maximum opening of `0.08 m`, moves home with a validated
minimum-jerk joint trajectory, and calls the scan action with
`execute_motion=true`, `capture_enabled=false` and `settle_time_s=2.0` for each
scan view. Predict reconstructs the
standard Gaussian/TSDF/width/local-maxima NeuGrasp candidates from tensors at
voxel centres `(index + 0.5) * voxel_size`.

Timing is explicit in `config/neugrasp_fake_trial.yaml`: every completed
non-terminal phase (`INIT_HOME`, `SCAN`, `PREDICT_ARTIFACT`,
`SELECT_PREFLIGHT`, `PREGRASP`, `GRASP`, `CLOSE`, `LIFT`) has a default
three-second settle before the next phase. The coordinator remains responsive:
cancel/failure stops immediately and never waits out a settle. The scan action
keeps its distinct two-second settle at each view.

No artifact is published during READY, INIT or SCAN. Entering
`PREDICT_ARTIFACT` publishes one retained snapshot in `neugrasp_volume`.
For WLAN/RViz it renders the configured inclusive Z-index range `[5, 30]`
from the NeuGrasp volume (the volume floor is `0.0503 m` below workspace);
tensor postprocess, candidate selection and motion still use the full 40-cubed
volumes. Standalone replay uses the same Z-index-range, one-shot retained snapshot
policy; it does not resend the TSDF at 1 Hz. Set
`republish_period_s` only for a legacy volatile subscriber.

Candidates are tried in descending quality only in a scratch IK preflight:
`pregrasp -> grasp -> lift`. The first candidate that makes all three states
feasible is selected. Each real motion is re-solved from fresh feedback; a
missing tensor/TF, scan failure, cancellation, infeasible candidate or motion
failure cancels the active motion and terminally stops the trial. There are no
retries and it never sends a later grasp command after a failure.

The fake profile explicitly configures `T_grasp_tool0`: translation
`[0, 0, +0.05] m`, then `Rx(pi)` (XYZW `[-1, 0, 0, 0]`). The coordinator
composes this direct grasp-to-tool0 relation when forming its tool target; it
is a synthetic mapping preflighted only for the current FakeRobotArm scene,
not a TCP or camera calibration. Pregrasp retreats `0.05 m` along predicted grasp
`-Z`; top-down lift retreats `0.10 m` along that same local axis, while a side
grasp lifts `+Z_base` by `0.10 m` (top-down threshold: `60 deg`). FakeRobotArm
has no collision, contact or object attachment model, so `COMPLETE` means the
command sequence finished, not that an object was physically grasped.

After a terminal phase the launch deliberately stays alive: RViz can keep the
TSDF, quality-coloured candidates, selected green tool target, cyan pregrasp,
purple lift and final fake robot pose visible. It does not create a second
trial. RViz Foxy has no built-in `std_msgs/String` display, so inspect phase
text from a terminal when needed:

```bash
ros2 topic echo /neugrasp/trial/phase
```

## RViz, messages and QoS

`myarm_rviz2/config/neugrasp.rviz` subscribes to:

```text
/neugrasp/workspace_marker
/neugrasp/tsdf_cloud
/neugrasp/grasp_wireframes
/neugrasp/selected_grasp              # PoseStamped, green tool0 target
/neugrasp/selected_grasp_marker       # Marker, green NeuGrasp grasp-frame wireframe
/neugrasp/pregrasp_marker             # Marker, cyan NeuGrasp pregrasp-frame wireframe
/neugrasp/lift_marker                 # Marker, purple NeuGrasp lift-frame wireframe
/neugrasp/scan_view_markers           # MarkerArray, scan phase only
```

Replay cloud and marker snapshots use reliable, transient-local one-shot
delivery; the Grasp Tensor Wireframes RViz display uses the matching QoS so a
remote RViz opened after Predict receives them. Trial target markers use the
same retained reliable policy. During `SCAN`, the scan MarkerArray displays
the capture route, camera frusta, labels and pose axes (X red, Y green, Z
blue); the scan node publishes a retained delete snapshot on completion,
cancel or abort. The phase topic is `std_msgs/String`, intended for terminal
or monitoring tools rather than the stock Foxy RViz display. The four raw
tensors are never streamed over DDS.

## Safety boundary

`neugrasp_fake_scan.launch.py` and `neugrasp_fake_trial.launch.py` explicitly
assert `fake_robot_arm`; no physical execution launch is supplied. Before any
physical profile is considered, collision checking, tracking-error fault
behavior, certified calibration and measured reachability of every scan view
must be completed.
