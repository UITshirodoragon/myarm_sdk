# neugrasp_bringup

NeuGrasp composition for MyArm M750. The canonical plain
`myarm_m750_poe_v3_2.urdf` remains unchanged for pycore and Pinocchio. This
package uses a separate Xacro profile only to append fixed wrist-camera frames.

The runtime is deliberately phase-oriented:

```text
scene_debug  -> inspect workspace and generated camera targets
fake_scan    -> execute targets one-at-a-time against FakeRobotArm
replay       -> inspect completed PLY/JSON artifacts without model or camera
```

Physical NeuGrasp motion is not enabled by this package.

## Frame ownership

```text
robot_state_publisher (NeuGrasp Xacro)
  base_link -> arm -> flange_link -> tool0
                              └-> gripper_base_link
                                   -> wrist_camera_mount_link
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

`neugrasp_fake_wrist_camera_calibration.yaml` is a named simulation transform,
not a real camera estimate.

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

## Fake sequential scan

The fake launch starts one RSP, FakeRobotArm, the normal motion executor and
its fake-only `FollowCartesianTrajectory` action. The scan node stays idle
until a `ScanWorkspace` action goal arrives; it never moves on launch.

```bash
ros2 launch neugrasp_bringup neugrasp_fake_scan.launch.py
```

First inspect all four targets without motion:

```bash
ros2 action send_goal --feedback /neugrasp/scan_workspace \
  myarm_interfaces/action/ScanWorkspace \
  "{profile_id: paper_phi180, execute_motion: false, capture_enabled: false, settle_time_s: -1.0}"
```

Then execute the same sequence against the fake adapter:

```bash
ros2 action send_goal --feedback /neugrasp/scan_workspace \
  myarm_interfaces/action/ScanWorkspace \
  "{profile_id: paper_phi180, execute_motion: true, capture_enabled: false, settle_time_s: -1.0}"
```

For each view the coordinator computes:

```text
workspace camera target
  -> base camera target
  -> base tool0 target using TF(tool0, wrist_camera_optical_frame)
  -> /myarm/follow_cartesian_trajectory
  -> settle -> measured base->camera snapshot
```

The coordinator never publishes a joint target/setpoint, opens no serial
device, and never calls legacy `robot.py`. Cancellation cancels the active
child action and requests the executor's normal cancel service.

`capture_enabled` currently records a phase boundary/feedback only; no camera
is faked. This permits TF, trajectory and replay validation before live sensor
integration.

`paper_phi180` is a deterministic visualization/benchmark profile, not
hardware-certified pose data. Add a calibrated `fixed_poses` profile only
after reachability and camera/workspace validation. Do not import legacy
`planned_scan_trajectory.json` as a live profile: its azimuth and
`T_eef_camera` convention differ.

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

## Replay a completed run

Replay starts no driver, camera or NeuGrasp inference model. It keeps only the
robot-state publisher by default so recorded `base_link` data has frame
context. It deliberately does **not** start the current deployment's workspace
scene: its calibration may differ from the recorded run. It republishes old
products at a low rate:

```bash
ros2 launch neugrasp_bringup neugrasp_replay.launch.py \
  run_dir:=/home/ktmt-agx-xv/Data/khoanhd/Octo_Lab/NeuGrasp_real_runtime_v0_1_3/NeuGrasp/real_runtime/runs/neugrasp_real_20260626_105154
```

Sources, when present:

```text
visualizations/tsdf_near_surface_base.ply
inference/candidates.json
inference/selected_grasp.json
visualizations/grasp_candidates_wireframes_base.ply  (fallback)
```

The legacy TSDF PLY is already in `base_link`; replay publishes it to
`/neugrasp/tsdf_cloud` without transforming it again. Candidate JSON becomes
gripper wireframe markers in the semantic grasp frame. The raw legacy PLY is
published separately as `/neugrasp/legacy_grasp_wireframes`: it encodes an
older TCP visualization and must never be mixed into the grasp-frame topic.
For a calibrated overlay, explicitly opt in only with a `scene_config` known
to match that exact run:

```bash
ros2 launch neugrasp_bringup neugrasp_replay.launch.py \
  run_dir:=/path/to/run enable_scene_frames:=true scene_config:=/path/to/matching_scene.yaml
```

The replay node rejects arbitrary `base_frame` relabeling unless
`allow_legacy_frame_relabel:=true` is explicitly supplied; legacy coordinates
are immutable `base_link` coordinates.

## RViz, messages and QoS

`myarm_rviz2/config/neugrasp.rviz` subscribes to:

```text
/neugrasp/workspace_marker
/neugrasp/scan_views
/neugrasp/scan_view_markers
/neugrasp/planned_camera_poses
/neugrasp/measured_camera_poses
/neugrasp/tsdf_cloud
/neugrasp/quality_cloud
/neugrasp/grasp_candidates
/neugrasp/selected_grasp
/neugrasp/selected_grasp_marker
/neugrasp/legacy_grasp_wireframes
```

The workflow repeats small, sequential phases. The larger replay cloud uses
volatile/best-effort delivery and is republished at a low rate; it has no
high-rate DDS reliability contract. Small scan/marker snapshots use retained
reliable delivery so a late RViz can see their current state, and the
workspace marker is also refreshed at low rate. Do not continuously send raw
`.npy`, `.ply` or tensors over DDS.

## Safety boundary

`neugrasp_fake_scan.launch.py` explicitly asserts `fake_robot_arm`; it is the
only execution launch supplied. Before any physical profile is considered,
collision checking, tracking-error fault behavior, certified calibration and
measured reachability of every scan view must be completed.
