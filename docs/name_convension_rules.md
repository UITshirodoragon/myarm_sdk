# MyArm M750 Python Coding Rules

## Naming

- Use `snake_case` for modules, functions, methods, variables, parameters,
  ROS packages, YAML keys and ROS runtime names.
- Use `PascalCase` for classes, dataclasses, protocols, enums and exceptions.
- Use `UPPER_SNAKE_CASE` for constants.
- Exception names must end with `Error`.
- Boolean names must start with `is_`, `has_`, `can_`, `should_` or `enable_`.
- Collection names must be plural.
- Physical quantities must include their units in the name:
  `_rad`, `_rad_s`, `_m`, `_s`, `_hz`, `_nm`, `_bytes`.
- Avoid generic names such as `Manager`, `Helper`, `Utils`, `Common`,
  `Data`, `Info`, `Object`, `Value` and `Thing` when a precise domain name exists.

## Architecture

- `core` must not import ROS 2, pymycobot, serial, Pinocchio, MuJoCo or
  another external framework.
- `port_interface` defines ROS-independent contracts.
- `plugin_adapter` implements a contract and may depend on an external library.
- `service` owns one capability-oriented workflow and is the interface used by
  ROS 2 nodes in this project stage.
- ROS 2 nodes only map ROS messages, timers and publishers to services; they
  must not call a plugin adapter directly.
- Hardware mapping and unit conversion belong in the corresponding plugin adapter.
- Each plugin adapter keeps its own `config/` directory. `service/config/services.yaml`
  is the single service manifest that enables services and instances.

## Method naming

- `read_*`: hardware, sensor or stream input.
- `write_*`: direct device or transport output.
- `publish_*`: ROS/pub-sub output.
- `load_*`: persistent files, YAML, URDF or calibration.
- `compute_*`: deterministic calculation.
- `estimate_*`: estimated result.
- `solve_*`: optimization or inverse problem.
- `validate_*`: validate input and raise on failure.
- `ensure_*`: guarantee a condition, possibly by performing an action.
- `is_*`, `has_*`, `can_*`: boolean result.
- `execute()`: application use cases and commands.
- `run()`: loops, applications and benchmark runners.
- `handle_*`: requests, events and errors.
- `on_*`: callbacks triggered by incoming events.

## Units and frames

- Core uses SI units.
- Joint angles use radians.
- Distances use meters.
- Time uses seconds.
- Angular velocity uses radians per second.
- Torque uses newton-meters.
- Frame variables end with `_frame_id`.
- Pose and transform names must identify source and target frames.
- URDF is the source of truth for kinematic joint order, axes, hard limits and
  fixed TCP transforms. Adapter configuration may validate those facts but must
  not silently reverse a joint axis or duplicate a conflicting TCP transform.
- Kinematics uses normalized `xyzw` quaternions and SE(3) residuals; do not
  introduce Euler-angle interpolation into the FK/IK path.

## Data models

- Use dataclasses for structured data.
- Prefer `frozen=True, slots=True` for immutable value objects.
- Use nouns for model names.
- Use commands for state-changing application inputs.
- Do not use ROS messages as domain or application models.
- Do not return SDK sentinel values such as `-1`; translate them into exceptions.

## Tests

- Tests must not require real hardware unless marked as integration tests.
- Provide a fake implementation for each critical hardware port.
- Test names must describe behavior and condition.
