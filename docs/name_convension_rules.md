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

- Domain code must not import ROS 2, pymycobot, serial, Pinocchio,
  MuJoCo or other frameworks.
- Application code depends only on domain models and ports.
- Adapters implement ports and may depend on external libraries.
- ROS 2 nodes only map ROS interfaces to application/core interfaces.
- Hardware mapping and unit conversion belong in the hardware adapter.
- Concrete implementations are selected only in the composition root.

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