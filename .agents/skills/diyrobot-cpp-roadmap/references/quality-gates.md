# C++ and ROS 2 quality gates

## Reference baseline

- MoveIt 2: formatting checks, multi-distribution CI, tests, and coverage reporting.
- ros2_control: public API separation, plugin/backend boundaries, formatting, tests, and multi-distribution CI.
- Navigation2: package-level ownership, documented configuration, plugins, migration notes, CI, coverage, and container workflows.

Apply the principles proportionally; this learning project does not need their full framework complexity.

## Design

- Keep public headers in `include/diyrobot`, implementations in `src`, entry points in `apps`, tests in `tests`, and ROS-specific integration inside `ros2_ws/src`.
- Give each module one responsibility. Keep OS, serial, camera, ROS, and GUI dependencies out of the domain model.
- Own resources with RAII. Prefer values, references, and `std::unique_ptr`; document non-owning lifetimes.
- Pass dependencies through constructors or function parameters. Tests must be able to substitute hardware.
- Store user-tunable values in configuration or named typed defaults. Do not duplicate ports, servo IDs, safety limits, camera IDs, or calibration matrices across entry points.
- Validate at system boundaries and return errors with context. Never silently continue after protocol, calibration, motion, or recovery failures.

## Readability

- Format C++ with the repository `.clang-format`; target 100 columns.
- Do not place control flow, multiple statements, functions, or class members on one line.
- Prefer early returns, descriptive names, `const`, scoped enums, and explicit units such as `_ms`, `_degrees`, or `_meters`.
- Keep functions focused. Extract argument parsing, formatting, hardware setup, and recovery from `main` and ROS callbacks.
- Comments explain intent, invariants, units, protocol quirks, and safety rationale; code explains mechanics.

## Safety and failure behavior

- Hardware commands are dry-run by default and require explicit opt-in.
- Validate device identity, limits, calibration, reach, and configuration before enabling motion.
- Every motion workflow defines its failure policy: hold position, return home, disable torque, or emergency stop. Recovery failure must be observable.
- Bound retries, durations, speeds, accelerations, and rollout time.
- Mocks and simulations prove software behavior only; they cannot validate payload, collision, electrical, thermal, latency, or emergency-stop behavior.

## Build and test

- Compile every owned target with strict warnings; CI treats warnings as errors.
- Build on Windows and Ubuntu/WSL with C++17 and no compiler extensions.
- Test normal paths, invalid frames/input, numerical boundaries, unreachable geometry, timeout/failure, and recovery.
- Run sanitizer tests on Linux for core code. Run ROS package build/tests separately from the standalone CMake build.
- Keep generated output, datasets, credentials, models, and build trees out of Git.
- Before completion, run tests, UTF-8/no-BOM checks, line-ending checks, secret/artifact scans, and `git diff --check`.
