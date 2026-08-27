---
name: diyrobot-cpp-roadmap
description: Implement, refactor, review, or validate diyRobotCpp and similar C++17/ROS 2 SO-ARM101 robotics code against the D1-D8 learning roadmap. Use for production structure, readable C++, safety boundaries, offline tests, CMake/CI, ROS 2 integration, or roadmap completion claims; do not use for unrelated C++ exercises or pure conceptual Q&A.
---

# diyRobot C++ Roadmap Engineering

Use the roadmap as an acceptance contract, not merely a file checklist.

1. Read `docs/ROADMAP_STATUS.md` and the relevant source before changing a stage.
2. Read [references/quality-gates.md](references/quality-gates.md) for any code, CMake, test, or CI change.
3. Read [references/roadmap-acceptance.md](references/roadmap-acceptance.md) before updating roadmap status or claiming completion.
4. Keep hardware access dry-run by default. Require an explicit run flag and any stage-specific physical confirmation.
5. Keep platform details behind interfaces; inject serial, servo, camera, and grasp hardware so offline tests do not require devices.
6. Prefer small named functions and typed configuration over compressed statements, duplicated literals, hidden global state, or comments that restate syntax.
7. Add or update tests for success, boundary, malformed input, timeout/failure, and recovery paths touched by the change.
8. Run the repository verification entry point before reporting completion:
   - Windows: `pwsh -File .agents/skills/diyrobot-cpp-roadmap/scripts/verify.ps1`
   - WSL/Linux: `bash .agents/skills/diyrobot-cpp-roadmap/scripts/verify.sh`
9. Report software evidence separately from camera, servo, training, and physical-world acceptance.

Use MoveIt 2, ros2_control, and Navigation2 as structural references, not as code to copy. Preserve this project's beginner-readable scope and avoid introducing a framework unless it removes real duplication or enforces a needed boundary.
