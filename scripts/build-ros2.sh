#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source /opt/ros/jazzy/setup.bash
set -u
cd "$project_root/ros2_ws"
if ! command -v colcon >/dev/null 2>&1; then
  echo "colcon not found; using the ament/CMake single-package fallback." >&2
  echo "For the standard multi-package workflow: sudo apt install python3-colcon-common-extensions" >&2
  cmake -S src/diyrobot_ros -B build-direct -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$project_root/ros2_ws/install-direct" \
    -DDIYROBOT_WARNINGS_AS_ERRORS=ON
  cmake --build build-direct --parallel
  cmake --install build-direct
  set +u
  source install-direct/share/diyrobot_ros/local_setup.bash
  set -u
  ros2 pkg prefix diyrobot_ros >/dev/null
  exit 0
fi
colcon build --base-paths src --symlink-install --event-handlers console_direct+ \
  --cmake-args -DDIYROBOT_WARNINGS_AS_ERRORS=ON
set +u
source install/setup.bash
set -u
ros2 pkg prefix diyrobot_ros >/dev/null
