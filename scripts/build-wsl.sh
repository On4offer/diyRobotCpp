#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cmake -S "$project_root" -B "$project_root/build-wsl" \
  -DCMAKE_BUILD_TYPE=Release -DDIYROBOT_BUILD_QT=OFF -DDIYROBOT_BUILD_OPENCV=OFF \
  -DDIYROBOT_WARNINGS_AS_ERRORS=ON
cmake --build "$project_root/build-wsl" --parallel
ctest --test-dir "$project_root/build-wsl" --output-on-failure
