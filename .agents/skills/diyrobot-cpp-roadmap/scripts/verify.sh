#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
cd "$project_root"

bash scripts/build-wsl.sh

if find include src apps tests ros2_ws/src -type f \
  \( -name '*.cpp' -o -name '*.hpp' -o -name '*.h' \) \
  -print0 | xargs -0 awk 'length($0) > 100 { print FILENAME ":" FNR ":" length($0); failed=1 } END { exit failed }'; then
  :
else
  echo 'C++ lines longer than 100 columns were found.' >&2
  exit 1
fi

if command -v clang-format >/dev/null 2>&1; then
  find include src apps tests ros2_ws/src -type f \( -name '*.cpp' -o -name '*.hpp' -o -name '*.h' \) \
    -print0 | xargs -0 clang-format --dry-run --Werror
else
  echo 'NOTE: clang-format is unavailable; CI performs the formatting check.'
fi

echo 'VERIFY_WSL=PASS'
