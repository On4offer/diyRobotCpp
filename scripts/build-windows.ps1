$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
cmake -S $root -B (Join-Path $root 'build') -G 'MinGW Makefiles' -DCMAKE_BUILD_TYPE=Release `
    -DDIYROBOT_WARNINGS_AS_ERRORS=ON
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
cmake --build (Join-Path $root 'build') --parallel
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
ctest --test-dir (Join-Path $root 'build') --output-on-failure
exit $LASTEXITCODE
