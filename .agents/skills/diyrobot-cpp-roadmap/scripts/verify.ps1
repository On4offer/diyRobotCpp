[CmdletBinding()]
param(
    [switch]$SkipBuild
)

$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..\..\..')).Path
Push-Location $projectRoot
try {
    if (-not $SkipBuild) {
        & pwsh -NoProfile -File scripts/build-windows.ps1
        if ($LASTEXITCODE -ne 0) { throw 'Windows build or tests failed.' }
    }

    $sourceFiles = Get-ChildItem include,src,apps,tests,ros2_ws/src -Recurse -File |
        Where-Object { $_.Extension -in '.cpp', '.hpp', '.h' }
    $tooLong = foreach ($file in $sourceFiles) {
        $lineNumber = 0
        foreach ($line in Get-Content -LiteralPath $file.FullName) {
            ++$lineNumber
            if ($line.Length -gt 100) {
                '{0}:{1}:{2}' -f $file.FullName, $lineNumber, $line.Length
            }
        }
    }
    if ($tooLong) {
        $tooLong | Write-Error
        throw 'C++ lines longer than 100 columns were found.'
    }

    $textFiles = Get-ChildItem . -Recurse -File |
        Where-Object {
            $_.FullName -notmatch '[\\/](build|build-wsl|install|log|\.git)[\\/]' -and
            $_.Extension -in '.cpp', '.hpp', '.h', '.cmake', '.txt', '.md', '.yaml', '.yml', '.py', '.sh', '.ps1'
        }
    foreach ($file in $textFiles) {
        $bytes = [System.IO.File]::ReadAllBytes($file.FullName)
        if ($bytes.Length -ge 3 -and $bytes[0] -eq 0xEF -and $bytes[1] -eq 0xBB -and $bytes[2] -eq 0xBF) {
            throw "UTF-8 BOM is not allowed: $($file.FullName)"
        }
    }

    Write-Host 'VERIFY_WINDOWS=PASS'
}
finally {
    Pop-Location
}
