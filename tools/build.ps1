# SPDX-License-Identifier: MIT
param(
    [string]$IdfPath = '',
    [string]$BuildDir = ''
)

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
if ($projectRoot -match '[^\x00-\x7F]') {
    throw 'On Windows, keep the project in an ASCII-only path before building with ESP-IDF.'
}
if (-not $BuildDir) {
    $BuildDir = Join-Path $projectRoot 'build'
} elseif (-not [IO.Path]::IsPathRooted($BuildDir)) {
    $BuildDir = Join-Path $projectRoot $BuildDir
}
$BuildDir = [IO.Path]::GetFullPath($BuildDir)

if ($IdfPath) {
    if (-not $env:IDF_PYTHON_ENV_PATH) {
        $idfToolsRoot = if ($env:IDF_TOOLS_PATH) {
            $env:IDF_TOOLS_PATH
        } else {
            Join-Path $env:USERPROFILE '.espressif'
        }
        $pythonEnv = Join-Path $idfToolsRoot 'python_env\idf5.5_py3.12_env'
        if (Test-Path -LiteralPath (Join-Path $pythonEnv 'Scripts\python.exe')) {
            $env:IDF_PYTHON_ENV_PATH = $pythonEnv
        }
    }
    $exportScript = Join-Path $IdfPath 'export.ps1'
    if (-not (Test-Path -LiteralPath $exportScript)) {
        throw "ESP-IDF export script not found: $exportScript"
    }
    & $exportScript
} elseif (-not $env:IDF_PATH) {
    throw 'Open an ESP-IDF 5.5.2 PowerShell first, or pass -IdfPath.'
}

$idf = Get-Command idf.py -ErrorAction Stop
$python = Get-Command python -ErrorAction Stop
$idfScript = Join-Path $env:IDF_PATH 'tools\idf.py'
if (-not (Test-Path -LiteralPath $idfScript)) {
    $idfScript = $idf.Source
}
$idfVersion = (& $python.Source $idfScript --version 2>&1 | Out-String).Trim()
if ($LASTEXITCODE -ne 0) { throw "Unable to query ESP-IDF version: $idfVersion" }
if ($idfVersion -notmatch '5\.5\.2') {
    throw "ESP-IDF 5.5.2 is required for the verified build. Found: $idfVersion"
}
$env:IDF_COMPONENT_MANAGER = '0'
$env:IDF_SKIP_CHECK_SUBMODULES = '1'

$layoutGenerator = Join-Path $projectRoot 'tools\generate_ui_menu_layout.py'
$assetGenerator = Join-Path $projectRoot 'tools\generate_ui_menu_assets.ps1'
$singleAssetGenerator = Join-Path $projectRoot 'tools\generate_ui_single_assets.ps1'
$calibrationSync = Join-Path $projectRoot 'tools\sync_laser_camera_calibration.py'
& $python.Source $calibrationSync
if ($LASTEXITCODE -ne 0) { throw "Laser-camera calibration sync failed with exit code $LASTEXITCODE" }
& $python.Source $layoutGenerator
if ($LASTEXITCODE -ne 0) { throw "UI layout generation failed with exit code $LASTEXITCODE" }
& $assetGenerator
if ($LASTEXITCODE -ne 0) { throw "UI asset generation failed with exit code $LASTEXITCODE" }
& $singleAssetGenerator
if ($LASTEXITCODE -ne 0) { throw "Single UI asset generation failed with exit code $LASTEXITCODE" }

& $python.Source $idfScript -C $projectRoot -B $BuildDir build
if ($LASTEXITCODE -ne 0) { throw "Build failed with exit code $LASTEXITCODE" }

$image = Join-Path $BuildDir 'board_self_test.bin'
$artifact = Get-Item -LiteralPath $image
$hash = Get-FileHash -LiteralPath $image -Algorithm SHA256

Write-Output "Build OK"
Write-Output "Image: $($artifact.FullName)"
Write-Output "Bytes: $($artifact.Length)"
Write-Output "SHA256: $($hash.Hash)"
