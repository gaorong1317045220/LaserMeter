# SPDX-License-Identifier: MIT
param(
    [string]$IdfPath = '',
    [string]$PythonPath = '',
    [switch]$SkipBuild,
    [switch]$SkipPcBuild
)

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$release = Join-Path $root 'Release'
$build = Join-Path $root 'build'

if (-not $PythonPath) {
    $PythonPath = Join-Path $root '.venv\Scripts\python.exe'
}
if (-not (Test-Path -LiteralPath $PythonPath)) {
    throw "Python environment not found: $PythonPath"
}

if ($IdfPath -and -not $env:IDF_PYTHON_ENV_PATH) {
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

if (-not $SkipBuild) {
    & (Join-Path $PSScriptRoot 'build.ps1') -IdfPath $IdfPath
    if ($LASTEXITCODE -ne 0) { throw "Firmware build failed with exit code $LASTEXITCODE" }
}

foreach ($required in @(
    'bootloader\bootloader.bin',
    'partition_table\partition-table.bin',
    'board_self_test.bin'
)) {
    if (-not (Test-Path -LiteralPath (Join-Path $build $required))) {
        throw "Firmware artifact missing: $required"
    }
}

if ($IdfPath) {
    $exportScript = Join-Path $IdfPath 'export.ps1'
    if (-not (Test-Path -LiteralPath $exportScript)) { throw "ESP-IDF export script not found: $exportScript" }
    & $exportScript
}
if (-not $env:IDF_PATH) {
    throw 'Open an ESP-IDF 5.5.2 PowerShell first, or pass -IdfPath.'
}

$idfPython = (Get-Command python -ErrorAction Stop).Source
$esptool = Join-Path $env:IDF_PATH 'components\esptool_py\esptool\esptool.py'
if (-not (Test-Path -LiteralPath $esptool)) { throw "esptool not found: $esptool" }

if (Test-Path -LiteralPath $release) {
    Remove-Item -LiteralPath $release -Recurse -Force
}
$firmwareOut = Join-Path $release 'firmware'
New-Item -ItemType Directory -Path $firmwareOut -Force | Out-Null

$merged = Join-Path $firmwareOut 'laser_meter_full.bin'
& $idfPython $esptool --chip esp32s3 merge_bin `
    --output $merged --flash_mode dio --flash_freq 80m --flash_size 16MB `
    0x0 (Join-Path $build 'bootloader\bootloader.bin') `
    0x8000 (Join-Path $build 'partition_table\partition-table.bin') `
    0x10000 (Join-Path $build 'board_self_test.bin')
if ($LASTEXITCODE -ne 0) { throw "Firmware merge failed with exit code $LASTEXITCODE" }

$firmwareHash = Get-FileHash -LiteralPath $merged -Algorithm SHA256
"$($firmwareHash.Hash)  laser_meter_full.bin" | Set-Content -LiteralPath (Join-Path $firmwareOut 'SHA256SUMS.txt') -Encoding ASCII

if (-not $SkipPcBuild) {
    & (Join-Path $PSScriptRoot 'build_exe.ps1') -MainPython $PythonPath -VisionPython $PythonPath
    if ($LASTEXITCODE -ne 0) { throw "PC package build failed with exit code $LASTEXITCODE" }

    $pcSource = Join-Path $root 'dist\LaserMeterPC'
    $pcOut = Join-Path $release 'pc'
    Copy-Item -LiteralPath $pcSource -Destination $pcOut -Recurse -Force
    $oldExe = Join-Path $pcOut 'LaserMeterPC.exe'
    if (-not (Test-Path -LiteralPath $oldExe)) { throw "PC executable missing: $oldExe" }
    Rename-Item -LiteralPath $oldExe -NewName 'LaserMeter.exe'

    Copy-Item -LiteralPath (Join-Path $root 'LICENSE') -Destination (Join-Path $pcOut 'LICENSE.txt')
    Copy-Item -LiteralPath (Join-Path $root 'THIRD_PARTY_NOTICES.md') -Destination $pcOut
    Copy-Item -LiteralPath (Join-Path $root 'door_window_ai\models\door_window_outlet_v3\MODEL_LICENSE.md') -Destination $pcOut
    & $PythonPath (Join-Path $PSScriptRoot 'collect_python_licenses.py') (Join-Path $pcOut 'licenses\python')
    if ($LASTEXITCODE -ne 0) { throw "Python license collection failed with exit code $LASTEXITCODE" }
    $flatbuffersLicenseDir = Join-Path $pcOut 'licenses\python\flatbuffers-25.12.19'
    New-Item -ItemType Directory -Path $flatbuffersLicenseDir -Force | Out-Null
    Copy-Item -LiteralPath (Join-Path $root 'managed_components\espressif__esp32-camera\LICENSE') `
        -Destination (Join-Path $flatbuffersLicenseDir 'LICENSE-Apache-2.0.txt')
}

Copy-Item -LiteralPath (Join-Path $PSScriptRoot 'release_readme.md') -Destination (Join-Path $release 'README.md')
Copy-Item -LiteralPath (Join-Path $root 'LICENSE') -Destination (Join-Path $release 'LICENSE.txt')
Copy-Item -LiteralPath (Join-Path $root 'docs\LICENSE.md') -Destination (Join-Path $release 'DOCUMENTATION_LICENSE.md')
Copy-Item -LiteralPath (Join-Path $root 'THIRD_PARTY_NOTICES.md') -Destination $release

$firmwareLicenses = Join-Path $firmwareOut 'licenses'
New-Item -ItemType Directory -Path $firmwareLicenses -Force | Out-Null
Copy-Item -LiteralPath (Join-Path $env:IDF_PATH 'LICENSE') -Destination (Join-Path $firmwareLicenses 'ESP-IDF-LICENSE.txt')
Copy-Item -LiteralPath (Join-Path $root 'managed_components\lvgl__lvgl\LICENCE.txt') -Destination (Join-Path $firmwareLicenses 'LVGL-LICENSE.txt')
Copy-Item -LiteralPath (Join-Path $root 'managed_components\espressif__esp32-camera\LICENSE') -Destination (Join-Path $firmwareLicenses 'esp32-camera-LICENSE.txt')
Copy-Item -LiteralPath (Join-Path $root 'managed_components\espressif__esp_jpeg\license.txt') -Destination (Join-Path $firmwareLicenses 'esp_jpeg-LICENSE.txt')
Copy-Item -LiteralPath (Join-Path $root 'managed_components\espressif__esp_new_jpeg\LICENSE') -Destination (Join-Path $firmwareLicenses 'esp_new_jpeg-LICENSE.txt')

$releaseHashLines = Get-ChildItem -LiteralPath $release -Recurse -File | Sort-Object FullName | ForEach-Object {
    $relative = $_.FullName.Substring($release.Length + 1).Replace('\', '/')
    $hash = (Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash
    "$hash  $relative"
}
$releaseHashLines | Set-Content -LiteralPath (Join-Path $release 'SHA256SUMS.txt') -Encoding ASCII

Write-Output "Release package: $release"
Write-Output "Merged firmware SHA256: $($firmwareHash.Hash)"
