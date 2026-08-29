# SPDX-License-Identifier: MIT
# build_exe.ps1 - Package the PC service + web UI + AI worker as a portable dir.
# Output: dist\LaserMeterPC\LaserMeterPC.exe (main service, double-click to run)
#         dist\LaserMeterPC\vision\vision_worker.exe (AI, called by main service)
# Usage : powershell -ExecutionPolicy Bypass -File tools\build_exe.ps1
param(
    [switch]$SkipVision,
    [string]$MainPython = '',
    [string]$VisionPython = ''
)
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
Set-Location $root

$mainPy = if ($MainPython) { $MainPython } else { (Get-Command python -ErrorAction Stop).Source }
$venvPy = if ($VisionPython) { $VisionPython } else { $mainPy }
$outDir = Join-Path $root "dist\LaserMeterPC"

if (-not (Test-Path $mainPy)) { throw "main python missing: $mainPy" }
if (-not (Test-Path $venvPy)) { throw "vision python missing: $venvPy" }

# ---------- 1. main service ----------
Write-Host "==> packaging main service ..." -ForegroundColor Cyan
& $mainPy -m PyInstaller --noconfirm --clean --onedir --name LaserMeterPC `
    --distpath (Join-Path $root "dist\main") --workpath (Join-Path $root "build\pyi_main") `
    --add-data "pc_app\static;static" `
    --hidden-import scan_analysis `
    --collect-submodules pc_app `
    pc_app\server.py
if ($LASTEXITCODE -ne 0) { throw "main service packaging failed" }

# ---------- 2. AI worker ----------
if (-not $SkipVision) {
    Write-Host "==> packaging AI worker ..." -ForegroundColor Cyan
    # Vision worker only uses the ONNX backend (onnxruntime InferenceSession).
    # Exclude torch/ultralytics/matplotlib etc. pulled in by static analysis of
    # lazy imports inside door_window_ai (training/evaluation code paths).
    & $venvPy -m PyInstaller --noconfirm --clean --onedir --name vision_worker `
        --distpath (Join-Path $root "dist\vision") --workpath (Join-Path $root "build\pyi_vision") `
        --paths "door_window_ai\src" `
        --collect-submodules door_window_ai `
        --hidden-import onnxruntime `
        --add-data "door_window_ai\models\door_window_outlet_v3;door_window_ai\models\door_window_outlet_v3" `
        --exclude-module ultralytics `
        --exclude-module torch --exclude-module torchvision `
        --exclude-module tensorflow --exclude-module tensorboard `
        --exclude-module matplotlib --exclude-module lap `
        --exclude-module onnx `
        --exclude-module onnxruntime.transformers `
        --exclude-module onnxruntime.tools `
        --exclude-module onnxruntime.quantization `
        --exclude-module tkinter `
        pc_app\vision_worker.py
    if ($LASTEXITCODE -ne 0) { throw "AI worker packaging failed" }
}

# ---------- 3. assemble ----------
Write-Host "==> assembling output ..." -ForegroundColor Cyan
if (Test-Path $outDir) { Remove-Item $outDir -Recurse -Force }
New-Item -ItemType Directory -Path $outDir -Force | Out-Null
Copy-Item (Join-Path $root "dist\main\LaserMeterPC\*") $outDir -Recurse -Force
if (-not $SkipVision) {
    Copy-Item (Join-Path $root "dist\vision\vision_worker") (Join-Path $outDir "vision") -Recurse -Force
}
New-Item -ItemType Directory -Path (Join-Path $outDir "data") -Force | Out-Null

# calibration data (used by photo annotation; optional)
$calibSrc = Join-Path $root "calibration_capture"
if (Test-Path $calibSrc) {
    New-Item -ItemType Directory -Path (Join-Path $outDir "calibration_capture") -Force | Out-Null
    foreach ($f in @("camera_intrinsics.json", "laser_camera_extrinsics.json")) {
        if (Test-Path (Join-Path $calibSrc $f)) {
            Copy-Item (Join-Path $calibSrc $f) (Join-Path $outDir "calibration_capture") -Force
        }
    }
}

# user-facing readme (UTF-8, written by tools\package_readme.txt)
if (Test-Path (Join-Path $root "tools\package_readme.txt")) {
    Copy-Item (Join-Path $root "tools\package_readme.txt") (Join-Path $outDir "README.txt") -Force
}

# ---------- cleanup ----------
Remove-Item (Join-Path $root "dist\main") -Recurse -Force
if (-not $SkipVision) { Remove-Item (Join-Path $root "dist\vision") -Recurse -Force }
Remove-Item (Join-Path $root "build\pyi_main") -Recurse -Force -ErrorAction SilentlyContinue
Remove-Item (Join-Path $root "build\pyi_vision") -Recurse -Force -ErrorAction SilentlyContinue

Write-Host ""
Write-Host "DONE: $outDir" -ForegroundColor Green
$size = (Get-ChildItem $outDir -Recurse -File | Measure-Object -Property Length -Sum).Sum
Write-Host ("total size: {0:N1} MB" -f ($size / 1MB))
