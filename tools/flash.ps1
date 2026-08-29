# SPDX-License-Identifier: MIT
param(
    [Parameter(Mandatory = $true)][string]$Port,
    [string]$IdfPath = '',
    [switch]$EraseFlash
)

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$buildScript = Join-Path $PSScriptRoot 'build.ps1'

& $buildScript -IdfPath $IdfPath
if ($LASTEXITCODE -ne 0) {
    throw "Build failed with exit code $LASTEXITCODE"
}

if ($IdfPath) {
    $exportScript = Join-Path $IdfPath 'export.ps1'
    if (-not (Test-Path -LiteralPath $exportScript)) {
        throw "ESP-IDF export script not found: $exportScript"
    }
    & $exportScript
}

$idf = Get-Command idf.py -ErrorAction Stop
$python = Get-Command python -ErrorAction Stop
$idfScript = Join-Path $env:IDF_PATH 'tools\idf.py'
if (-not (Test-Path -LiteralPath $idfScript)) {
    $idfScript = $idf.Source
}
if ($EraseFlash) {
    & $python.Source $idfScript -C $projectRoot -B (Join-Path $projectRoot 'build') -p $Port erase-flash
    if ($LASTEXITCODE -ne 0) { throw "Erase failed with exit code $LASTEXITCODE" }
}

& $python.Source $idfScript -C $projectRoot -B (Join-Path $projectRoot 'build') -p $Port flash
if ($LASTEXITCODE -ne 0) { throw "Flash failed with exit code $LASTEXITCODE" }

Write-Output "Flash OK: $Port"
Write-Output 'To view serial logs, run: idf.py -p <PORT> monitor'
