# SPDX-License-Identifier: MIT
$ErrorActionPreference = 'Stop'
$listener = Get-NetTCPConnection -LocalPort 8000 -State Listen -ErrorAction SilentlyContinue
if ($listener) {
    Write-Host "Laser meter PC service is already running. PID=$($listener.OwningProcess)"
    Write-Host 'Open http://127.0.0.1:8000/'
    exit 0
}
$python = Get-Command python -ErrorAction Stop
Write-Host 'Laser meter PC service: http://127.0.0.1:8000/'
& $python.Source "$PSScriptRoot\server.py"
