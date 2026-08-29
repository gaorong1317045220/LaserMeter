# SPDX-License-Identifier: MIT
$ErrorActionPreference = "Stop"

$listeners = Get-NetTCPConnection -LocalPort 8000,8765 -State Listen -ErrorAction SilentlyContinue
$processIds = @($listeners | Select-Object -ExpandProperty OwningProcess -Unique)
foreach ($processId in $processIds) {
    $process = Get-CimInstance Win32_Process -Filter "ProcessId=$processId" -ErrorAction SilentlyContinue
    if ($process -and $process.Name -match '^pythonw?\.exe$' -and
        $process.CommandLine -like '*pc_app*server.py*') {
        Stop-Process -Id $processId -Force
        Write-Host "Stopped laser meter PC service. PID=$processId"
    }
}
