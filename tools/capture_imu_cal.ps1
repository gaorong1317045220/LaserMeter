# SPDX-License-Identifier: MIT
param(
    [string]$PortName = "COM27",
    [string]$OutputDir = ".\calibration_capture"
)

$ErrorActionPreference = "Stop"
$port = [System.IO.Ports.SerialPort]::new($PortName, 115200, 'None', 8, 'One')
$port.ReadTimeout = 300
$port.WriteTimeout = 1000
$port.DtrEnable = $false
$port.RtsEnable = $false

try {
    $port.Open()
    $port.Write("help`n")
    $ready = ""
    $deadline = [DateTime]::UtcNow.AddSeconds(50)
    while ([DateTime]::UtcNow -lt $deadline -and $ready -notmatch 'selftest>') {
        Start-Sleep -Milliseconds 200
        $ready += $port.ReadExisting()
    }
    if ($ready -notmatch 'selftest>') { throw "Timed out waiting for selftest prompt" }

    $port.Write("imu_cal_dump`n")
    $dump = ""
    $deadline = [DateTime]::UtcNow.AddMinutes(5)
    while ([DateTime]::UtcNow -lt $deadline -and $dump -notmatch '=== IMU CAL DUMP END ===') {
        Start-Sleep -Milliseconds 100
        $dump += $port.ReadExisting()
    }
    if ($dump -notmatch '=== IMU CAL DUMP END ===') { throw "Timed out receiving calibration dump" }
} finally {
    if ($port.IsOpen) { $port.Close() }
    $port.Dispose()
}

[IO.Directory]::CreateDirectory($OutputDir) | Out-Null
$names = @('imu_calibration.csv', 'events.csv', 'bno086.csv')
foreach ($name in $names) {
    $begin = "=== FILE BEGIN $([regex]::Escape($name)) SIZE=(\d+) ===\r?\n"
    $end = "\r?\n=== FILE END $([regex]::Escape($name)) ==="
    $match = [regex]::Match($dump, $begin + '(.*?)' + $end, [Text.RegularExpressions.RegexOptions]::Singleline)
    if (-not $match.Success) { throw "Missing framed file: $name" }
    # ESP-IDF UART console expands each LF from the file to CRLF. Restore the
    # original bytes before checking the size advertised by the device.
    $content = $match.Groups[2].Value -replace "`r`n", "`n"
    $expected = [int64]$match.Groups[1].Value
    $bytes = [Text.Encoding]::UTF8.GetBytes($content)
    if ($bytes.Length -ne $expected) {
        throw "Size mismatch for ${name}: received $($bytes.Length), expected $expected"
    }
    [IO.File]::WriteAllBytes((Join-Path $OutputDir $name), $bytes)
    Write-Output "$name $expected"
}
