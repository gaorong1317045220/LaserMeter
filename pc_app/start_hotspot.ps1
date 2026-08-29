# SPDX-License-Identifier: MIT
$ErrorActionPreference = "Stop"

Add-Type -AssemblyName System.Runtime.WindowsRuntime
[Windows.Networking.Connectivity.NetworkInformation,Windows.Networking.Connectivity,ContentType=WindowsRuntime] | Out-Null
[Windows.Networking.NetworkOperators.NetworkOperatorTetheringManager,Windows.Networking.NetworkOperators,ContentType=WindowsRuntime] | Out-Null
[Windows.Networking.NetworkOperators.NetworkOperatorTetheringAccessPointConfiguration,Windows.Networking.NetworkOperators,ContentType=WindowsRuntime] | Out-Null

$profile = [Windows.Networking.Connectivity.NetworkInformation]::GetInternetConnectionProfile()
if (-not $profile) {
    throw "No Internet connection profile is available for hotspot sharing."
}
$capability = [Windows.Networking.NetworkOperators.NetworkOperatorTetheringManager]::GetTetheringCapabilityFromConnectionProfile($profile)
if ($capability -ne "Enabled") {
    throw "Windows mobile hotspot is unavailable: $capability"
}

$manager = [Windows.Networking.NetworkOperators.NetworkOperatorTetheringManager]::CreateFromConnectionProfile($profile)
$config = [Windows.Networking.NetworkOperators.NetworkOperatorTetheringAccessPointConfiguration]::new()
$config.Ssid = "LASER-PC"
$config.Passphrase = "12345678"
$manager.ConfigureAccessPointAsync($config) | Out-Null

for ($attempt = 0; $attempt -lt 20; $attempt++) {
    Start-Sleep -Milliseconds 100
    $current = $manager.GetCurrentAccessPointConfiguration()
    if ($current.Ssid -eq $config.Ssid -and $current.Passphrase -eq $config.Passphrase) { break }
}
$manager.StartTetheringAsync() | Out-Null
for ($attempt = 0; $attempt -lt 40; $attempt++) {
    Start-Sleep -Milliseconds 250
    if ($manager.TetheringOperationalState -eq "On") { break }
}
if ($manager.TetheringOperationalState -ne "On") {
    throw "Windows mobile hotspot did not enter the On state."
}
Write-Host "Hotspot LASER-PC is On. Password: 12345678"
