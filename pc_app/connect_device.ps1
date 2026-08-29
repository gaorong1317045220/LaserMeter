# SPDX-License-Identifier: MIT
$ErrorActionPreference = "Stop"

$profileName = "LASER-METER-SETUP"
$interfaceName = "WLAN"
$profilePath = Join-Path $env:TEMP "laser-meter-setup-wifi.xml"
$profileXml = @"
<?xml version="1.0"?>
<WLANProfile xmlns="http://www.microsoft.com/networking/WLAN/profile/v1">
  <name>LASER-METER-SETUP</name>
  <SSIDConfig><SSID><name>LASER-METER-SETUP</name></SSID></SSIDConfig>
  <connectionType>ESS</connectionType>
  <connectionMode>manual</connectionMode>
  <MSM><security>
    <authEncryption><authentication>WPA2PSK</authentication><encryption>AES</encryption><useOneX>false</useOneX></authEncryption>
    <sharedKey><keyType>passPhrase</keyType><protected>false</protected><keyMaterial>12345678</keyMaterial></sharedKey>
  </security></MSM>
</WLANProfile>
"@

try {
    Set-Content -LiteralPath $profilePath -Value $profileXml -Encoding utf8
    netsh wlan add profile filename="$profilePath" interface="$interfaceName" user=current | Out-Host
    netsh wlan connect name="$profileName" ssid="$profileName" interface="$interfaceName" | Out-Host
    Write-Host "Connecting to $profileName. Then open http://127.0.0.1:8000/ to bind the device."
}
finally {
    Remove-Item -LiteralPath $profilePath -Force -ErrorAction SilentlyContinue
}
