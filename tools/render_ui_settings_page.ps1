# SPDX-License-Identifier: MIT
$ErrorActionPreference = 'Stop'
# Render the settings-page design SVG -> assets/ui/settings_page/reference.png (240x284)
$projectRoot = Split-Path -Parent $PSScriptRoot
$sourceRoot = Join-Path $projectRoot 'assets\ui\settings_page'
$svgPath = Join-Path $sourceRoot 'settings.svg'
$outputPath = Join-Path $sourceRoot 'reference.png'
$edge = 'C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe'
if (-not (Test-Path $edge)) { $edge = 'C:\Program Files\Microsoft\Edge\Application\msedge.exe' }
if (-not (Test-Path $edge)) { throw 'Edge not found' }

# Use <img> with an exact pixel size so the window margins cannot shift the crop.
$html = "<!doctype html><html><head><style>html,body{margin:0;padding:0;overflow:hidden;background:#000}</style></head><body><img src='file:///$($svgPath.Replace('\','/'))' width='240' height='284'></body></html>"
$htmlPath = Join-Path $sourceRoot 'render.html'
[IO.File]::WriteAllText($htmlPath, $html, [Text.UTF8Encoding]::new($false))
$profile = Join-Path $sourceRoot 'edge-profile'
$args = @('--headless=new','--disable-gpu','--hide-scrollbars','--force-device-scale-factor=1',
          '--window-size=240,284', '--default-background-color=00000000',
          "--user-data-dir=$profile", "--screenshot=$outputPath", ([Uri]::new($htmlPath).AbsoluteUri))
$process = Start-Process -FilePath $edge -ArgumentList $args -Wait -PassThru -WindowStyle Hidden
if ($process.ExitCode -ne 0) { throw "Edge render failed: $($process.ExitCode)" }
Remove-Item -LiteralPath $htmlPath -ErrorAction SilentlyContinue
Write-Output "Rendered: $outputPath"
