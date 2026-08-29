# SPDX-License-Identifier: MIT
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$sourceRoot = Join-Path $projectRoot 'assets\ui\single_distance'
$outputRoot = Join-Path $sourceRoot 'generated'
$edge = 'C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe'
New-Item -ItemType Directory -Force -Path $outputRoot | Out-Null

$assets = @(
    @{Name='single_title'; File='single_title.svg'; W=74; H=24},
    @{Name='reference_front'; File='reference_front.svg'; W=48; H=20},
    @{Name='reference_tripod'; File='reference_tripod.svg'; W=48; H=20},
    @{Name='zoom_1x'; File='1X.svg'; W=25; H=18},
    @{Name='zoom_2x'; File='2X.svg'; W=25; H=18},
    @{Name='zoom_5x'; File='5X.svg'; W=25; H=18},
    @{Name='laser_label'; File='Label.svg'; W=24; H=15},
    @{Name='laser_off'; File='Vector.svg'; W=24; H=16},
    @{Name='level_track'; File='Slider-Track.svg'; W=8; H=100},
    @{Name='crosshair'; File='crosshair.svg'; W=75; H=73},
    @{Name='bottom_history'; File='bottom_history.svg'; W=64; H=24},
    @{Name='bottom_measure'; File='bottom_measure.svg'; W=64; H=24},
    @{Name='bottom_back'; File='bottom_back.svg'; W=64; H=24},
    @{Name='bottom_save'; File='bottom_save.svg'; W=64; H=24},
    @{Name='bottom_delete'; File='bottom_delete.svg'; W=64; H=24},
    @{Name='bottom_select'; File='bottom_select.svg'; W=64; H=24},
    @{Name='bottom_export'; File='bottom_export.svg'; W=64; H=24}
)

$x = 0; $y = 0; $rowHeight = 0; $sheetWidth = 512
$html = '<!doctype html><html><head><style>html,body{margin:0;background:transparent;overflow:hidden}img{position:absolute}</style></head><body>'
foreach ($asset in $assets) {
    if ($x + $asset.W -gt $sheetWidth) { $x = 0; $y += $rowHeight + 4; $rowHeight = 0 }
    $asset.X = $x; $asset.Y = $y
    $sourcePath = Join-Path $sourceRoot $asset.File
    $encoded = [Convert]::ToBase64String([IO.File]::ReadAllBytes($sourcePath))
    $html += "<img src='data:image/svg+xml;base64,$encoded' style='left:${x}px;top:${y}px;width:$($asset.W)px;height:$($asset.H)px'>"
    $x += $asset.W + 4; $rowHeight = [Math]::Max($rowHeight, $asset.H)
}
$sheetHeight = $y + $rowHeight
$html += '</body></html>'
$htmlPath = Join-Path $outputRoot 'asset-sheet.html'
$sheetPath = Join-Path $outputRoot 'asset-sheet.png'
[IO.File]::WriteAllText($htmlPath, $html, [Text.UTF8Encoding]::new($false))
$profile = Join-Path $outputRoot 'edge-profile'
$args = @('--headless=new','--disable-gpu','--hide-scrollbars','--force-device-scale-factor=1',
          "--window-size=$sheetWidth,$sheetHeight", '--default-background-color=00000000',
          "--user-data-dir=$profile", "--screenshot=$sheetPath", ([Uri]::new($htmlPath).AbsoluteUri))
$process = Start-Process -FilePath $edge -ArgumentList $args -Wait -PassThru -WindowStyle Hidden
if ($process.ExitCode -ne 0) { throw "Edge asset render failed: $($process.ExitCode)" }

Add-Type -AssemblyName System.Drawing
$sheet = [Drawing.Bitmap]::new($sheetPath)
try {
    foreach ($asset in $assets) {
        $bitmap = [Drawing.Bitmap]::new($asset.W, $asset.H, [Drawing.Imaging.PixelFormat]::Format32bppArgb)
        $graphics = [Drawing.Graphics]::FromImage($bitmap)
        try {
            $graphics.CompositingMode = [Drawing.Drawing2D.CompositingMode]::SourceCopy
            $graphics.DrawImage($sheet, [Drawing.Rectangle]::new(0,0,$asset.W,$asset.H),
                [Drawing.Rectangle]::new($asset.X,$asset.Y,$asset.W,$asset.H), [Drawing.GraphicsUnit]::Pixel)
        } finally { $graphics.Dispose() }
        try { $bitmap.Save((Join-Path $outputRoot ($asset.Name + '.png')), [Drawing.Imaging.ImageFormat]::Png) }
        finally { $bitmap.Dispose() }
    }
} finally { $sheet.Dispose() }
Write-Output "Rendered $($assets.Count) transparent UI assets: $outputRoot"
