# SPDX-License-Identifier: MIT
$ErrorActionPreference = 'Stop'
# Generate ui_menu_camera asset from the supplied camera icon (white -> transparent, ARGB8565)
$projectRoot = Split-Path -Parent $PSScriptRoot
$source = Join-Path $projectRoot 'assets\ui\menu_pages\camera_icon.png'
if (-not (Test-Path -LiteralPath $source)) { throw "Missing camera icon: $source" }
$output = Join-Path $projectRoot 'main\ui_menu_camera_assets.c'
$temporaryOutput = Join-Path ([IO.Path]::GetTempPath()) ("laser-camera-assets-{0}.c" -f [Guid]::NewGuid())

Add-Type -AssemblyName System.Drawing

$W = 134; $H = 175
$sourceBmp = [Drawing.Bitmap]::new($source)
$canvas = [Drawing.Bitmap]::new($W, $H, [Drawing.Imaging.PixelFormat]::Format32bppArgb)
$graphics = [Drawing.Graphics]::FromImage($canvas)
try {
    $graphics.Clear([Drawing.Color]::Transparent)
    $graphics.CompositingMode = [Drawing.Drawing2D.CompositingMode]::SourceCopy
    $graphics.InterpolationMode = [Drawing.Drawing2D.InterpolationMode]::HighQualityBicubic
    # Content bbox is ~942x762 centered in the 1254 square. Fit it into the
    # 134x175 canvas keeping the aspect ratio, centered vertically.
    $srcW = 942; $srcH = 762
    $scale = [Math]::Min([double]$W / $srcW, [double]$H / $srcH)
    $drawW = [Math]::Round($srcW * $scale)
    $drawH = [Math]::Round($srcH * $scale)
    $drawX = [Math]::Round(($W - $drawW) / 2)
    $drawY = [Math]::Round(($H - $drawH) / 2)
    $srcX = [Math]::Round((1254 - $srcW) / 2); $srcY = [Math]::Round((1254 - $srcH) / 2)
    $graphics.DrawImage($sourceBmp,
        [Drawing.Rectangle]::new($drawX, $drawY, $drawW, $drawH),
        [Drawing.Rectangle]::new($srcX, $srcY, $srcW, $srcH),
        [Drawing.GraphicsUnit]::Pixel)
} finally { $graphics.Dispose(); $sourceBmp.Dispose() }

$writer = [IO.StreamWriter]::new($temporaryOutput, $false, [Text.UTF8Encoding]::new($false))
try {
    $writer.WriteLine('// Generated from the supplied camera icon artwork. Do not edit.')
    $writer.WriteLine('#include "lvgl.h"')
    $writer.WriteLine()
    $writer.WriteLine("static const uint8_t ui_menu_camera_map[] = {")
    $column = 0
    for ($y = 0; $y -lt $H; ++$y) {
        for ($x = 0; $x -lt $W; ++$x) {
            $pixel = $canvas.GetPixel($x, $y)
            $alpha = [int]$pixel.A
            # White-ish background becomes fully transparent.
            if ($pixel.R -ge 245 -and $pixel.G -ge 245 -and $pixel.B -ge 245) { $alpha = 0 }
            # The camera graphic is rendered pure white (LVGL shows the alpha
            # shape as-is; the artwork stays white instead of blue-grey).
            $rgb565 = 0xFFFF
            foreach ($byte in @(($rgb565 -band 0xFF), (($rgb565 -shr 8) -band 0xFF), $alpha)) {
                $writer.Write(('0x{0:X2},' -f $byte)); ++$column
                if ($column -eq 24) { $writer.WriteLine(); $column = 0 }
            }
        }
    }
    if ($column -ne 0) { $writer.WriteLine() }
    $writer.WriteLine('};')
    $writer.WriteLine("const lv_img_dsc_t ui_menu_camera = {")
    $writer.WriteLine("    .header = {.cf = LV_IMG_CF_TRUE_COLOR_ALPHA, .always_zero = 0, .reserved = 0, .w = $W, .h = $H},")
    $writer.WriteLine("    .data_size = sizeof(ui_menu_camera_map), .data = ui_menu_camera_map,")
    $writer.WriteLine('};')
    $writer.WriteLine()
} finally { $writer.Dispose(); $canvas.Dispose() }

if ((Test-Path -LiteralPath $output) -and
    ((Get-FileHash -LiteralPath $output -Algorithm SHA256).Hash -eq
     (Get-FileHash -LiteralPath $temporaryOutput -Algorithm SHA256).Hash)) {
    Remove-Item -LiteralPath $temporaryOutput
    Write-Output "Up to date: $output"
} else {
    Move-Item -LiteralPath $temporaryOutput -Destination $output -Force
    Write-Output "Generated: $output"
}
