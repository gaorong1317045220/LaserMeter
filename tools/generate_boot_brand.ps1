# SPDX-License-Identifier: MIT
$ErrorActionPreference = 'Stop'
# Render the boot brand bitmap (240x92) with real text and emit an RGB565 C array.
$projectRoot = Split-Path -Parent $PSScriptRoot
$output = Join-Path $projectRoot 'main\nv3030b_boot_brand.c'
$temporaryOutput = Join-Path ([IO.Path]::GetTempPath()) ("laser-boot-brand-{0}.c" -f [Guid]::NewGuid())

Add-Type -AssemblyName System.Drawing

$W = 240; $H = 92
$bmp = [Drawing.Bitmap]::new($W, $H, [Drawing.Imaging.PixelFormat]::Format24bppRgb)
$g = [Drawing.Graphics]::FromImage($bmp)
try {
    $g.TextRenderingHint = [Drawing.Text.TextRenderingHint]::AntiAliasGridFit
    $g.Clear([Drawing.Color]::FromArgb(0x10, 0x1B, 0x29))
    $sf = [Drawing.StringFormat]::new()
    $sf.Alignment = [Drawing.StringAlignment]::Center
    $font1 = [Drawing.Font]::new('Microsoft YaHei', 26, [Drawing.FontStyle]::Bold, [Drawing.GraphicsUnit]::Pixel)
    $brush = [Drawing.SolidBrush]::new([Drawing.Color]::White)
    $g.DrawString('激光测距仪', $font1, $brush, [Drawing.RectangleF]::new(0, 20, $W, 40), $sf)
    $font2 = [Drawing.Font]::new('Microsoft YaHei', 13, [Drawing.FontStyle]::Regular, [Drawing.GraphicsUnit]::Pixel)
    $brush2 = [Drawing.SolidBrush]::new([Drawing.Color]::FromArgb(0x8F, 0xA6, 0xC8))
    $g.DrawString('智能量房 · 便携激光', $font2, $brush2, [Drawing.RectangleF]::new(0, 58, $W, 22), $sf)
    $font1.Dispose(); $font2.Dispose(); $brush.Dispose(); $brush2.Dispose(); $sf.Dispose()
} finally { $g.Dispose() }
# Also save a preview PNG for the web preview page.
$previewPath = Join-Path $projectRoot 'docs\boot_brand.png'
$bmp.Save($previewPath, [Drawing.Imaging.ImageFormat]::Png)

$writer = [IO.StreamWriter]::new($temporaryOutput, $false, [Text.UTF8Encoding]::new($false))
try {
    $writer.WriteLine('// Generated boot brand bitmap (240x92 RGB565). Do not edit.')
    $writer.WriteLine('#include "lvgl.h"')
    $writer.WriteLine()
    $writer.WriteLine("static const uint8_t nv3030b_boot_brand_map[] = {")
    $column = 0
    for ($y = 0; $y -lt $H; ++$y) {
        for ($x = 0; $x -lt $W; ++$x) {
            $p = $bmp.GetPixel($x, $y)
            $rgb565 = (([int]$p.R -shr 3) -shl 11) -bor
                      (([int]$p.G -shr 2) -shl 5) -bor ([int]$p.B -shr 3)
            foreach ($byte in @(($rgb565 -band 0xFF), (($rgb565 -shr 8) -band 0xFF))) {
                $writer.Write(('0x{0:X2},' -f $byte)); ++$column
                if ($column -eq 24) { $writer.WriteLine(); $column = 0 }
            }
        }
    }
    if ($column -ne 0) { $writer.WriteLine() }
    $writer.WriteLine('};')
    $writer.WriteLine("const lv_img_dsc_t nv3030b_boot_brand = {")
    $writer.WriteLine("    .header = {.cf = LV_IMG_CF_TRUE_COLOR, .always_zero = 0, .reserved = 0, .w = $W, .h = $H},")
    $writer.WriteLine("    .data_size = sizeof(nv3030b_boot_brand_map), .data = nv3030b_boot_brand_map,")
    $writer.WriteLine('};')
    $writer.WriteLine()
} finally { $writer.Dispose(); $bmp.Dispose() }

if ((Test-Path -LiteralPath $output) -and
    ((Get-FileHash -LiteralPath $output -Algorithm SHA256).Hash -eq
     (Get-FileHash -LiteralPath $temporaryOutput -Algorithm SHA256).Hash)) {
    Remove-Item -LiteralPath $temporaryOutput
    Write-Output "Up to date: $output"
} else {
    Move-Item -LiteralPath $temporaryOutput -Destination $output -Force
    Write-Output "Generated: $output"
}
