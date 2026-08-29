# SPDX-License-Identifier: MIT
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$output = Join-Path $projectRoot 'main\ui_menu_assets.c'
$temporaryOutput = Join-Path ([IO.Path]::GetTempPath()) ("laser-ui-assets-{0}.c" -f [Guid]::NewGuid())

Add-Type -AssemblyName System.Drawing

function Copy-Crop {
    param([Drawing.Bitmap]$Source, [int]$X, [int]$Y, [int]$Width, [int]$Height)
    $result = [Drawing.Bitmap]::new($Width, $Height, [Drawing.Imaging.PixelFormat]::Format32bppArgb)
    $graphics = [Drawing.Graphics]::FromImage($result)
    try {
        $graphics.CompositingMode = [Drawing.Drawing2D.CompositingMode]::SourceCopy
        $graphics.DrawImage($Source, [Drawing.Rectangle]::new(0, 0, $Width, $Height),
            [Drawing.Rectangle]::new($X, $Y, $Width, $Height), [Drawing.GraphicsUnit]::Pixel)
    } finally { $graphics.Dispose() }
    return $result
}

# ALPHA_4BIT shape asset, recolored by the firmware (title / wifi / battery / arrows).
function Write-LvglAlpha4Image {
    param([IO.StreamWriter]$Writer, [string]$Name, [Drawing.Bitmap]$Bitmap,
          [int]$ForegroundRed, [int]$ForegroundGreen, [int]$ForegroundBlue)
    $Writer.WriteLine("static const uint8_t ${Name}_map[] = {")
    $column = 0
    for ($y = 0; $y -lt $Bitmap.Height; ++$y) {
        for ($x = 0; $x -lt $Bitmap.Width; $x += 2) {
            $packed = 0
            for ($part = 0; $part -lt 2; ++$part) {
                $nibble = 0
                if ($x + $part -lt $Bitmap.Width) {
                    $pixel = $Bitmap.GetPixel($x + $part, $y)
                    $ratios = @()
                    if ($ForegroundRed -gt 0) { $ratios += ([double]$pixel.R / $ForegroundRed) }
                    if ($ForegroundGreen -gt 0) { $ratios += ([double]$pixel.G / $ForegroundGreen) }
                    if ($ForegroundBlue -gt 0) { $ratios += ([double]$pixel.B / $ForegroundBlue) }
                    $alpha = if ($ratios.Count) { [Math]::Min(1.0, ($ratios | Measure-Object -Average).Average) } else { 0 }
                    $nibble = [Math]::Min(15, [Math]::Max(0, [Math]::Round($alpha * 15)))
                }
                if ($part -eq 0) { $packed = $nibble -shl 4 } else { $packed = $packed -bor $nibble }
            }
            $Writer.Write(('0x{0:X2},' -f $packed)); ++$column
            if ($column -eq 24) { $Writer.WriteLine(); $column = 0 }
        }
    }
    if ($column -ne 0) { $Writer.WriteLine() }
    $Writer.WriteLine('};')
    $Writer.WriteLine("const lv_img_dsc_t $Name = {")
    $Writer.WriteLine("    .header = {.cf = LV_IMG_CF_ALPHA_4BIT, .always_zero = 0, .reserved = 0, .w = $($Bitmap.Width), .h = $($Bitmap.Height)},")
    $Writer.WriteLine("    .data_size = sizeof(${Name}_map), .data = ${Name}_map,")
    $Writer.WriteLine('};')
    $Writer.WriteLine()
}

# TRUE_COLOR_ALPHA (ARGB8565) white artwork, same format as ui_menu_camera:
# RGB is pure white, alpha carries the shape. No firmware recolor needed.
function Write-LvglArgb8565White {
    param([IO.StreamWriter]$Writer, [string]$Name, [Drawing.Bitmap]$Bitmap)
    $Writer.WriteLine("static const uint8_t ${Name}_map[] = {")
    $column = 0
    for ($y = 0; $y -lt $Bitmap.Height; ++$y) {
        for ($x = 0; $x -lt $Bitmap.Width; ++$x) {
            $pixel = $Bitmap.GetPixel($x, $y)
            $brightness = ([double]$pixel.R + [double]$pixel.G + [double]$pixel.B) / 3.0
            $alpha = [Math]::Min(255, [Math]::Max(0, [Math]::Round($brightness)))
            # Pure white RGB565; the shape is carried entirely by alpha.
            $rgb565 = 0xFFFF
            foreach ($byte in @(($rgb565 -band 0xFF), (($rgb565 -shr 8) -band 0xFF), $alpha)) {
                $Writer.Write(('0x{0:X2},' -f $byte)); ++$column
                if ($column -eq 24) { $Writer.WriteLine(); $column = 0 }
            }
        }
    }
    if ($column -ne 0) { $Writer.WriteLine() }
    $Writer.WriteLine('};')
    $Writer.WriteLine("const lv_img_dsc_t $Name = {")
    $Writer.WriteLine("    .header = {.cf = LV_IMG_CF_TRUE_COLOR_ALPHA, .always_zero = 0, .reserved = 0, .w = $($Bitmap.Width), .h = $($Bitmap.Height)},")
    $Writer.WriteLine("    .data_size = sizeof(${Name}_map), .data = ${Name}_map,")
    $Writer.WriteLine('};')
    $Writer.WriteLine()
}

$references = @()
$images = [ordered]@{}
try {
    for ($index = 1; $index -le 6; ++$index) {
        $path = Join-Path $projectRoot "assets\ui\menu_pages\page$index\reference.png"
        if (-not (Test-Path -LiteralPath $path)) { throw "Missing UI reference: $path" }
        $references += [Drawing.Bitmap]::new($path)
    }
    $images.ui_menu_title = @{ Bitmap=(Copy-Crop $references[0] 52 6 93 24); Color=@(239,239,239) }
    $images.ui_menu_wifi = @{ Bitmap=(Copy-Crop $references[0] 163 6 24 24); Color=@(239,239,239) }
    $images.ui_menu_battery = @{ Bitmap=(Copy-Crop $references[0] 193 10 24 16); Color=@(255,255,255) }
    $images.ui_menu_arrow_left = @{ Bitmap=(Copy-Crop $references[0] 0 116 30 30); Color=@(172,204,229) }
    $images.ui_menu_arrow_right = @{ Bitmap=(Copy-Crop $references[0] 210 116 30 30); Color=@(172,204,229) }
    # Main artwork rects: bottom text band is cropped away so the firmware can
    # render a uniform caption (same font, same position) on every menu page.
    # Heights stop above the text band (page1:205 page2:206 page3:210
    # page4:213 page5:213). Page 6 (camera) is superseded by ui_menu_camera.
    $rectangles = @(@(39,75,161,127), @(27,111,185,92), @(49,60,142,147),
                    @(61,63,128,147), @(61,74,118,136))
    for ($index = 0; $index -lt 5; ++$index) {
        $r = $rectangles[$index]
        $images["ui_menu_page$($index + 1)"] = @{
            Bitmap=(Copy-Crop $references[$index] $r[0] $r[1] $r[2] $r[3]); Color=@(255,255,255)
        }
    }

    $writer = [IO.StreamWriter]::new($temporaryOutput, $false, [Text.UTF8Encoding]::new($false))
    try {
        $writer.WriteLine('// Generated as LVGL artwork from menu Page1..Page6. Do not edit.')
        $writer.WriteLine('#include "lvgl.h"')
        $writer.WriteLine()
        foreach ($entry in $images.GetEnumerator()) {
            if ($entry.Key -like 'ui_menu_page*') {
                Write-LvglArgb8565White $writer $entry.Key $entry.Value.Bitmap
            } else {
                $c = $entry.Value.Color
                Write-LvglAlpha4Image $writer $entry.Key $entry.Value.Bitmap $c[0] $c[1] $c[2]
            }
        }
    } finally { $writer.Dispose() }
} finally {
    foreach ($entry in $images.Values) { if ($entry.Bitmap) { $entry.Bitmap.Dispose() } }
    foreach ($reference in $references) { $reference.Dispose() }
}

if ((Test-Path -LiteralPath $output) -and
    ((Get-FileHash -LiteralPath $output -Algorithm SHA256).Hash -eq
     (Get-FileHash -LiteralPath $temporaryOutput -Algorithm SHA256).Hash)) {
    Remove-Item -LiteralPath $temporaryOutput
    Write-Output "Up to date: $output"
} else {
    Move-Item -LiteralPath $temporaryOutput -Destination $output -Force
    Write-Output "Generated: $output"
}
