# SPDX-License-Identifier: MIT
$ErrorActionPreference = 'Stop'
# 从设置页 reference.png 裁剪 ALPHA_4BIT 资产 -> main/ui_settings_assets.c
$projectRoot = Split-Path -Parent $PSScriptRoot
$referencePath = Join-Path $projectRoot 'assets\ui\settings_page\reference.png'
if (-not (Test-Path -LiteralPath $referencePath)) { throw "Missing UI reference: $referencePath" }
$output = Join-Path $projectRoot 'main\ui_settings_assets.c'
$temporaryOutput = Join-Path ([IO.Path]::GetTempPath()) ("laser-settings-assets-{0}.c" -f [Guid]::NewGuid())

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

$reference = [Drawing.Bitmap]::new($referencePath)
$images = [ordered]@{}
try {
    # Top bar: back arrow (about 4,4 26x28) and title (about 42,4 60x28)
    $images.ui_settings_back = @{ Bitmap=(Copy-Crop $reference 4 4 26 28); Color=@(172,204,229) }
    $images.ui_settings_title = @{ Bitmap=(Copy-Crop $reference 42 4 60 28); Color=@(239,239,239) }
    # Single settings card (224x39), reused at 4 rows by the firmware
    $images.ui_settings_card = @{ Bitmap=(Copy-Crop $reference 8 43 224 39); Color=@(255,255,255) }

    $writer = [IO.StreamWriter]::new($temporaryOutput, $false, [Text.UTF8Encoding]::new($false))
    try {
        $writer.WriteLine('// Generated from assets/ui/settings_page/settings.svg. Do not edit.')
        $writer.WriteLine('#include "lvgl.h"')
        $writer.WriteLine()
        foreach ($entry in $images.GetEnumerator()) {
            $c = $entry.Value.Color
            Write-LvglAlpha4Image $writer $entry.Key $entry.Value.Bitmap $c[0] $c[1] $c[2]
        }
    } finally { $writer.Dispose() }
} finally {
    foreach ($entry in $images.Values) { if ($entry.Bitmap) { $entry.Bitmap.Dispose() } }
    $reference.Dispose()
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
