# SPDX-License-Identifier: MIT
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$sourceRoot = Join-Path $projectRoot 'assets\ui\single_distance'
$generatedRoot = Join-Path $sourceRoot 'generated'
$output = Join-Path $projectRoot 'main\ui_single_assets.c'
$temporaryOutput = Join-Path ([IO.Path]::GetTempPath()) ("laser-single-assets-{0}.c" -f [Guid]::NewGuid())

Add-Type -AssemblyName System.Drawing

function Write-LvglImage {
    param([IO.StreamWriter]$Writer, [string]$Name, [Drawing.Bitmap]$Bitmap)
    $Writer.WriteLine("static const uint8_t ${Name}_map[] = {")
    $column = 0
    for ($y = 0; $y -lt $Bitmap.Height; ++$y) {
        for ($x = 0; $x -lt $Bitmap.Width; ++$x) {
            $pixel = $Bitmap.GetPixel($x, $y)
            $rgb565 = (([int]$pixel.R -shr 3) -shl 11) -bor
                      (([int]$pixel.G -shr 2) -shl 5) -bor ([int]$pixel.B -shr 3)
            foreach ($byte in @(($rgb565 -band 0xFF), (($rgb565 -shr 8) -band 0xFF), [int]$pixel.A)) {
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

$sources = [ordered]@{
    ui_single_title='single_title.png'
    ui_single_ref_front='reference_front.png'
    ui_single_ref_tripod='reference_tripod.png'
    ui_single_zoom_1x='zoom_1x.png'
    ui_single_zoom_2x='zoom_2x.png'
    ui_single_zoom_5x='zoom_5x.png'
    ui_single_laser_label='laser_label.png'
    ui_single_laser_off='laser_off.png'
    ui_single_level_track='level_track.png'
    ui_single_crosshair='crosshair.png'
    ui_single_bottom_history='bottom_history.png'
    ui_single_bottom_measure='bottom_measure.png'
    ui_single_bottom_back='bottom_back.png'
    ui_single_bottom_save='bottom_save.png'
    ui_single_bottom_delete='bottom_delete.png'
    ui_single_bottom_select='bottom_select.png'
    ui_single_bottom_export='bottom_export.png'
}

$bitmaps = [ordered]@{}
try {
    foreach ($entry in $sources.GetEnumerator()) {
        $path = Join-Path $generatedRoot $entry.Value
        if (-not (Test-Path -LiteralPath $path)) { throw "Missing rendered UI asset: $path" }
        $bitmaps[$entry.Key] = [Drawing.Bitmap]::new($path)
    }
    $bitmaps.ui_single_ref_rear = [Drawing.Bitmap]::new((Join-Path $sourceRoot 'reference_rear.png'))
    $bitmaps.ui_single_laser_on = [Drawing.Bitmap]::new((Join-Path $sourceRoot 'laser_on.png'))

    $writer = [IO.StreamWriter]::new($temporaryOutput, $false, [Text.UTF8Encoding]::new($false))
    try {
        $writer.WriteLine('// Generated from supplied single-distance UI artwork. Do not edit.')
        $writer.WriteLine('#include "lvgl.h"')
        $writer.WriteLine()
        foreach ($entry in $bitmaps.GetEnumerator()) { Write-LvglImage $writer $entry.Key $entry.Value }
    } finally { $writer.Dispose() }
} finally {
    foreach ($bitmap in $bitmaps.Values) { $bitmap.Dispose() }
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
