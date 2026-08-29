# SPDX-License-Identifier: MIT
"""Render the boot brand bitmap (240x92) with real Chinese text and emit an
RGB565 C array for the LCD boot screen, plus a PNG preview for the web page."""
from __future__ import annotations

from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parent.parent
W, H = 240, 92
BG = (0x10, 0x1B, 0x29)
WHITE = (255, 255, 255)
SUBTITLE = (0x8F, 0xA6, 0xC8)


def main() -> None:
    img = Image.new("RGB", (W, H), BG)
    draw = ImageDraw.Draw(img)
    font_path = r"C:\Windows\Fonts\msyhbd.ttc"
    title_font = ImageFont.truetype(font_path, 26)
    sub_font = ImageFont.truetype(r"C:\Windows\Fonts\msyh.ttc", 13)

    title = "激光测距仪"
    tw = draw.textlength(title, font=title_font)
    draw.text(((W - tw) / 2, 20), title, font=title_font, fill=WHITE)

    subtitle = "智能量房 · 便携激光"
    sw = draw.textlength(subtitle, font=sub_font)
    draw.text(((W - sw) / 2, 60), subtitle, font=sub_font, fill=SUBTITLE)

    (ROOT / "docs" / "boot_brand.png").parent.mkdir(parents=True, exist_ok=True)
    img.save(ROOT / "docs" / "boot_brand.png")

    out = ROOT / "main" / "nv3030b_boot_brand.c"
    lines = ["// Generated boot brand bitmap (240x92 RGB565). Do not edit.",
             '#include "lvgl.h"', "",
             "static const uint8_t nv3030b_boot_brand_map[] = {"]
    row = []
    col = 0
    for y in range(H):
        for x in range(W):
            r, g, b = img.getpixel((x, y))
            v = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)
            for byte in (v & 0xFF, (v >> 8) & 0xFF):
                row.append(f"0x{byte:02X},")
                col += 1
                if col == 24:
                    lines.append("".join(row))
                    row = []
                    col = 0
    if row:
        lines.append("".join(row))
    lines += ["};",
              "const lv_img_dsc_t nv3030b_boot_brand = {",
              f"    .header = {{.cf = LV_IMG_CF_TRUE_COLOR, .always_zero = 0, .reserved = 0, .w = {W}, .h = {H}}},",
              "    .data_size = sizeof(nv3030b_boot_brand_map), .data = nv3030b_boot_brand_map,",
              "};", ""]
    out.write_text("\n".join(lines), encoding="utf-8")
    print(f"Generated: {out} ({out.stat().st_size} bytes)")


if __name__ == "__main__":
    main()
