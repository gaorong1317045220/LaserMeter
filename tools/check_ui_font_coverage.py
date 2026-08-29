# SPDX-License-Identifier: MIT
"""Fail when an LVGL UI string uses a glyph absent from ui_font_16.c."""

from __future__ import annotations

import ast
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
UI_SOURCES = [ROOT / "main" / "device_ui_lvgl.cpp", ROOT / "main" / "ui_menu_pages.c"]
FONT_SOURCE = ROOT / "main" / "ui_font_16.c"


def ui_codepoints(source: str) -> set[int]:
    result: set[int] = set()
    for match in re.finditer(r'"(?:\\.|[^"\\])*"', source):
        try:
            value = ast.literal_eval(match.group(0))
        except (SyntaxError, ValueError):
            continue
        result.update(ord(char) for char in value if ord(char) > 0x7F)
    return result


def font_codepoints(source: str) -> set[int]:
    # 从每个字形注释 /* U+XXXX "c" */ 提取,兼容新旧 lv_font_conv 头部格式
    result: set[int] = set()
    for match in re.finditer(r"U\+([0-9A-Fa-f]{4,5})", source):
        result.add(int(match.group(1), 16))
    if not result:
        raise RuntimeError("ui_font_16.c contains no glyph codepoints")
    return result


def main() -> int:
    required: set[int] = set()
    for source in UI_SOURCES:
        required |= ui_codepoints(source.read_text(encoding="utf-8"))
    available = font_codepoints(FONT_SOURCE.read_text(encoding="utf-8"))
    missing = sorted(required - available)
    if missing:
        chars = "".join(chr(codepoint) for codepoint in missing)
        values = ", ".join(f"U+{codepoint:04X}" for codepoint in missing)
        print(f"FAIL: {len(missing)} UI glyphs are missing: {chars} ({values})")
        return 1
    print(f"PASS: all {len(required)} non-ASCII UI glyphs are present")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
