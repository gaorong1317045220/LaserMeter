# SPDX-License-Identifier: MIT
"""Generate shared LVGL menu constants from all supplied menu layouts."""

from __future__ import annotations

import json
from pathlib import Path
import re


def rgb(value: str) -> int:
    match = re.fullmatch(r"rgb\(\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*\)", value)
    if not match:
        raise ValueError(f"unsupported layout color: {value!r}")
    red, green, blue = (int(part) for part in match.groups())
    return (red << 16) | (green << 8) | blue


def integer(value: object, field: str) -> int:
    number = float(value)
    rounded = round(number)
    if abs(number - rounded) > 0.01:
        raise ValueError(f"{field} must resolve to an integer pixel, got {value!r}")
    return int(rounded)


def read_page(path: Path) -> tuple[dict, dict, list[dict]]:
    document = json.loads(path.read_text(encoding="utf-8"))
    screen = document["screen"]
    if integer(screen["width"], "screen.width") != 240 or integer(screen["height"], "screen.height") != 284:
        raise ValueError(f"unexpected screen size in {path}")
    elements = document["elements"]
    mains = [item for item in elements if item["type"] == "rect" and str(item.get("fill", "")).startswith("url(")]
    dots = sorted((item for item in elements if item["type"] == "ellipse"), key=lambda item: float(item["x"]))
    backgrounds = [item for item in elements if item["type"] == "rect" and item["x"] == 0 and item["y"] == 0]
    if len(mains) != 1 or len(dots) not in (0, 6) or not backgrounds:
        raise ValueError(f"unexpected menu layout structure: {path}")
    return backgrounds[0], mains[0], dots


def values(items: list[dict], field: str) -> str:
    return ", ".join(str(integer(item[field], f"dot.{field}")) for item in items)


def main() -> None:
    root = Path(__file__).resolve().parent.parent
    pages_root = root / "assets" / "ui" / "menu_pages"
    pages = [
        read_page(pages_root / f"page{index}" / f"layout ({index + 1}).json")
        for index in range(1, 7)
    ]
    output_path = root / "main" / "ui_menu_pages_layout.h"

    background = pages[0][0]
    mains = [page[1] for page in pages]
    dots = [page[2] for page in pages]
    if any(rgb(background["fill"]) != rgb(page[0]["fill"]) for page in pages):
        raise ValueError("all menu pages must share the same background")

    lines = [
        "// Generated from assets/ui/menu_pages/page1..page6. Do not edit manually.",
        "#pragma once",
        "",
        "#define UI_MENU_SCREEN_WIDTH 240",
        "#define UI_MENU_SCREEN_HEIGHT 284",
        "#define UI_MENU_PAGE_COUNT 6",
        f"#define UI_MENU_BACKGROUND_COLOR 0x{rgb(background['fill']):06X}u",
        "",
        "// Canvas positions in the supplied full-page SVGs; visible bounds are in layout JSON.",
        "// All six pages share the same header positions so the top bar is aligned.",
        "static const int ui_menu_title_x[UI_MENU_PAGE_COUNT] = {52, 52, 52, 52, 52, 52};",
        "#define UI_MENU_TITLE_Y 6",
        "static const int ui_menu_wifi_x[UI_MENU_PAGE_COUNT] = {163, 163, 163, 163, 163, 163};",
        "#define UI_MENU_WIFI_Y 6",
        "static const int ui_menu_battery_x[UI_MENU_PAGE_COUNT] = {193, 193, 193, 193, 193, 193};",
        "#define UI_MENU_BATTERY_Y 10",
        "#define UI_MENU_ARROW_LEFT_X 0",
        "#define UI_MENU_ARROW_LEFT_Y 116",
        "#define UI_MENU_ARROW_RIGHT_X 210",
        "#define UI_MENU_ARROW_RIGHT_Y 116",
        "",
        "static const int ui_menu_main_x[UI_MENU_PAGE_COUNT] = {" +
        ", ".join(str(integer(item["x"], "main.x")) for item in mains) + "};",
        "static const int ui_menu_main_y[UI_MENU_PAGE_COUNT] = {" +
        ", ".join(str(integer(item["y"], "main.y")) for item in mains) + "};",
        "static const int ui_menu_main_width[UI_MENU_PAGE_COUNT] = {" +
        ", ".join(str(integer(item["width"], "main.width")) for item in mains) + "};",
        "static const int ui_menu_main_height[UI_MENU_PAGE_COUNT] = {" +
        ", ".join(str(integer(item["height"], "main.height")) for item in mains) + "};",
        "",
        "#define UI_MENU_DOT_COUNT 6",
        "static const int ui_menu_dot_visible[UI_MENU_PAGE_COUNT] = {1, 1, 1, 1, 1, 1};",
        "static const int ui_menu_dot_x[UI_MENU_DOT_COUNT] = {" + values(dots[0], "x") + "};",
        "static const int ui_menu_dot_y[UI_MENU_DOT_COUNT] = {" + values(dots[0], "y") + "};",
        "static const int ui_menu_dot_size[UI_MENU_DOT_COUNT] = {" + values(dots[0], "width") + "};",
        "static const unsigned int ui_menu_dot_color[UI_MENU_PAGE_COUNT][UI_MENU_DOT_COUNT] = {",
        *("    {" + ", ".join(f"0x{rgb(item['fill']):06X}u" for item in page_dots) + "},"
          for page_dots in dots),
        "};",
        "",
    ]
    generated = "\n".join(lines)
    previous = output_path.read_text(encoding="utf-8") if output_path.exists() else ""
    if previous != generated:
        output_path.write_text(generated, encoding="utf-8", newline="\n")
        print(f"Generated: {output_path}")
    else:
        print(f"Up to date: {output_path}")


if __name__ == "__main__":
    main()
