# SPDX-License-Identifier: MIT
"""Update all menu page layout JSONs to 6 pagination dots (one per menu page).

Positions: 54, 78, 102, 126, 150, 174 (12px dots, 24px pitch, centered on 240px).
Colors: current page highlighted (0x3662EC, page1 keeps 0x1677FF), others 0xEFEFEF.
"""
from __future__ import annotations

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PAGES = ROOT / "assets" / "ui" / "menu_pages"

DOT_X = [54, 78, 102, 126, 150, 174]
DOT_Y = 254
DOT_SIZE = 12
HIGHLIGHT = {0: "rgb(22, 119, 255)", 1: "rgb(54, 98, 236)", 2: "rgb(54, 98, 236)",
             3: "rgb(54, 98, 236)", 4: "rgb(54, 98, 236)", 5: "rgb(54, 98, 236)"}
NORMAL = "rgb(239, 239, 239)"


def ellipse(index: int, x: int, fill: str) -> dict:
    return {
        "id": f"ellipse_dot_{index}",
        "source_id": None,
        "type": "ellipse",
        "coordinate_reference": "screen_top_left",
        "x": x,
        "y": DOT_Y,
        "width": DOT_SIZE,
        "height": DOT_SIZE,
        "fill": fill,
        "stroke": "none",
        "opacity": 255,
        "clickable": False,
        "suggested_clickable": False,
        "z_index": 900 + index,
        "depth": 1,
        "parent_id": "g",
    }


def main() -> None:
    for page in range(1, 7):
        path = PAGES / f"page{page}" / f"layout ({page + 1}).json"
        doc = json.loads(path.read_text(encoding="utf-8"))
        elements = [e for e in doc["elements"] if e.get("type") != "ellipse"]
        for i, x in enumerate(DOT_X):
            fill = HIGHLIGHT[page - 1] if i == page - 1 else NORMAL
            elements.append(ellipse(i, x, fill))
        doc["elements"] = elements
        doc["extraction"]["elements"] = len(elements)
        path.write_text(json.dumps(doc, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(f"Updated: {path}")


if __name__ == "__main__":
    main()
