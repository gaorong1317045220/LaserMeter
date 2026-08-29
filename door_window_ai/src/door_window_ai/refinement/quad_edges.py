"""ROI 内门/窗边缘几何提取(第一版)。

输入:整幅 BGR 图像 + ROI bbox + 可选语义掩码。
流程:ROI 裁剪 -> 灰度/CLAHE -> 自适应 Canny -> HoughLinesP ->
      线段按角度分类(垂直/水平) -> 按位置聚类成左/右/顶/底四边 ->
      最小二乘拟合边线 -> 四线求交得四角点(LT,RT,RB,LB) ->
      输出支持度/偏差/对称性组成的质量分。

诚实声明:本模块是"视觉区域边缘"提取,不是毫米级结构边界;
轴对齐四边形是第一版模型,透视校正依赖后续相机标定。
"""

# SPDX-License-Identifier: MIT
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Sequence

import cv2
import numpy as np

LtRtRbLb = np.ndarray  # shape (4, 2), LT,RT,RB,LB order

# 角度分类容差(度)
_VERTICAL_TOL_DEG = 30.0
_HORIZONTAL_TOL_DEG = 30.0


@dataclass(slots=True)
class SideLine:
    side: str            # 'left' | 'right' | 'top' | 'bottom'
    kind: str            # 'vertical' | 'horizontal'
    axis_value: float    # 垂直边:x 坐标(整图);水平边:y 坐标(整图)
    support: float       # 该边获得的线段支持比例 0..1
    segment_count: int   # 分配给该边的线段数
    rms_dev_px: float    # 线段中点相对拟合线的均方根偏差(px)
    from_bbox: bool      # True 表示无支持,回退到 bbox 边


@dataclass(slots=True)
class QuadEdges:
    corners_px: LtRtRbLb              # (4,2) LT,RT,RB,LB,整图坐标
    corner_confidence: np.ndarray     # (4,) 每角置信度 0..1
    sides: list[SideLine] = field(default_factory=list)
    supported_sides: int = 0
    quality: float = 0.0
    method: str = "canny+hough_axis_aligned_v1"
    roi_xyxy: tuple[int, int, int, int] = (0, 0, 0, 0)


def _adaptive_canny(gray: np.ndarray) -> np.ndarray:
    med = float(np.median(gray))
    low = max(0.0, 0.5 * med)
    high = min(255.0, 1.33 * med)
    return cv2.Canny(gray, low, high)


def _segment_angle_deg(x1: float, y1: float, x2: float, y2: float) -> float:
    deg = float(np.degrees(np.arctan2(y2 - y1, x2 - x1))) % 180.0
    return deg


def _classify_segments(
    segments: Sequence[tuple[int, int, int, int]],
) -> tuple[list[tuple[int, int, int, int]], list[tuple[int, int, int, int]]]:
    """按角度分成垂直(竖边)与水平(横边)线段。"""
    vertical: list[tuple[int, int, int, int]] = []
    horizontal: list[tuple[int, int, int, int]] = []
    for seg in segments:
        x1, y1, x2, y2 = seg
        angle = _segment_angle_deg(x1, y1, x2, y2)
        if abs(angle - 90.0) <= _VERTICAL_TOL_DEG:
            vertical.append(seg)
        elif angle <= _HORIZONTAL_TOL_DEG or angle >= 180.0 - _HORIZONTAL_TOL_DEG:
            horizontal.append(seg)
    return vertical, horizontal


def _filter_by_mask(
    segments: Sequence[tuple[int, int, int, int]],
    roi_mask: np.ndarray | None,
) -> list[tuple[int, int, int, int]]:
    """保留中点落在(膨胀后的)掩码内的线段,抑制背景边缘。

    若过滤后线段过少(掩码过紧/误标),回退到不过滤的结果。
    """
    if roi_mask is None:
        return list(segments)
    dilated = cv2.dilate(roi_mask.astype(np.uint8), np.ones((7, 7), np.uint8), iterations=1) > 0
    kept: list[tuple[int, int, int, int]] = []
    for x1, y1, x2, y2 in segments:
        mx = int(round((x1 + x2) / 2))
        my = int(round((y1 + y2) / 2))
        if 0 <= mx < dilated.shape[1] and 0 <= my < dilated.shape[0] and dilated[my, mx]:
            kept.append((x1, y1, x2, y2))
    return kept if len(kept) >= 3 else list(segments)


def _two_side_assign(
    values: Sequence[float],
) -> tuple[float, float, int, int, float, float]:
    """把一维值分到低/高两组(2-means,中位数更新,抗离群)。

    返回 (low, high, low_n, high_n, low_std, high_std)。空输入返回全 0。
    """
    if not values:
        return 0.0, 0.0, 0, 0, 0.0, 0.0
    arr = np.asarray(values, dtype=np.float64)
    if arr.size == 1:
        return float(arr[0]), float(arr[0]), 1, 0, 0.0, 0.0
    low, high = float(np.quantile(arr, 0.05)), float(np.quantile(arr, 0.95))
    if high - low < 1e-6:
        return low, high, arr.size, 0, 0.0, 0.0
    for _ in range(8):
        mid = (low + high) / 2
        low_mask = arr <= mid
        high_mask = ~low_mask
        if not low_mask.any() or not high_mask.any():
            break
        new_low = float(np.median(arr[low_mask]))
        new_high = float(np.median(arr[high_mask]))
        if abs(new_low - low) < 1e-3 and abs(new_high - high) < 1e-3:
            low, high = new_low, new_high
            break
        low, high = new_low, new_high
    mid = (low + high) / 2
    low_mask = arr <= mid
    high_mask = ~low_mask
    return (
        low,
        high,
        int(low_mask.sum()),
        int(high_mask.sum()),
        float(arr[low_mask].std()) if low_mask.any() else 0.0,
        float(arr[high_mask].std()) if high_mask.any() else 0.0,
    )


def extract_quad_edges(
    image_bgr: np.ndarray,
    bbox_xyxy: tuple[float, float, float, float],
    mask: np.ndarray | None = None,
    *,
    margin_frac: float = 0.06,
    min_roi_px: int = 40,
) -> QuadEdges:
    """从 ROI 提取轴对齐四边形的四条边与四角点。

    返回的坐标全部映射回整幅图像。掩码(整图 bool)可选,用于过滤背景线段。
    """
    height, width = image_bgr.shape[:2]
    x1, y1, x2, y2 = (int(round(v)) for v in bbox_xyxy)
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(width - 1, x2), min(height - 1, y2)
    if x2 - x1 < 8 or y2 - y1 < 8:
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2
        half = 4
        corners = np.array(
            [[cx - half, cy - half], [cx + half, cy - half],
             [cx + half, cy + half], [cx - half, cy + half]],
            dtype=np.float32,
        )
        return QuadEdges(
            corners_px=corners,
            corner_confidence=np.full(4, 0.1),
            sides=[],
            supported_sides=0,
            quality=0.0,
            roi_xyxy=(x1, y1, x2, y2),
        )

    margin_x = max(0, int(round((x2 - x1) * margin_frac)))
    margin_y = max(0, int(round((y2 - y1) * margin_frac)))
    rx1 = max(0, x1 - margin_x)
    ry1 = max(0, y1 - margin_y)
    rx2 = min(width - 1, x2 + margin_x)
    ry2 = min(height - 1, y2 + margin_y)

    roi = image_bgr[ry1 : ry2 + 1, rx1 : rx2 + 1]
    roi_h, roi_w = roi.shape[:2]
    if min(roi_h, roi_w) < min_roi_px:
        # ROI 太小,直接回退 bbox 角
        corners = np.array(
            [[x1, y1], [x2, y1], [x2, y2], [x1, y2]], dtype=np.float32
        )
        return QuadEdges(
            corners_px=corners,
            corner_confidence=np.full(4, 0.15),
            sides=[],
            supported_sides=0,
            quality=0.0,
            roi_xyxy=(rx1, ry1, rx2, ry2),
        )

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    enhanced = clahe.apply(gray)
    edges = _adaptive_canny(enhanced)

    roi_mask = None
    if mask is not None:
        roi_mask = mask[ry1 : ry2 + 1, rx1 : rx2 + 1]

    min_len = max(16, int(round(min(roi_w, roi_h) * 0.15)))
    lines = cv2.HoughLinesP(
        edges,
        rho=1,
        theta=np.pi / 180.0,
        threshold=40,
        minLineLength=min_len,
        maxLineGap=12,
    )
    segments: list[tuple[int, int, int, int]] = []
    if lines is not None:
        lines = np.asarray(lines)
        # OpenCV 4.x: (N,1,4); OpenCV 5.x: (N,4)
        if lines.ndim == 3:
            lines = lines[:, 0, :]
        for line in lines:
            segments.append((int(line[0]), int(line[1]), int(line[2]), int(line[3])))
    segments = _filter_by_mask(segments, roi_mask)

    vertical, horizontal = _classify_segments(segments)
    total = max(1, len(vertical) + len(horizontal))
    center_x = roi_w / 2.0
    center_y = roi_h / 2.0

    v_axis = [((s[0] + s[2]) / 2.0) for s in vertical]
    h_axis = [((s[1] + s[3]) / 2.0) for s in horizontal]

    left_x, right_x, left_n, right_n, left_std, right_std = _two_side_assign(v_axis)
    top_y, bottom_y, top_n, bottom_n, top_std, bottom_std = _two_side_assign(h_axis)

    def rms_dev(values: Sequence[float], fitted: float) -> float:
        if not values:
            return 0.0
        return float(np.sqrt(np.mean([(v - fitted) ** 2 for v in values])))

    diag = float(max(1.0, np.hypot(roi_w, roi_h)))
    v_total = max(1, len(vertical))
    h_total = max(1, len(horizontal))
    sides: list[SideLine] = [
        SideLine("left", "vertical", rx1 + left_x,
                 min(1.0, left_n / v_total), left_n, rms_dev(v_axis, left_x) / diag,
                 from_bbox=left_n == 0),
        SideLine("right", "vertical", rx1 + right_x,
                 min(1.0, right_n / v_total), right_n, rms_dev(v_axis, right_x) / diag,
                 from_bbox=right_n == 0),
        SideLine("top", "horizontal", ry1 + top_y,
                 min(1.0, top_n / h_total), top_n, rms_dev(h_axis, top_y) / diag,
                 from_bbox=top_n == 0),
        SideLine("bottom", "horizontal", ry1 + bottom_y,
                 min(1.0, bottom_n / h_total), bottom_n, rms_dev(h_axis, bottom_y) / diag,
                 from_bbox=bottom_n == 0),
    ]

    by_name = {side.side: side for side in sides}
    left_line = by_name["left"].axis_value
    right_line = by_name["right"].axis_value
    top_line = by_name["top"].axis_value
    bottom_line = by_name["bottom"].axis_value

    corners = np.array(
        [
            [left_line, top_line],
            [right_line, top_line],
            [right_line, bottom_line],
            [left_line, bottom_line],
        ],
        dtype=np.float32,
    )
    supported = sum(1 for side in sides if not side.from_bbox)

    # 每角置信度 = 相邻两边支持的几何平均(无支持则低分)
    corner_conf = np.zeros(4, dtype=np.float32)
    pairs = [("left", "top"), ("right", "top"), ("right", "bottom"), ("left", "bottom")]
    for index, (first, second) in enumerate(pairs):
        f = by_name[first]
        s = by_name[second]
        corner_conf[index] = float(np.sqrt((f.support if not f.from_bbox else 0.15) *
                                           (s.support if not s.from_bbox else 0.15)))

    support_score = supported / 4.0
    dev_score = 1.0 - min(1.0, sum(side.rms_dev_px for side in sides) / 4.0 * 8.0)
    span_x = right_line - left_line
    span_y = bottom_line - top_line
    if span_x > 0 and span_y > 0:
        sym_x = 1.0 - min(1.0, abs((right_line - rx1 - center_x) - (center_x - (left_line - rx1))) / (span_x + 1e-6))
        sym_y = 1.0 - min(1.0, abs((bottom_line - ry1 - center_y) - (center_y - (top_line - ry1))) / (span_y + 1e-6))
        sym_score = 0.5 * sym_x + 0.5 * sym_y
    else:
        sym_score = 0.0

    quality = float(np.clip(0.5 * support_score + 0.3 * max(0.0, dev_score) + 0.2 * sym_score, 0.0, 1.0))

    return QuadEdges(
        corners_px=corners,
        corner_confidence=corner_conf,
        sides=sides,
        supported_sides=supported,
        quality=quality,
        roi_xyxy=(rx1, ry1, rx2, ry2),
    )


def mask_bbox(mask: np.ndarray | None, fallback: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    """掩码外接框(更贴合目标),无掩码时回退检测框。"""
    if mask is None:
        return fallback
    ys, xs = np.nonzero(mask)
    if xs.size == 0:
        return fallback
    return float(xs.min()), float(ys.min()), float(xs.max()), float(ys.max())


def quad_to_json(quad: QuadEdges) -> dict:
    """序列化为 JSON 安全的 dict。"""
    return {
        "corners_px": [[float(v) for v in corner] for corner in quad.corners_px],
        "corner_confidence": [float(v) for v in quad.corner_confidence],
        "sides": [
            {
                "side": side.side,
                "kind": side.kind,
                "axis_value": side.axis_value,
                "support": round(side.support, 3),
                "segment_count": side.segment_count,
                "rms_dev_px": round(side.rms_dev_px, 3),
                "from_bbox": side.from_bbox,
            }
            for side in quad.sides
        ],
        "supported_sides": quad.supported_sides,
        "quality": round(quad.quality, 3),
        "method": quad.method,
        "roi_xyxy": list(quad.roi_xyxy),
    }
