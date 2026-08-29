# SPDX-License-Identifier: MIT
"""Deterministic wall extraction for an ordered fixed-station laser scan.

The input points must be in acquisition/azimuth order.  The implementation is
deliberately dependency-free so the PC receiver can run without NumPy/OpenCV.
It keeps raw measurements separate from derived geometry: median filtering,
RANSAC wall hypotheses, corner intersections and short recess candidates are
all returned with provenance and quality statistics.
"""

from __future__ import annotations

import math
import random
import statistics
from typing import Iterable


def _distance(a: dict, b: dict) -> float:
    return math.hypot(float(a["x_mm"]) - float(b["x_mm"]),
                      float(a["y_mm"]) - float(b["y_mm"]))


def _line_from_points(a: dict, b: dict) -> tuple[float, float, float] | None:
    dx = float(b["x_mm"]) - float(a["x_mm"])
    dy = float(b["y_mm"]) - float(a["y_mm"])
    length = math.hypot(dx, dy)
    if length < 1.0:
        return None
    # Unit normal and signed offset: nx*x + ny*y + c = 0.
    nx, ny = -dy / length, dx / length
    return nx, ny, -(nx * float(a["x_mm"]) + ny * float(a["y_mm"]))


def _line_distance(point: dict, line: tuple[float, float, float]) -> float:
    return abs(line[0] * float(point["x_mm"]) +
               line[1] * float(point["y_mm"]) + line[2])


def _fit_tls(points: list[dict]) -> tuple[float, float, float, float, float] | None:
    """Total-least-squares line as centre + unit tangent."""
    if len(points) < 2:
        return None
    cx = statistics.fmean(float(p["x_mm"]) for p in points)
    cy = statistics.fmean(float(p["y_mm"]) for p in points)
    sxx = statistics.fmean((float(p["x_mm"]) - cx) ** 2 for p in points)
    syy = statistics.fmean((float(p["y_mm"]) - cy) ** 2 for p in points)
    sxy = statistics.fmean((float(p["x_mm"]) - cx) *
                           (float(p["y_mm"]) - cy) for p in points)
    angle = 0.5 * math.atan2(2.0 * sxy, sxx - syy)
    tx, ty = math.cos(angle), math.sin(angle)
    nx, ny = -ty, tx
    c = -(nx * cx + ny * cy)
    return cx, cy, tx, ty, c


def median_filter_points(points: list[dict], *, radius: int = 2,
                         absolute_threshold_mm: float = 180.0,
                         relative_threshold: float = 0.10) -> tuple[list[dict], list[dict]]:
    """Remove only isolated radial spikes, preserving sustained recesses.

    A point is removed only when both immediate neighbours agree with the
    neighbourhood median while the centre point disagrees with all of them.
    Corners and multi-sample door/furniture recesses therefore survive for the
    wall/occlusion stage instead of being silently erased.
    """
    if len(points) < 5:
        return list(points), []
    kept: list[dict] = []
    removed: list[dict] = []
    for index, point in enumerate(points):
        if index == 0 or index == len(points) - 1:
            kept.append(point)
            continue
        lo, hi = max(0, index - radius), min(len(points), index + radius + 1)
        local = [float(p["distance_mm"]) for p in points[lo:hi]]
        median = statistics.median(local)
        threshold = max(absolute_threshold_mm, abs(median) * relative_threshold)
        value = float(point["distance_mm"])
        previous = float(points[index - 1]["distance_mm"])
        following = float(points[index + 1]["distance_mm"])
        neighbours_agree = abs(previous - following) <= threshold
        isolated = (abs(value - median) > threshold and neighbours_agree and
                    abs(value - previous) > threshold and abs(value - following) > threshold)
        if isolated:
            removed.append({**point, "filter_reason": "isolated_radial_spike",
                            "median_distance_mm": round(median, 2),
                            "deviation_mm": round(value - median, 2)})
        else:
            kept.append(point)
    return kept, removed


def _groups(indices: list[int], points: list[dict], max_index_gap: int,
            max_angle_gap_deg: float) -> list[list[int]]:
    if not indices:
        return []
    result = [[indices[0]]]
    for index in indices[1:]:
        previous = result[-1][-1]
        angle_gap = abs(float(points[index]["angle_unwrapped_deg"]) -
                        float(points[previous]["angle_unwrapped_deg"]))
        if index - previous <= max_index_gap and angle_gap <= max_angle_gap_deg:
            result[-1].append(index)
        else:
            result.append([index])
    return result


def _wall_from_indices(points: list[dict], indices: list[int], wall_id: int,
                       threshold_mm: float) -> dict | None:
    fit_points = [points[i] for i in indices]
    fit = _fit_tls(fit_points)
    if fit is None:
        return None
    cx, cy, tx, ty, _ = fit
    # Preserve acquisition direction so adjacent-wall intersections map to
    # start/end consistently even when the TLS tangent points the other way.
    first_projection = ((float(fit_points[0]["x_mm"]) - cx) * tx +
                        (float(fit_points[0]["y_mm"]) - cy) * ty)
    last_projection = ((float(fit_points[-1]["x_mm"]) - cx) * tx +
                       (float(fit_points[-1]["y_mm"]) - cy) * ty)
    if last_projection < first_projection:
        tx, ty = -tx, -ty
    projections = [((float(p["x_mm"]) - cx) * tx +
                    (float(p["y_mm"]) - cy) * ty) for p in fit_points]
    start_t, end_t = min(projections), max(projections)
    ax, ay = cx + tx * start_t, cy + ty * start_t
    bx, by = cx + tx * end_t, cy + ty * end_t
    if ((ax - float(fit_points[0]["x_mm"])) ** 2 +
            (ay - float(fit_points[0]["y_mm"])) ** 2) > (
            (bx - float(fit_points[0]["x_mm"])) ** 2 +
            (by - float(fit_points[0]["y_mm"])) ** 2):
        ax, ay, bx, by = bx, by, ax, ay
        tx, ty = -tx, -ty
    nx, ny = -ty, tx
    c = -(nx * cx + ny * cy)
    residuals = [_line_distance(p, (nx, ny, c)) for p in fit_points]
    length = math.hypot(bx - ax, by - ay)
    if length < 1.0:
        return None
    return {
        "id": f"fit_wall_{wall_id}",
        "ax_mm": round(ax, 2), "ay_mm": round(ay, 2),
        "bx_mm": round(bx, 2), "by_mm": round(by, 2),
        "length_mm": round(length, 2),
        "angle_deg": round(math.degrees(math.atan2(by - ay, bx - ax)), 3),
        "inlier_count": len(indices),
        "rms_error_mm": round(math.sqrt(statistics.fmean(r * r for r in residuals)), 2),
        "max_error_mm": round(max(residuals), 2),
        "start_sequence": int(points[indices[0]]["sequence"]),
        "end_sequence": int(points[indices[-1]]["sequence"]),
        "source_index_start": indices[0], "source_index_end": indices[-1],
        "inlier_indices": indices,
        "threshold_mm": threshold_mm,
    }


def _intersection(first: dict, second: dict) -> tuple[float, float] | None:
    x1, y1, x2, y2 = first["ax_mm"], first["ay_mm"], first["bx_mm"], first["by_mm"]
    x3, y3, x4, y4 = second["ax_mm"], second["ay_mm"], second["bx_mm"], second["by_mm"]
    denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(denominator) < 1.0e-6:
        return None
    determinant1 = x1 * y2 - y1 * x2
    determinant2 = x3 * y4 - y3 * x4
    x = (determinant1 * (x3 - x4) - (x1 - x2) * determinant2) / denominator
    y = (determinant1 * (y3 - y4) - (y1 - y2) * determinant2) / denominator
    return x, y


def extract_walls(points: list[dict], *, ransac_threshold_mm: float = 90.0,
                  min_inliers: int = 8, min_wall_length_mm: float = 500.0,
                  iterations: int = 900, bridge_gap_points: int = 18,
                  bridge_angle_deg: float = 14.0,
                  max_corner_extension_mm: float = 900.0,
                  max_occlusion_width_mm: float = 1200.0,
                  seed: int = 42) -> dict:
    """Extract ordered wall lines with deterministic contiguous RANSAC."""
    if len(points) < min_inliers:
        return {"walls": [], "corners": [], "occlusions": [], "unmodelled_points": points}
    available = set(range(len(points)))
    walls: list[dict] = []
    occlusions: list[dict] = []
    rng = random.Random(seed)

    while len(available) >= min_inliers and len(walls) < 32:
        candidates = sorted(available)
        best: tuple[float, list[int]] | None = None
        for _ in range(iterations):
            left, right = rng.sample(candidates, 2)
            if abs(left - right) < 4 or _distance(points[left], points[right]) < min_wall_length_mm:
                continue
            line = _line_from_points(points[left], points[right])
            if line is None:
                continue
            inliers = [i for i in candidates if _line_distance(points[i], line) <= ransac_threshold_mm]
            for group in _groups(inliers, points, bridge_gap_points, bridge_angle_deg):
                if len(group) < min_inliers:
                    continue
                length = _distance(points[group[0]], points[group[-1]])
                if length < min_wall_length_mm:
                    continue
                span = group[-1] - group[0] + 1
                gap_penalty = max(0, span - len(group)) * 0.35
                score = len(group) + length / 350.0 - gap_penalty
                if best is None or score > best[0]:
                    best = score, group
        if best is None:
            break

        preliminary = _wall_from_indices(points, best[1], len(walls) + 1, ransac_threshold_mm)
        if preliminary is None or preliminary["length_mm"] < min_wall_length_mm:
            break
        refined_line = _line_from_points(
            {"x_mm": preliminary["ax_mm"], "y_mm": preliminary["ay_mm"]},
            {"x_mm": preliminary["bx_mm"], "y_mm": preliminary["by_mm"]})
        span_start, span_end = best[1][0], best[1][-1]
        refined_indices = [i for i in range(span_start, span_end + 1) if i in available and
                           refined_line is not None and
                           _line_distance(points[i], refined_line) <= ransac_threshold_mm]
        wall = _wall_from_indices(points, refined_indices, len(walls) + 1, ransac_threshold_mm)
        if wall is None or wall["length_mm"] < min_wall_length_mm:
            break

        inlier_set = set(refined_indices)
        rejected = [i for i in range(span_start, span_end + 1) if i in available and i not in inlier_set]
        for group in _groups(rejected, points, 2, 4.0):
            if not group:
                continue
            width = _distance(points[group[0]], points[group[-1]]) if len(group) > 1 else 0.0
            depths = [_line_distance(points[i], refined_line) for i in group] if refined_line else []
            if (len(group) >= 2 and width >= 100.0 and depths and
                    max(depths) >= ransac_threshold_mm and width <= max_occlusion_width_mm):
                occlusions.append({
                    "id": f"occlusion_{len(occlusions) + 1}",
                    "wall_id": wall["id"],
                    "start_sequence": int(points[group[0]]["sequence"]),
                    "end_sequence": int(points[group[-1]]["sequence"]),
                    "width_mm": round(width, 2),
                    "max_depth_mm": round(max(depths), 2),
                    "point_count": len(group),
                    "classification": "short_recess_or_occlusion",
                })

        walls.append(wall)
        # Consume the complete angular span.  Rejected short recess samples are
        # already preserved as occlusions and must not become false inner walls.
        for index in range(span_start, span_end + 1):
            available.discard(index)

    walls.sort(key=lambda wall: wall["source_index_start"])
    # A wall crossed by the 0/360-degree ray is naturally extracted as the
    # first and last segment.  Merge those collinear pieces before computing
    # cyclic corners, otherwise the editable plan contains a false seam.
    if len(walls) >= 2:
        first, last = walls[0], walls[-1]
        first_angle = math.atan2(first["by_mm"] - first["ay_mm"],
                                 first["bx_mm"] - first["ax_mm"])
        last_angle = math.atan2(last["by_mm"] - last["ay_mm"],
                                last["bx_mm"] - last["ax_mm"])
        parallel_error = abs((math.degrees(first_angle - last_angle) + 90.0) % 180.0 - 90.0)
        first_line = _line_from_points(
            {"x_mm": first["ax_mm"], "y_mm": first["ay_mm"]},
            {"x_mm": first["bx_mm"], "y_mm": first["by_mm"]})
        collinear_error = max(_line_distance({"x_mm": last["ax_mm"], "y_mm": last["ay_mm"]}, first_line),
                              _line_distance({"x_mm": last["bx_mm"], "y_mm": last["by_mm"]}, first_line)) \
            if first_line else math.inf
        if parallel_error <= 8.0 and collinear_error <= ransac_threshold_mm * 2.0:
            ordered_indices = last["inlier_indices"] + first["inlier_indices"]
            ordered_points = [points[i] for i in ordered_indices]
            merged = _wall_from_indices(ordered_points, list(range(len(ordered_points))),
                                        len(walls) + 1, ransac_threshold_mm)
            if merged is not None:
                old_ids = {first["id"], last["id"]}
                merged["id"] = "fit_wall_wrap"
                merged["start_sequence"] = last["start_sequence"]
                merged["end_sequence"] = first["end_sequence"]
                merged["source_index_start"] = last["source_index_start"]
                merged["source_index_end"] = first["source_index_end"]
                merged["inlier_indices"] = ordered_indices
                for item in occlusions:
                    if item["wall_id"] in old_ids:
                        item["wall_id"] = merged["id"]
                walls = walls[1:-1] + [merged]
    corners: list[dict] = []
    if len(walls) >= 2:
        for index, wall in enumerate(walls):
            following = walls[(index + 1) % len(walls)]
            hit = _intersection(wall, following)
            if hit is None:
                continue
            x, y = hit
            extension = max(math.hypot(x - wall["bx_mm"], y - wall["by_mm"]),
                            math.hypot(x - following["ax_mm"], y - following["ay_mm"]))
            first_angle = math.atan2(wall["by_mm"] - wall["ay_mm"],
                                     wall["bx_mm"] - wall["ax_mm"])
            second_angle = math.atan2(following["by_mm"] - following["ay_mm"],
                                      following["bx_mm"] - following["ax_mm"])
            turn = abs((math.degrees(second_angle - first_angle) + 180.0) % 360.0 - 180.0)
            if extension > max_corner_extension_mm or turn < 12.0 or turn > 168.0:
                continue
            corner = {"id": f"corner_{len(corners) + 1}", "x_mm": round(x, 2),
                      "y_mm": round(y, 2), "wall_before": wall["id"],
                      "wall_after": following["id"], "turn_deg": round(turn, 2),
                      "extension_mm": round(extension, 2)}
            corners.append(corner)
            wall["bx_mm"], wall["by_mm"] = corner["x_mm"], corner["y_mm"]
            following["ax_mm"], following["ay_mm"] = corner["x_mm"], corner["y_mm"]

    for wall in walls:
        wall["length_mm"] = round(math.hypot(wall["bx_mm"] - wall["ax_mm"],
                                             wall["by_mm"] - wall["ay_mm"]), 2)
        # Inlier indices are an internal implementation detail and make the
        # HTTP response unnecessarily large.
        wall.pop("inlier_indices", None)

    unmodelled = [points[i] for i in sorted(available)]
    return {"walls": walls, "corners": corners, "occlusions": occlusions,
            "unmodelled_points": unmodelled}


def process_scan_geometry(points: Iterable[dict]) -> dict:
    raw = list(points)
    filtered, removed = median_filter_points(raw)
    model = extract_walls(filtered)
    walls = model["walls"]
    corners = model["corners"]
    rms_values = [float(wall["rms_error_mm"]) for wall in walls]
    model_ready = len(walls) >= 3 and len(corners) >= 2
    return {
        "parameters": {
            "median_radius": 2,
            "median_absolute_threshold_mm": 180.0,
            "median_relative_threshold": 0.10,
            "ransac_distance_threshold_mm": 90.0,
            "ransac_iterations_per_wall": 900,
            "minimum_wall_inliers": 8,
            "minimum_wall_length_mm": 500.0,
            "maximum_occlusion_width_mm": 1200.0,
            "maximum_corner_extension_mm": 900.0,
            "random_seed": 42,
        },
        "summary": {
            "raw_count": len(raw), "filtered_count": len(filtered),
            "removed_outlier_count": len(removed), "wall_count": len(walls),
            "corner_count": len(corners), "occlusion_count": len(model["occlusions"]),
            "unmodelled_count": len(model["unmodelled_points"]),
            "mean_wall_rms_error_mm": round(statistics.fmean(rms_values), 2) if rms_values else None,
            "model_ready": model_ready,
        },
        "filtered_points": filtered,
        "removed_outliers": removed,
        **model,
    }
