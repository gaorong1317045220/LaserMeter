# SPDX-License-Identifier: MIT
"""Parse a completed room scan into a reviewable PC-side point-cloud preview."""

from __future__ import annotations

import csv
import math
import statistics
from pathlib import Path

try:
    from pc_app.wall_extraction import process_scan_geometry
except ModuleNotFoundError:  # Direct execution: python pc_app/scan_analysis.py
    from wall_extraction import process_scan_geometry


class ScanFormatError(ValueError):
    """Raised when an uploaded file is not a supported room scan."""


def _number(row: dict[str, str], key: str) -> float:
    try:
        value = float(row[key])
    except (KeyError, TypeError, ValueError) as exc:
        raise ScanFormatError(f"invalid {key}") from exc
    if not math.isfinite(value):
        raise ScanFormatError(f"non-finite {key}")
    return value


def analyze_scan(path: Path) -> dict:
    metadata: dict[str, str] = {}
    csv_lines: list[str] = []
    try:
        with path.open("r", encoding="utf-8-sig", newline="") as source:
            for line in source:
                if line.startswith("#"):
                    item = line[1:].strip()
                    if "=" in item:
                        key, value = item.split("=", 1)
                        metadata[key.strip()] = value.strip()
                elif line.strip():
                    csv_lines.append(line)
    except (OSError, UnicodeError) as exc:
        raise ScanFormatError("cannot read scan file") from exc

    schema = metadata.get("schema")
    if schema not in {"laser_room_scan_v1", "laser_room_scan_v2"}:
        raise ScanFormatError("unsupported or missing scan schema")
    reader = csv.DictReader(csv_lines)
    required = {"timestamp_us", "angle_deg", "distance_mm", "valid", "roll_deg",
                "pitch_deg", "quality", "sync_error_ms", "discontinuity"}
    if not reader.fieldnames or not required.issubset(reader.fieldnames):
        raise ScanFormatError("scan CSV columns are incomplete")
    projected_columns = {"x_mm", "y_mm", "z_mm", "horizontal_distance_mm", "motion_risk"}
    has_projected_coordinates = projected_columns.issubset(reader.fieldnames)
    if schema == "laser_room_scan_v2" and not has_projected_coordinates:
        raise ScanFormatError("v2 scan is missing quaternion-projected coordinates")

    points: list[dict] = []
    angles: list[float] = []
    timestamps_us: list[int] = []
    invalid_count = 0
    for sequence, row in enumerate(reader):
        try:
            angle = _number(row, "angle_deg")
            timestamp_us = int(row["timestamp_us"])
            distance = _number(row, "distance_mm")
            valid = int(row["valid"]) == 1
            roll = _number(row, "roll_deg")
            pitch = _number(row, "pitch_deg")
            quality = int(row["quality"])
            sync_error = int(row["sync_error_ms"])
            discontinuity = int(row["discontinuity"]) != 0
            if has_projected_coordinates:
                x_mm = _number(row, "x_mm")
                y_mm = _number(row, "y_mm")
                z_mm = _number(row, "z_mm")
                horizontal_mm = _number(row, "horizontal_distance_mm")
                motion_risk = int(row["motion_risk"])
            else:
                radians = math.radians(angle)
                x_mm = math.cos(radians) * distance
                y_mm = -math.sin(radians) * distance
                z_mm = 0.0
                horizontal_mm = distance
                motion_risk = 0
        except (ValueError, TypeError, KeyError) as exc:
            raise ScanFormatError(f"invalid row {sequence + 2}") from exc
        angles.append(angle)
        timestamps_us.append(timestamp_us)
        if not valid:
            invalid_count += 1
            continue
        points.append({
            "sequence": sequence,
            "angle_unwrapped_deg": round(angle, 4),
            "angle_deg": round(angle % 360.0, 4),
            "distance_mm": round(distance),
            "x_mm": round(x_mm, 2),
            "y_mm": round(y_mm, 2),
            "z_mm": round(z_mm, 2),
            "horizontal_distance_mm": round(horizontal_mm, 2),
            "roll_deg": round(roll, 4),
            "pitch_deg": round(pitch, 4),
            "quality": quality,
            "sync_error_ms": sync_error,
            "discontinuity": discontinuity,
            "motion_risk": motion_risk,
        })

    total_count = len(points) + invalid_count
    if total_count == 0:
        raise ScanFormatError("scan contains no samples")
    coverage = max(angles) - min(angles) if angles else 0.0
    distances = [point["distance_mm"] for point in points]
    median_distance = statistics.median(distances) if distances else 0.0
    display_limit = max(10000.0, median_distance * 6.0)
    for point in points:
        point["display_outlier"] = point["distance_mm"] > display_limit
    invalid_ratio = invalid_count / total_count
    discontinuity_ratio = (sum(point["discontinuity"] for point in points) / len(points)
                           if points else 1.0)
    intervals_us = [b - a for a, b in zip(timestamps_us, timestamps_us[1:]) if b > a]
    median_interval_us = statistics.median(intervals_us) if intervals_us else 0.0
    effective_rate_hz = 1_000_000.0 / median_interval_us if median_interval_us > 0 else 0.0

    normalized = sorted({round(point["angle_deg"], 3) for point in points})
    if len(normalized) >= 2:
        gaps = [b - a for a, b in zip(normalized, normalized[1:])]
        gaps.append(360.0 - normalized[-1] + normalized[0])
        maximum_gap = max(gaps)
    else:
        maximum_gap = 360.0

    warnings: list[str] = []
    if len(points) < 30:
        warnings.append("有效点少于30个")
    if invalid_ratio > 0.35:
        warnings.append(f"无效采样比例过高（{invalid_ratio:.0%}）")
    if coverage > 450.0:
        warnings.append(f"扫描超过一周（{coverage:.1f}°），存在重复方向数据")
    if maximum_gap > 25.0:
        warnings.append(f"最大角度缺口过大（{maximum_gap:.1f}°）")
    if discontinuity_ratio > 0.25:
        warnings.append(f"距离突跳比例较高（{discontinuity_ratio:.0%}）")
    if distances and (min(distances) <= 300 or max(distances) >= 15000):
        warnings.append("存在接近量程边界的距离值")

    # This is only a visual guide.  Multiple revolutions may overlap and sparse
    # directions must not be silently promoted to editable wall geometry.
    preview_outline = sorted((point for point in points if not point["display_outlier"]),
                             key=lambda item: (item["angle_deg"], item["sequence"]))
    outline_ready = bool(points) and not warnings and 330.0 <= coverage <= 450.0
    geometry = process_scan_geometry(preview_outline)
    if not geometry["summary"]["model_ready"]:
        warnings.append("自动墙线不足，需保留原始点云并人工补画")
        outline_ready = False
    return {
        "schema": schema,
        "session_id": metadata.get("session_id", ""),
        "source_file": path.name,
        "points": points,
        "preview_outline": preview_outline,
        "processed_geometry": geometry,
        "summary": {
            "total_count": total_count,
            "valid_count": len(points),
            "invalid_count": invalid_count,
            "invalid_ratio": round(invalid_ratio, 4),
            "coverage_deg": round(coverage, 3),
            "revolutions": round(coverage / 360.0, 3),
            "maximum_angular_gap_deg": round(maximum_gap, 3),
            "discontinuity_ratio": round(discontinuity_ratio, 4),
            "distance_min_mm": min(distances) if distances else None,
            "distance_median_mm": round(median_distance, 1) if distances else None,
            "distance_max_mm": max(distances) if distances else None,
            "effective_saved_sample_rate_hz": round(effective_rate_hz, 2) if effective_rate_hz else None,
        },
        "quality": {
            "outline_ready": outline_ready,
            "preview_only": not outline_ready,
            "fixed_station_projection": has_projected_coordinates,
            "warnings": warnings,
        },
    }
