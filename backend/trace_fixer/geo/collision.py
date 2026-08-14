"""Oriented bounding box (OBB) overlap test via the separating axis theorem."""
from __future__ import annotations

import math


def _corners(x: float, y: float, heading_deg: float, length: float, width: float) -> list[tuple[float, float]]:
    h = math.radians(heading_deg)
    cos_h, sin_h = math.cos(h), math.sin(h)
    hl, hw = length / 2.0, width / 2.0
    local = [(hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw)]
    return [(x + cos_h * lx - sin_h * ly, y + sin_h * lx + cos_h * ly) for lx, ly in local]


def _axes(corners: list[tuple[float, float]]) -> list[tuple[float, float]]:
    axes = []
    for i in range(2):
        x1, y1 = corners[i]
        x2, y2 = corners[i + 1]
        edge = (x2 - x1, y2 - y1)
        length = math.hypot(*edge)
        if length > 1e-9:
            axes.append((-edge[1] / length, edge[0] / length))
    return axes


def _project(corners: list[tuple[float, float]], axis: tuple[float, float]) -> tuple[float, float]:
    dots = [c[0] * axis[0] + c[1] * axis[1] for c in corners]
    return min(dots), max(dots)


def obb_overlap(
    x1: float, y1: float, heading1_deg: float, length1: float, width1: float,
    x2: float, y2: float, heading2_deg: float, length2: float, width2: float,
) -> bool:
    """True if the two oriented rectangles intersect (SAT test)."""
    c1 = _corners(x1, y1, heading1_deg, length1, width1)
    c2 = _corners(x2, y2, heading2_deg, length2, width2)
    for axis in _axes(c1) + _axes(c2):
        min1, max1 = _project(c1, axis)
        min2, max2 = _project(c2, axis)
        if max1 < min2 or max2 < min1:
            return False
    return True


def obb_overlap_area_fraction(
    x1: float, y1: float, heading1_deg: float, length1: float, width1: float,
    x2: float, y2: float, heading2_deg: float, length2: float, width2: float,
) -> float:
    """Cheap overlap-severity proxy: 1.0 if centers coincide, 0.0 at first separating gap.
    Not a true polygon-intersection area -- used only to rank/severity-classify collisions.
    """
    dist = math.hypot(x2 - x1, y2 - y1)
    combined_half_diag = (math.hypot(length1, width1) + math.hypot(length2, width2)) / 4.0
    if combined_half_diag <= 0:
        return 0.0
    return max(0.0, min(1.0, 1.0 - dist / combined_half_diag))
