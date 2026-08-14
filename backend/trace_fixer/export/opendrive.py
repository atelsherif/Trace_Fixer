"""Minimal OpenDRIVE (.xodr) generator.

Scope (deliberately limited -- see project README): the road's reference
line is a piecewise-linear approximation of the (fixed) ego path, and lanes
are a constant-width, constant-count cross-section derived from the
annotation's NUM_lanes field. This is meant to be "good enough to run our
own in-GUI simulation and sanity-check trajectories against lane
boundaries", not a replacement for the HERE-derived map Applied Intuition
already builds. Notably: the reference line runs along the *edge* of the
ego's lane rather than exactly through its center (a half-lane-width
simplification), since we don't attempt curvature-continuous geometry or
precise lane-center alignment here.
"""
from __future__ import annotations

import math
from xml.etree.ElementTree import Element, SubElement, tostring
from xml.dom import minidom

from trace_fixer.models import Trace

DEFAULT_LANE_WIDTH_M = 3.5
MIN_SEGMENT_LEN_M = 5.0


def _downsample_by_arclength(points: list[tuple[float, float]], min_len: float) -> list[tuple[float, float]]:
    if len(points) <= 2:
        return points
    out = [points[0]]
    last = points[0]
    for p in points[1:-1]:
        if math.hypot(p[0] - last[0], p[1] - last[1]) >= min_len:
            out.append(p)
            last = p
    out.append(points[-1])
    return out


def _majority_num_lanes(trace: Trace, default: int = 2) -> int:
    counts: dict[int, int] = {}
    for m in trace.annotation.frame_meta:
        if m.num_lanes:
            counts[m.num_lanes] = counts.get(m.num_lanes, 0) + 1
    if not counts:
        return default
    return max(counts, key=lambda k: counts[k])


def generate_opendrive(trace: Trace, road_name: str = "trace_fixer_road") -> str:
    points = [(p.x_m, p.y_m) for p in trace.ego.poses]
    points = _downsample_by_arclength(points, MIN_SEGMENT_LEN_M)
    if len(points) < 2:
        raise ValueError("Ego path too short to build a road")

    num_lanes = _majority_num_lanes(trace)

    odr = Element(
        "OpenDRIVE",
    )
    header = SubElement(odr, "header", {
        "revMajor": "1", "revMinor": "6", "name": road_name, "version": "1.00",
        "north": "0", "south": "0", "east": "0", "west": "0",
    })

    total_len = sum(
        math.hypot(points[i + 1][0] - points[i][0], points[i + 1][1] - points[i][1])
        for i in range(len(points) - 1)
    )

    road = SubElement(odr, "road", {
        "name": road_name, "length": f"{total_len:.3f}", "id": "1", "junction": "-1",
    })
    plan_view = SubElement(road, "planView")

    s_cursor = 0.0
    for i in range(len(points) - 1):
        x0, y0 = points[i]
        x1, y1 = points[i + 1]
        seg_len = math.hypot(x1 - x0, y1 - y0)
        if seg_len <= 1e-6:
            continue
        hdg = math.atan2(y1 - y0, x1 - x0)
        geometry = SubElement(plan_view, "geometry", {
            "s": f"{s_cursor:.3f}", "x": f"{x0:.3f}", "y": f"{y0:.3f}",
            "hdg": f"{hdg:.6f}", "length": f"{seg_len:.3f}",
        })
        SubElement(geometry, "line")
        s_cursor += seg_len

    lanes = SubElement(road, "lanes")
    SubElement(lanes, "laneOffset", {"s": "0", "a": "0", "b": "0", "c": "0", "d": "0"})
    lane_section = SubElement(lanes, "laneSection", {"s": "0"})
    center = SubElement(lane_section, "center")
    center_lane = SubElement(center, "lane", {"id": "0", "type": "none", "level": "false"})
    SubElement(center_lane, "roadMark", {"sOffset": "0", "type": "solid", "weight": "standard", "color": "standard", "width": "0.12"})

    right = SubElement(lane_section, "right")
    for i in range(1, num_lanes + 1):
        lane = SubElement(right, "lane", {"id": str(-i), "type": "driving", "level": "false"})
        SubElement(lane, "width", {
            "sOffset": "0", "a": f"{DEFAULT_LANE_WIDTH_M:.2f}", "b": "0", "c": "0", "d": "0",
        })
        SubElement(lane, "roadMark", {
            "sOffset": "0",
            "type": "broken" if i < num_lanes else "solid",
            "weight": "standard", "color": "standard", "width": "0.12",
        })

    xml_bytes = tostring(odr, encoding="utf-8")
    pretty = minidom.parseString(xml_bytes).toprettyxml(indent="  ")
    return pretty
