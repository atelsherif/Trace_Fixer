"""OpenDRIVE (.xodr) generator.

Scope (deliberately limited -- see project README): the road's reference
line follows the (fixed) ego path, and lane count/width are estimated from
annotation data (lane markings, cross-checked against vehicles' obj_lane
labels and the annotation's own frame_meta.num_lanes) with a robust
fallback to a constant-width, constant-count cross-section wherever that
estimate isn't trustworthy -- see export.road_geometry for the estimation
and its sanity checks. This is meant to be "good enough to run our own
in-GUI simulation, sanity-check trajectories against lane boundaries, and
interoperate with other ASAM-OpenDRIVE-consuming tools", not a replacement
for a real HD map. Notably: the reference line runs along the ego's own
driven path rather than a true lane-center/road-center line, and plan-view
geometry is a sequence of constant-curvature arcs fit to the ego's real
heading profile (not full clothoid/spiral curvature continuity).
"""
from __future__ import annotations

import math
from xml.dom import minidom
from xml.etree.ElementTree import Element, SubElement, tostring

from trace_fixer.export.road_geometry import RoadGeometryPlan, build_road_geometry_plan
from trace_fixer.models import Trace

# Below this curvature, a segment renders as <line/> rather than <arc/> --
# avoids emitting a technically-nonzero but meaningless curvature value for
# what's really a straight stretch (heading noise between two samples).
STRAIGHT_CURVATURE_EPS = 1e-4


def _emit_plan_view(plan_view: Element, plan: RoadGeometryPlan) -> None:
    ref = plan.ref_points
    for i in range(len(ref) - 1):
        a, b = ref[i], ref[i + 1]
        length = b.s - a.s
        if length <= 1e-6:
            continue
        dheading = ((b.heading_rad - a.heading_rad + math.pi) % (2 * math.pi)) - math.pi
        curvature = dheading / length
        geometry = SubElement(
            plan_view,
            "geometry",
            {"s": f"{a.s:.3f}", "x": f"{a.x:.3f}", "y": f"{a.y:.3f}", "hdg": f"{a.heading_rad:.6f}", "length": f"{length:.3f}"},
        )
        if abs(curvature) < STRAIGHT_CURVATURE_EPS:
            SubElement(geometry, "line")
        else:
            SubElement(geometry, "arc", {"curvature": f"{curvature:.6f}"})


def _emit_lanes(lanes: Element, plan: RoadGeometryPlan) -> None:
    for section in plan.lane_sections:
        lane_section = SubElement(lanes, "laneSection", {"s": f"{section.s_start:.3f}"})
        center = SubElement(lane_section, "center")
        center_lane = SubElement(center, "lane", {"id": "0", "type": "none", "level": "false"})
        SubElement(
            center_lane, "roadMark", {"sOffset": "0", "type": "solid", "weight": "standard", "color": "standard", "width": "0.12"}
        )

        right = SubElement(lane_section, "right")
        for i in range(1, section.num_lanes + 1):
            width_m = section.lane_widths_m[i - 1]
            lane = SubElement(right, "lane", {"id": str(-i), "type": "driving", "level": "false"})
            SubElement(lane, "width", {"sOffset": "0", "a": f"{width_m:.2f}", "b": "0", "c": "0", "d": "0"})
            SubElement(
                lane,
                "roadMark",
                {
                    "sOffset": "0",
                    "type": "broken" if i < section.num_lanes else "solid",
                    "weight": "standard",
                    "color": "standard",
                    "width": "0.12",
                },
            )


def _emit_objects(road: Element, plan: RoadGeometryPlan) -> None:
    if not plan.static_objects:
        return
    objects = SubElement(road, "objects")
    for placement in plan.static_objects:
        SubElement(
            objects,
            "object",
            {
                "id": str(placement.obj_id),
                "name": placement.label,
                "type": placement.odr_type,
                "s": f"{placement.s:.3f}",
                "t": f"{placement.t:.3f}",
                "zOffset": "0",
                "hdg": "0",
                "pitch": "0",
                "roll": "0",
                "length": f"{placement.length:.2f}",
                "width": f"{placement.width:.2f}",
                "height": f"{placement.height:.2f}",
                "orientation": "none",
            },
        )


def generate_opendrive(trace: Trace, road_name: str = "trace_fixer_road") -> str:
    plan = build_road_geometry_plan(trace)

    odr = Element("OpenDRIVE")
    SubElement(
        odr,
        "header",
        {"revMajor": "1", "revMinor": "6", "name": road_name, "version": "1.00", "north": "0", "south": "0", "east": "0", "west": "0"},
    )

    road = SubElement(odr, "road", {"name": road_name, "length": f"{plan.total_length:.3f}", "id": "1", "junction": "-1"})
    plan_view = SubElement(road, "planView")
    _emit_plan_view(plan_view, plan)

    lanes = SubElement(road, "lanes")
    SubElement(lanes, "laneOffset", {"s": "0", "a": "0", "b": "0", "c": "0", "d": "0"})
    _emit_lanes(lanes, plan)

    _emit_objects(road, plan)

    xml_bytes = tostring(odr, encoding="utf-8")
    return minidom.parseString(xml_bytes).toprettyxml(indent="  ")
