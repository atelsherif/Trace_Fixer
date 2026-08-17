"""Parser for the manually-annotated scene XML (Lidar/vision label export).

Format notes (reverse engineered from sample exports):
  - <vehicles>/<rect_vehicle> holds one track per tracked object, with sparse
    per-frame <rect_vehicle_timestamp> observations. Bounding box coordinates
    (xp, yp, zp, xs, ys, zs, zrot) are in the *ego vehicle frame at that
    instant* (x forward, y left, zrot heading offset from ego).
  - zrot's *unit* is not consistent across exports: some files (observed
    with structurefile minorversion 8) use radians, others (minorversion 7)
    use degrees. There's no explicit unit field, so it's auto-detected per
    file (see `detect_vehicle_zrot_unit`) and always normalized to radians
    on the parsed VehicleObs -- downstream code never needs to know which
    convention the source file used.
  - <lane_markings>/<line_static_lm> and <border_polygons>/<line_bp> hold
    polyline geometry, also ego-relative, but only sampled at a handful of
    keyframes (typically scene start/end plus any frame where the road
    topology changes) rather than every frame. Points are encoded as
    xp_1/yp_1/zp_1, xp_2/yp_2/zp_2, ... in element order.
  - <static_objects>/<rect_static> holds static objects (signs, poles, ...)
    with the same per-instant ego-relative bounding box convention as
    vehicles. Their zrot appears to follow a different (wider-range, not
    investigated) convention and is left untouched by unit detection.
"""
from __future__ import annotations

import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path

from trace_fixer.models import (
    Annotation,
    BorderLine,
    FrameMeta,
    LaneMarking,
    LaneSnapshot,
    StaticObject,
    StaticObs,
    VehicleObs,
    VehicleTrack,
)

_POINT_RE = re.compile(r"^xp_(\d+)$")


def _text(el: ET.Element, tag: str, default: str | None = None) -> str | None:
    child = el.find(tag)
    return child.text if child is not None and child.text is not None else default


def _float(el: ET.Element, tag: str, default: float = 0.0) -> float:
    v = _text(el, tag)
    return float(v) if v is not None else default


def _int_attr(el: ET.Element, attr: str, default: int = 0) -> int:
    v = el.attrib.get(attr)
    return int(v) if v is not None else default


def _parse_bbox_coords(coords: ET.Element) -> tuple[float, float, float, float, float, float, float]:
    xp = float(coords.findtext("xp", "0"))
    yp = float(coords.findtext("yp", "0"))
    zp = float(coords.findtext("zp", "0"))
    xs = float(coords.findtext("xs", "0"))
    ys = float(coords.findtext("ys", "0"))
    zs = float(coords.findtext("zs", "0"))
    zrot = float(coords.findtext("zrot", "0"))
    return xp, yp, zp, xs, ys, zs, zrot


def _parse_polyline_coords(coords: ET.Element) -> list[tuple[float, float]]:
    """Extract (x, y) points from xp_N / yp_N / zp_N elements, ordered by N."""
    indices: set[int] = set()
    for child in coords:
        m = _POINT_RE.match(child.tag)
        if m:
            indices.add(int(m.group(1)))
    points = []
    for n in sorted(indices):
        x = coords.findtext(f"xp_{n}")
        y = coords.findtext(f"yp_{n}")
        if x is not None and y is not None:
            points.append((float(x), float(y)))
    return points


def _parse_vehicles(root: ET.Element) -> dict[int, VehicleTrack]:
    out: dict[int, VehicleTrack] = {}
    vehicles_el = root.find("vehicles")
    if vehicles_el is None:
        return out
    for rv in vehicles_el.findall("rect_vehicle"):
        obj_id = int(_text(rv, "id", "0"))
        track = VehicleTrack(
            obj_id=obj_id,
            obj_type=_text(rv, "obj_type", "Unknown") or "Unknown",
            reflecting_parts=_text(rv, "reflecting_parts"),
        )
        timestamps_el = rv.find("timestamps")
        if timestamps_el is not None:
            for ts in timestamps_el.findall("rect_vehicle_timestamp"):
                coords = ts.find("bounding_box/vehicle_bb/coordinates")
                if coords is None:
                    continue
                xp, yp, zp, xs, ys, zs, zrot = _parse_bbox_coords(coords)
                track.observations.append(
                    VehicleObs(
                        t_us=_int_attr(ts, "chunktime"),
                        frame=_int_attr(ts, "frame"),
                        obj_movement=_text(ts, "obj_movement", "") or "",
                        obj_lane=_text(ts, "obj_lane", "") or "",
                        obj_confidence=_text(ts, "obj_confidence", "") or "",
                        x_rel=xp,
                        y_rel=yp,
                        z_rel=zp,
                        length=xs,
                        width=ys,
                        height=zs,
                        zrot=zrot,
                        interpolation_state=ts.attrib.get("interpolationState"),
                    )
                )
        track.observations.sort(key=lambda o: o.t_us)
        out[obj_id] = track
    return out


def _parse_lane_markings(root: ET.Element) -> dict[int, LaneMarking]:
    out: dict[int, LaneMarking] = {}
    lm_el = root.find("lane_markings")
    if lm_el is None:
        return out
    for line in lm_el.findall("line_static_lm"):
        obj_id = int(_text(line, "id", "0"))
        lane = LaneMarking(
            obj_id=obj_id,
            lane_type=_text(line, "lane_type", "") or "",
            width=_float(line, "width"),
            group=int(_text(line, "group", "0")),
        )
        la = line.find("line_attributes")
        if la is not None:
            for ts in la.findall("line_static_check_II_timestamp"):
                coords = ts.find("line_static/line_static_check_II_geom/coordinates")
                if coords is None:
                    continue
                lane.snapshots.append(
                    LaneSnapshot(
                        t_us=_int_attr(ts, "chunktime"),
                        frame=_int_attr(ts, "frame"),
                        points_rel=_parse_polyline_coords(coords),
                    )
                )
        lane.snapshots.sort(key=lambda s: s.t_us)
        out[obj_id] = lane
    return out


def _parse_border_lines(root: ET.Element) -> dict[int, BorderLine]:
    out: dict[int, BorderLine] = {}
    bp_el = root.find("border_polygons")
    if bp_el is None:
        return out
    for line in bp_el.findall("line_bp"):
        obj_id = int(_text(line, "id", "0"))
        border = BorderLine(obj_id=obj_id, obj_type=_text(line, "obj_type", "") or "")
        la = line.find("line_attributes")
        if la is not None:
            for ts in la.findall("line_static_timestamp"):
                coords = ts.find("line_static/line_static_geom/coordinates")
                if coords is None:
                    continue
                border.snapshots.append(
                    LaneSnapshot(
                        t_us=_int_attr(ts, "chunktime"),
                        frame=_int_attr(ts, "frame"),
                        points_rel=_parse_polyline_coords(coords),
                    )
                )
        border.snapshots.sort(key=lambda s: s.t_us)
        out[obj_id] = border
    return out


def _parse_static_objects(root: ET.Element) -> dict[int, StaticObject]:
    out: dict[int, StaticObject] = {}
    so_el = root.find("static_objects")
    if so_el is None:
        return out
    for rs in so_el.findall("rect_static"):
        obj_id = int(_text(rs, "id", "0"))
        static_obj = StaticObject(obj_id=obj_id, obj_type=_text(rs, "obj_type", "") or "")
        timestamps_el = rs.find("timestamps")
        if timestamps_el is not None:
            for ts in timestamps_el.findall("rect_static_timestamp"):
                coords = ts.find("bounding_box/rect_static_geom/coordinates")
                if coords is None:
                    continue
                xp, yp, zp, xs, ys, zs, zrot = _parse_bbox_coords(coords)
                static_obj.observations.append(
                    StaticObs(
                        t_us=_int_attr(ts, "chunktime"),
                        frame=_int_attr(ts, "frame"),
                        x_rel=xp,
                        y_rel=yp,
                        z_rel=zp,
                        length=xs,
                        width=ys,
                        height=zs,
                        zrot=zrot,
                    )
                )
        static_obj.observations.sort(key=lambda o: o.t_us)
        out[obj_id] = static_obj
    return out


def _parse_frame_meta(root: ET.Element) -> list[FrameMeta]:
    out: list[FrameMeta] = []
    ts_root = root.find("frame_specific/timestamp")
    if ts_root is None:
        return out
    for fs in ts_root.findall("frame_specific_timestamp"):
        num_lanes = _text(fs, "NUM_lanes")
        ego_lane = _text(fs, "EGO_lane")
        out.append(
            FrameMeta(
                t_us=_int_attr(fs, "chunktime"),
                frame=_int_attr(fs, "frame"),
                road_type=_text(fs, "Road_type"),
                num_lanes=int(num_lanes) if num_lanes else None,
                ego_lane=int(ego_lane) if ego_lane and ego_lane.isdigit() else None,
                weather=_text(fs, "Weather_1"),
                light_conditions=_text(fs, "Light_conditions"),
            )
        )
    out.sort(key=lambda m: m.t_us)
    return out


def detect_vehicle_zrot_unit(vehicles: dict[int, VehicleTrack]) -> str:
    """Returns "rad" or "deg". A properly-wrapped relative heading can never
    legitimately exceed +/-pi radians -- so if any raw zrot value does, the
    file must be using degrees (empirically confirmed against
    position-implied heading on multiple real exports; see README).
    """
    max_abs = 0.0
    for track in vehicles.values():
        for obs in track.observations:
            max_abs = max(max_abs, abs(obs.zrot))
    return "deg" if max_abs > math.pi else "rad"


def parse_annotation_xml(path: str | Path) -> Annotation:
    path = Path(path)
    tree = ET.parse(path)
    root = tree.getroot()
    country_code = None
    scene_attrs = root.find("scene_attributes")
    if scene_attrs is not None:
        country_code = _text(scene_attrs, "Country_code")

    vehicles = _parse_vehicles(root)
    zrot_unit = detect_vehicle_zrot_unit(vehicles)
    if zrot_unit == "deg":
        for track in vehicles.values():
            for obs in track.observations:
                obs.zrot = math.radians(obs.zrot)

    return Annotation(
        country_code=country_code,
        frame_meta=_parse_frame_meta(root),
        vehicles=vehicles,
        lane_markings=_parse_lane_markings(root),
        border_lines=_parse_border_lines(root),
        static_objects=_parse_static_objects(root),
        vehicle_zrot_unit=zrot_unit,
    )
