"""Ties parsers + geo transforms together into a loaded Trace and a JSON scene
payload consumable by the frontend."""
from __future__ import annotations

import math
from pathlib import Path

from trace_fixer.analysis import build_trace_summary
from trace_fixer.geo.populate import populate_global_coords
from trace_fixer.geo.transform import global_to_ego_relative, heading_to_yaw_rad
from trace_fixer.models import Trace
from trace_fixer.parsers.adma_csv import parse_adma_csv
from trace_fixer.parsers.annotation_xml import parse_annotation_xml

EGO_LENGTH_M = 4.9
EGO_WIDTH_M = 1.9


def load_trace(trace_id: str, adma_csv_path: str | Path, annotation_xml_path: str | Path) -> Trace:
    ego = parse_adma_csv(adma_csv_path)
    annotation = parse_annotation_xml(annotation_xml_path)
    trace = Trace(trace_id=trace_id, ego=ego, annotation=annotation)
    populate_global_coords(trace)
    return trace


def _downsample(seq: list, max_points: int) -> list:
    if len(seq) <= max_points:
        return seq
    step = math.ceil(len(seq) / max_points)
    return seq[::step]


_OVERTAKE_LABELS = {
    "vehicle_overtakes_ego": "Vehicle overtakes ego",
    "ego_overtakes_vehicle": "Ego overtakes vehicle",
}


def _build_events(trace: Trace, t0: int) -> list[dict]:
    """Behavioral events for the GUI's Events panel -- distinct from
    trace.issues (data-quality problems): these are "what happened" rather
    than "what's wrong," computed fresh from the trace's current state
    (so re-running after a fix/predict reflects the corrected trace), same
    detectors that feed the trace summary report and catalog phenomena.
    """
    summary = build_trace_summary(trace)

    def rel_s(t_us: int) -> float:
        return (t_us - t0) / 1e6

    events: list[dict] = []
    for i, ev in enumerate(summary.braking_events):
        events.append(
            {
                "event_id": f"braking-{i}",
                "type": "braking",
                "vehicle_id": None,
                "t_start_s": rel_s(ev.t_start_us),
                "t_end_s": rel_s(ev.t_end_us),
                "description": f"{ev.severity.capitalize()} braking (peak {ev.peak_decel_mps2:.1f} m/s²)",
            }
        )
    for i, ev in enumerate(summary.overtake_events):
        events.append(
            {
                "event_id": f"overtake-{i}",
                "type": "overtake",
                "vehicle_id": ev.vehicle_id,
                "t_start_s": rel_s(ev.t_us),
                "t_end_s": rel_s(ev.t_us),
                "description": f"{_OVERTAKE_LABELS[ev.direction]} (vehicle {ev.vehicle_id})",
            }
        )
    for i, ev in enumerate(summary.short_headway_events):
        label = "Near miss" if ev.near_miss else "Short headway"
        events.append(
            {
                "event_id": f"headway-{i}",
                "type": "near_miss" if ev.near_miss else "short_headway",
                "vehicle_id": ev.vehicle_id,
                "t_start_s": rel_s(ev.t_start_us),
                "t_end_s": rel_s(ev.t_end_us),
                "description": f"{label}: vehicle {ev.vehicle_id}, {ev.min_headway_s:.2f}s gap",
            }
        )
    for i, ev in enumerate(summary.cut_in_events):
        events.append(
            {
                "event_id": f"cutin-{i}",
                "type": "cut_in",
                "vehicle_id": ev.vehicle_id,
                "t_start_s": rel_s(ev.t_us),
                "t_end_s": rel_s(ev.t_us),
                "description": f"Vehicle {ev.vehicle_id} cuts in, {ev.range_m:.0f} m ahead",
            }
        )
    for i, ev in enumerate(summary.standstill_events):
        events.append(
            {
                "event_id": f"standstill-{i}",
                "type": "standstill",
                "vehicle_id": None,
                "t_start_s": rel_s(ev.t_start_us),
                "t_end_s": rel_s(ev.t_end_us),
                "description": f"Ego standstill ({rel_s(ev.t_end_us) - rel_s(ev.t_start_us):.1f}s)",
            }
        )
    for i, ev in enumerate(summary.sharp_turn_events):
        events.append(
            {
                "event_id": f"turn-{i}",
                "type": "sharp_turn",
                "vehicle_id": None,
                "t_start_s": rel_s(ev.t_start_us),
                "t_end_s": rel_s(ev.t_end_us),
                "description": f"Sharp turn ({ev.heading_change_deg:+.0f}°)",
            }
        )

    events.sort(key=lambda e: e["t_start_s"])
    return events


def build_scene_json(trace: Trace, max_ego_points: int = 1500) -> dict:
    t0 = trace.ego.t0_us

    ego_path = []
    for pose in _downsample(trace.ego.poses, max_ego_points):
        # vx_mps/vy_mps are North/East velocity, not vehicle-frame forward/
        # lateral despite the field names (see models.EgoPose) -- rotate
        # into the vehicle frame with the same helper used for annotation
        # coordinates, passing (East, North) = (vy_mps, vx_mps).
        yaw_rad = heading_to_yaw_rad(pose.heading_deg)
        v_fwd, v_lat = global_to_ego_relative(pose.vy_mps, pose.vx_mps, 0.0, 0.0, yaw_rad)
        ego_path.append(
            {
                "t_s": (pose.t_us - t0) / 1e6,
                "x": pose.x_m,
                "y": pose.y_m,
                "heading_deg": (90 + pose.heading_deg) % 360,
                "speed_mps": math.hypot(pose.vx_mps, pose.vy_mps),
                "v_fwd_mps": v_fwd,
                "v_lat_mps": v_lat,
                "v_vert_mps": pose.vz_mps,
                "lat": pose.lat_deg,
                "lon": pose.lon_deg,
            }
        )

    vehicles = []
    for track in trace.annotation.vehicles.values():
        obs_list = [
            {
                "t_s": (obs.t_us + trace.sync_offset_us - t0) / 1e6,
                "frame": obs.frame,
                "x": obs.x_m,
                "y": obs.y_m,
                "heading_deg": obs.heading_deg,
                "length": obs.length,
                "width": obs.width,
                "obj_lane": obs.obj_lane,
                "obj_movement": obs.obj_movement,
                "obj_confidence": obs.obj_confidence,
                "synthetic": obs.synthetic,
            }
            for obs in track.observations
        ]
        vehicles.append({"id": track.obj_id, "obj_type": track.obj_type, "observations": obs_list})

    def _build_lines(lines_dict) -> list:
        out = []
        for line in lines_dict.values():
            snapshots = [
                {
                    "t_s": (snap.t_us + trace.sync_offset_us - t0) / 1e6,
                    "frame": snap.frame,
                    "points": snap.points_m,
                }
                for snap in line.snapshots
            ]
            out.append(
                {
                    "id": line.obj_id,
                    "type": getattr(line, "lane_type", None) or getattr(line, "obj_type", None),
                    "snapshots": snapshots,
                }
            )
        return out

    lane_markings = _build_lines(trace.annotation.lane_markings)
    border_lines = _build_lines(trace.annotation.border_lines)

    static_objects = []
    for so in trace.annotation.static_objects.values():
        obs_list = [
            {
                "t_s": (obs.t_us + trace.sync_offset_us - t0) / 1e6,
                "frame": obs.frame,
                "x": obs.x_m,
                "y": obs.y_m,
                "length": obs.length,
                "width": obs.width,
            }
            for obs in so.observations
        ]
        static_objects.append({"id": so.obj_id, "obj_type": so.obj_type, "observations": obs_list})

    duration_s = (trace.ego.t1_us - trace.ego.t0_us) / 1e6

    return {
        "events": _build_events(trace, t0),
        "trace_id": trace.trace_id,
        "duration_s": duration_s,
        "ego": {"length": EGO_LENGTH_M, "width": EGO_WIDTH_M, "path": ego_path},
        "vehicles": vehicles,
        "lane_markings": lane_markings,
        "border_lines": border_lines,
        "static_objects": static_objects,
        "sync_offset_us": trace.sync_offset_us,
        "issues": [
            {
                "issue_id": i.issue_id,
                "category": i.category,
                "severity": i.severity,
                "vehicle_id": i.vehicle_id,
                "t_start_s": (i.t_start_us - t0) / 1e6,
                "t_end_s": (i.t_end_us - t0) / 1e6,
                "description": i.description,
                "fixable": i.fixable,
                "fixed": i.fixed,
            }
            for i in trace.issues
        ],
    }
