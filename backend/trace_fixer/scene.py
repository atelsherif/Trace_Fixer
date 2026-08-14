"""Ties parsers + geo transforms together into a loaded Trace and a JSON scene
payload consumable by the frontend."""
from __future__ import annotations

import math
from pathlib import Path

from trace_fixer.geo.populate import populate_global_coords
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


def build_scene_json(trace: Trace, max_ego_points: int = 1500) -> dict:
    t0 = trace.ego.t0_us

    ego_path = []
    for pose in _downsample(trace.ego.poses, max_ego_points):
        ego_path.append(
            {
                "t_s": (pose.t_us - t0) / 1e6,
                "x": pose.x_m,
                "y": pose.y_m,
                "heading_deg": (90 + pose.heading_deg) % 360,
                "speed_mps": math.hypot(pose.vx_mps, pose.vy_mps),
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
