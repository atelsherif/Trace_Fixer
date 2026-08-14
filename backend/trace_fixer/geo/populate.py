"""Populates global (x_m, y_m, heading_deg) fields on every annotation element
of a Trace, using the ego trace as the reference frame. Single source of
truth consumed by scene JSON building, validation, prediction, fixing, and
export.
"""
from __future__ import annotations

import math

from trace_fixer.geo.sync import apply_offset
from trace_fixer.geo.transform import EgoInterpolator, ego_relative_to_global, global_heading_rad, project_ego_trace
from trace_fixer.models import Trace


def populate_global_coords(trace: Trace) -> None:
    project_ego_trace(trace.ego)
    interp = EgoInterpolator(trace.ego)

    for track in trace.annotation.vehicles.values():
        for obs in track.observations:
            t_ego_us = apply_offset(obs.t_us, trace.sync_offset_us)
            ex, ey, eyaw, _ = interp.at(t_ego_us)
            obs.x_m, obs.y_m = ego_relative_to_global(obs.x_rel, obs.y_rel, ex, ey, eyaw)
            obs.heading_deg = math.degrees(global_heading_rad(eyaw, obs.zrot)) % 360

    for so in trace.annotation.static_objects.values():
        for obs in so.observations:
            t_ego_us = apply_offset(obs.t_us, trace.sync_offset_us)
            ex, ey, eyaw, _ = interp.at(t_ego_us)
            obs.x_m, obs.y_m = ego_relative_to_global(obs.x_rel, obs.y_rel, ex, ey, eyaw)

    for lines_dict in (trace.annotation.lane_markings, trace.annotation.border_lines):
        for line in lines_dict.values():
            for snap in line.snapshots:
                t_ego_us = apply_offset(snap.t_us, trace.sync_offset_us)
                ex, ey, eyaw, _ = interp.at(t_ego_us)
                snap.points_m = [ego_relative_to_global(px, py, ex, ey, eyaw) for px, py in snap.points_rel]
