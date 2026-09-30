"""Rule-based fix engine: smooths noisy vehicle tracks, clamps off-road
positions back into the annotated drivable corridor, and drops trailing
observations that still collide with the ego vehicle after smoothing
(a common "lost track right as it merges into the ego lane" artifact).

Every fix keeps the observation's own timestamp and re-derives *both* the
global (x_m, y_m, heading_deg) and ego-relative (x_rel, y_rel, zrot) fields
so the corrected track stays consistent for rendering, re-validation, and
export.
"""
from __future__ import annotations

import math

import numpy as np

from trace_fixer.geo.road_corridor import corridor_bounds
from trace_fixer.geo.sync import apply_offset
from trace_fixer.geo.transform import (
    EgoInterpolator,
    ego_relative_to_global,
    global_heading_to_zrot,
    global_to_ego_relative,
)
from trace_fixer.models import Trace, VehicleTrack
from trace_fixer.validation.checks import EGO_LENGTH_M, EGO_WIDTH_M, OFFROAD_MARGIN_M, run_validation
from trace_fixer.geo.collision import obb_overlap

MIN_POINTS_FOR_SMOOTHING = 5
SMOOTH_FACTOR_PER_POINT = 0.25  # UnivariateSpline `s`, larger = smoother


def _smooth_positions(track: VehicleTrack) -> None:
    obs = track.observations
    n = len(obs)
    if n < MIN_POINTS_FOR_SMOOTHING:
        return
    from scipy.interpolate import UnivariateSpline

    t = np.array([o.t_us for o in obs], dtype=float)
    x = np.array([o.x_m for o in obs], dtype=float)
    y = np.array([o.y_m for o in obs], dtype=float)

    # UnivariateSpline requires strictly increasing x. Real annotations do
    # repeat a timestamp (two observations of one vehicle in the same
    # frame), and scipy answers a non-monotonic fit with an all-NaN curve
    # *without raising* -- so the try/except below never saw it, every
    # position in the track became NaN, and the scene failed to serialize
    # (a 500 from Apply fixes). Collapse repeats to their mean, fit on
    # that, then evaluate at every original timestamp so duplicates still
    # each get a smoothed position.
    t_unique, inverse = np.unique(t, return_inverse=True)
    if len(t_unique) < MIN_POINTS_FOR_SMOOTHING:
        return
    counts = np.bincount(inverse)
    x_unique = np.bincount(inverse, weights=x) / counts
    y_unique = np.bincount(inverse, weights=y) / counts

    k = min(3, len(t_unique) - 1)
    s = SMOOTH_FACTOR_PER_POINT * len(t_unique)
    try:
        spl_x = UnivariateSpline(t_unique, x_unique, k=k, s=s)
        spl_y = UnivariateSpline(t_unique, y_unique, k=k, s=s)
        x_smooth = spl_x(t)
        y_smooth = spl_y(t)
    except Exception:
        return
    # Belt and braces: a silently degenerate fit must leave the track
    # exactly as it was rather than write NaN into it.
    if not (np.all(np.isfinite(x_smooth)) and np.all(np.isfinite(y_smooth))):
        return

    for i, o in enumerate(obs):
        moved = math.hypot(x_smooth[i] - o.x_m, y_smooth[i] - o.y_m)
        o.x_m, o.y_m = float(x_smooth[i]), float(y_smooth[i])
        if moved > 0.05:
            o.fixed = True

    # Re-derive heading from the smoothed path tangent (central difference);
    # this also damps the box-angle jitter that drives spurious yaw-rate flags.
    for i, o in enumerate(obs):
        if n == 1:
            continue
        i0, i1 = max(0, i - 1), min(n - 1, i + 1)
        if i0 == i1:
            continue
        dx = obs[i1].x_m - obs[i0].x_m
        dy = obs[i1].y_m - obs[i0].y_m
        if math.hypot(dx, dy) < 1e-3:
            continue
        new_heading = math.degrees(math.atan2(dy, dx)) % 360
        if abs(((new_heading - o.heading_deg) + 180) % 360 - 180) > 0.5:
            o.fixed = True
        o.heading_deg = new_heading


def _reproject_relative(track: VehicleTrack, interp: EgoInterpolator, sync_offset_us: int) -> None:
    for o in track.observations:
        t_ego_us = apply_offset(o.t_us, sync_offset_us)
        ex, ey, eyaw, _ = interp.at(t_ego_us)
        o.x_rel, o.y_rel = global_to_ego_relative(o.x_m, o.y_m, ex, ey, eyaw)
        o.zrot = global_heading_to_zrot(math.radians(o.heading_deg), eyaw)


def _clamp_offroad(track: VehicleTrack, trace: Trace, interp: EgoInterpolator) -> None:
    for o in track.observations:
        left, right = corridor_bounds(trace.annotation.border_lines, o.t_us, o.x_rel)
        half_w = o.width / 2.0
        new_y_rel = o.y_rel
        if left is not None and o.y_rel - half_w > left - OFFROAD_MARGIN_M:
            new_y_rel = left - OFFROAD_MARGIN_M + half_w
        elif right is not None and o.y_rel + half_w < right + OFFROAD_MARGIN_M:
            new_y_rel = right + OFFROAD_MARGIN_M - half_w
        if abs(new_y_rel - o.y_rel) > 1e-6:
            o.y_rel = new_y_rel
            o.fixed = True
            t_ego_us = apply_offset(o.t_us, trace.sync_offset_us)
            ex, ey, eyaw, _ = interp.at(t_ego_us)
            o.x_m, o.y_m = ego_relative_to_global(o.x_rel, o.y_rel, ex, ey, eyaw)


def _drop_trailing_ego_collisions(track: VehicleTrack, interp: EgoInterpolator, sync_offset_us: int) -> None:
    while len(track.observations) > 1:
        o = track.observations[-1]
        t_ego_us = apply_offset(o.t_us, sync_offset_us)
        ex, ey, eyaw, _ = interp.at(t_ego_us)
        eh = math.degrees(eyaw)
        if obb_overlap(o.x_m, o.y_m, o.heading_deg, o.length, o.width, ex, ey, eh, EGO_LENGTH_M, EGO_WIDTH_M):
            track.observations.pop()
        else:
            break


def apply_fixes(trace: Trace) -> list[str]:
    """Applies smoothing + off-road clamp + trailing-collision trim to every
    vehicle track, then re-runs validation. Returns a list of fix summary
    strings for the UI/log.
    """
    interp = EgoInterpolator(trace.ego)
    summary: list[str] = []
    for track in trace.annotation.vehicles.values():
        # Snapshot before anything moves, so the GUI can draw the
        # before/after difference. Idempotent, so fixing twice still
        # compares against the as-recorded positions, not the first pass's.
        for o in track.observations:
            o.remember_original()
        before_n = len(track.observations)
        _smooth_positions(track)
        _reproject_relative(track, interp, trace.sync_offset_us)
        _clamp_offroad(track, trace, interp)
        _reproject_relative(track, interp, trace.sync_offset_us)  # zrot must reflect clamp too
        _drop_trailing_ego_collisions(track, interp, trace.sync_offset_us)
        n_fixed = sum(1 for o in track.observations if o.fixed)
        dropped = before_n - len(track.observations)
        if n_fixed or dropped:
            summary.append(
                f"Vehicle {track.obj_id}: smoothed/clamped {n_fixed} observation(s)"
                + (f", dropped {dropped} trailing ego-overlap observation(s)" if dropped else "")
            )
    run_validation(trace)
    return summary
