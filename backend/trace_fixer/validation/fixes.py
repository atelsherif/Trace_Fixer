"""Rule-based fix engine: smooths noisy vehicle tracks, clamps off-road
positions back into the annotated drivable corridor, and drops trailing
observations that still collide with the ego vehicle after smoothing
(a common "lost track right as it merges into the ego lane" artifact).

Two principles keep the engine from editing away real driving:

* **Smoothing is targeted, bounded and optional.** It only touches the
  stretches of a track that validation actually flagged as kinematically
  implausible, tapers back into the recorded path at the edges of those
  stretches, and can never move an observation more than
  MAX_SMOOTH_SHIFT_M. A vehicle that simply is not travelling in a perfectly
  straight line is not a defect, and the engine now leaves it alone instead
  of flattening it.
* **The off-road clamp nudges; it never relocates.** A vehicle that has
  genuinely left the ego's road (an exit ramp, a turn into a side street) is
  reported for review and left exactly as recorded -- see
  geo.road_departure. Even for genuine annotation noise the clamp gives up
  past MAX_CLAMP_M, because a correction that large is evidence the corridor
  geometry, not the vehicle, is what's wrong.

Every fix keeps the observation's own timestamp and re-derives *both* the
global (x_m, y_m, heading_deg) and ego-relative (x_rel, y_rel, zrot) fields
so the corrected track stays consistent for rendering, re-validation, and
export.
"""
from __future__ import annotations

import math

import numpy as np

from trace_fixer.geo.road_corridor import MIN_EXCURSION_M, OFFROAD_MARGIN_M, corridor_bounds
from trace_fixer.geo.road_departure import DEPARTURE, classify_offroad
from trace_fixer.geo.sync import apply_offset
from trace_fixer.geo.transform import (
    EgoInterpolator,
    ego_relative_to_global,
    global_heading_to_zrot,
    global_to_ego_relative,
)
from trace_fixer.models import Trace, VehicleTrack
from trace_fixer.validation.checks import EGO_LENGTH_M, EGO_WIDTH_M, run_validation
from trace_fixer.geo.collision import obb_overlap

MIN_POINTS_FOR_SMOOTHING = 5
SMOOTH_FACTOR_PER_POINT = 0.25  # UnivariateSpline `s`, larger = smoother

# Named smoothing strengths, multiplying SMOOTH_FACTOR_PER_POINT. "off" skips
# smoothing entirely and leaves every position exactly as recorded.
SMOOTHING_STRENGTHS = {"off": 0.0, "light": 0.4, "standard": 1.0, "strong": 2.5}
DEFAULT_SMOOTHING = "standard"

# How far either side of a flagged interval still counts as part of it: the
# frames bracketing an implausible jump are part of the same artifact.
SMOOTH_WINDOW_PAD_S = 0.5
# Cosine taper outside that window, so a corrected stretch rejoins the
# recorded path continuously instead of stepping onto it.
SMOOTH_BLEND_S = 0.5
# A smoother may nudge an observation; it may not relocate it. Anything
# needing more than this is not jitter.
MAX_SMOOTH_SHIFT_M = 1.0
# Same idea for the corridor clamp.
MAX_CLAMP_M = 1.5


def _flagged_windows(trace: Trace, obj_id: int) -> list[tuple[int, int]]:
    """The (t_start_us, t_end_us) intervals of this vehicle's kinematic
    issues, padded. These -- and only these -- are what smoothing may touch.
    Collision and off-road issues are handled by the clamp and the trailing
    trim, not by bending the path.
    """
    pad = int(SMOOTH_WINDOW_PAD_S * 1e6)
    return [
        (issue.t_start_us - pad, issue.t_end_us + pad)
        for issue in trace.issues
        if issue.vehicle_id == obj_id and issue.category == "kinematic"
    ]


def _blend_weight(t_us: int, windows: list[tuple[int, int]]) -> float:
    """1.0 inside a flagged window, cosine-tapering to 0.0 over
    SMOOTH_BLEND_S outside it."""
    if not windows:
        return 0.0
    gap_us = min(max(start - t_us, 0, t_us - end) for start, end in windows)
    if gap_us <= 0:
        return 1.0
    blend_us = SMOOTH_BLEND_S * 1e6
    if gap_us >= blend_us:
        return 0.0
    return 0.5 * (1.0 + math.cos(math.pi * gap_us / blend_us))


def _smooth_positions(track: VehicleTrack, windows: list[tuple[int, int]], strength: float) -> None:
    obs = track.observations
    n = len(obs)
    if n < MIN_POINTS_FOR_SMOOTHING or strength <= 0.0:
        return
    weights = [_blend_weight(o.t_us, windows) for o in obs]
    if not any(w > 0.0 for w in weights):
        return  # nothing about this track was flagged as implausible
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
    s = SMOOTH_FACTOR_PER_POINT * strength * len(t_unique)
    try:
        # The fit spans the whole track -- a spline fitted to a short window
        # alone is dominated by its own end conditions -- but only the
        # flagged stretches are written back, via `weights` below.
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

    touched = [False] * n
    for i, o in enumerate(obs):
        w = weights[i]
        if w <= 0.0:
            continue
        dx, dy = x_smooth[i] - o.x_m, y_smooth[i] - o.y_m
        shift = math.hypot(dx, dy)
        if shift > MAX_SMOOTH_SHIFT_M:
            scale = MAX_SMOOTH_SHIFT_M / shift
            dx, dy = dx * scale, dy * scale
        o.x_m += w * dx
        o.y_m += w * dy
        if math.hypot(w * dx, w * dy) > 0.05:
            o.fixed = True
            touched[i] = True

    # Re-derive heading from the smoothed path tangent (central difference),
    # which also damps the box-angle jitter that drives spurious yaw-rate
    # flags -- but only where the position was actually corrected. Rewriting
    # every heading from the path tangent would discard the annotated box
    # orientation across the whole track, including the stretches nothing
    # was wrong with.
    for i, o in enumerate(obs):
        if n == 1 or not touched[i]:
            continue
        i0, i1 = max(0, i - 1), min(n - 1, i + 1)
        if i0 == i1:
            continue
        dx = obs[i1].x_m - obs[i0].x_m
        dy = obs[i1].y_m - obs[i0].y_m
        if math.hypot(dx, dy) < 1e-3:
            continue
        tangent = math.degrees(math.atan2(dy, dx)) % 360
        delta = ((tangent - o.heading_deg) + 180) % 360 - 180
        o.heading_deg = (o.heading_deg + weights[i] * delta) % 360


def _reproject_relative(track: VehicleTrack, interp: EgoInterpolator, sync_offset_us: int) -> None:
    for o in track.observations:
        t_ego_us = apply_offset(o.t_us, sync_offset_us)
        ex, ey, eyaw, _ = interp.at(t_ego_us)
        o.x_rel, o.y_rel = global_to_ego_relative(o.x_m, o.y_m, ex, ey, eyaw)
        o.zrot = global_heading_to_zrot(math.radians(o.heading_deg), eyaw)


def _clamp_offroad(track: VehicleTrack, trace: Trace, interp: EgoInterpolator) -> tuple[int, int]:
    """Pulls boxes that sit slightly outside the annotated corridor back
    inside it. Returns (departures_left_alone, clamps_refused_as_too_large).

    Observations the departure classifier calls a real turn-off are skipped,
    and so is any correction larger than MAX_CLAMP_M: the recorded position
    stands, and the validator's flag stands with it.
    """
    labels = classify_offroad(track, trace.annotation.border_lines)
    departures = 0
    too_large = 0
    for o in track.observations:
        if labels.get(o.t_us) == DEPARTURE:
            departures += 1
            continue
        left, right = corridor_bounds(trace.annotation.border_lines, o.t_us, o.x_rel)
        half_w = o.width / 2.0
        new_y_rel = o.y_rel
        if left is not None and o.y_rel - half_w > left - OFFROAD_MARGIN_M:
            new_y_rel = left - OFFROAD_MARGIN_M + half_w
        elif right is not None and o.y_rel + half_w < right + OFFROAD_MARGIN_M:
            new_y_rel = right + OFFROAD_MARGIN_M - half_w
        shift = abs(new_y_rel - o.y_rel)
        # Same floor the off-road check uses, so the clamp never edits
        # something the validator didn't consider worth flagging.
        if shift < MIN_EXCURSION_M:
            continue
        if shift > MAX_CLAMP_M:
            too_large += 1
            continue
        o.y_rel = new_y_rel
        o.fixed = True
        t_ego_us = apply_offset(o.t_us, trace.sync_offset_us)
        ex, ey, eyaw, _ = interp.at(t_ego_us)
        o.x_m, o.y_m = ego_relative_to_global(o.x_rel, o.y_rel, ex, ey, eyaw)
    return departures, too_large


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


def apply_fixes(trace: Trace, smoothing: str = DEFAULT_SMOOTHING) -> list[str]:
    """Applies smoothing + off-road clamp + trailing-collision trim to every
    vehicle track, then re-runs validation. Returns a list of fix summary
    strings for the UI/log.

    `smoothing` is one of SMOOTHING_STRENGTHS ("off" / "light" / "standard" /
    "strong"). Smoothing only ever touches stretches that validation flagged
    as kinematically implausible, so it needs the issue list -- validation is
    run first if the caller hasn't.
    """
    strength = SMOOTHING_STRENGTHS.get(smoothing)
    if strength is None:
        raise ValueError(f"unknown smoothing strength {smoothing!r}; expected one of {sorted(SMOOTHING_STRENGTHS)}")
    if not trace.issues:
        run_validation(trace)

    interp = EgoInterpolator(trace.ego)
    summary: list[str] = []
    for track in trace.annotation.vehicles.values():
        # Snapshot before anything moves, so the GUI can draw the
        # before/after difference. Idempotent, so fixing twice still
        # compares against the as-recorded positions, not the first pass's.
        for o in track.observations:
            o.remember_original()
        before_n = len(track.observations)
        _smooth_positions(track, _flagged_windows(trace, track.obj_id), strength)
        _reproject_relative(track, interp, trace.sync_offset_us)
        departures, too_large = _clamp_offroad(track, trace, interp)
        _reproject_relative(track, interp, trace.sync_offset_us)  # zrot must reflect clamp too
        _drop_trailing_ego_collisions(track, interp, trace.sync_offset_us)
        n_fixed = sum(1 for o in track.observations if o.fixed)
        dropped = before_n - len(track.observations)
        notes: list[str] = []
        if n_fixed or dropped:
            notes.append(f"smoothed/clamped {n_fixed} observation(s)")
        if dropped:
            notes.append(f"dropped {dropped} trailing ego-overlap observation(s)")
        if departures:
            notes.append(f"left {departures} observation(s) as recorded (leaves the ego's road)")
        if too_large:
            notes.append(f"declined {too_large} corridor correction(s) over {MAX_CLAMP_M:.1f} m")
        if notes:
            summary.append(f"Vehicle {track.obj_id}: " + ", ".join(notes))
    run_validation(trace)
    return summary
