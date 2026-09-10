"""Trajectory prediction for the parts of a vehicle's path the Lidar never
saw: before it entered the ~120deg front-bumper field of view (already
moving when first observed) and after it left it -- most commonly a vehicle
the ego overtakes, or that overtakes the ego and pulls away, dropping out of
the front-facing cone while still on the road.

Model (both directions): constant-speed extrapolation from the vehicle's
observed velocity at the relevant end of its track, with heading gently
steered toward the local road-tangent (taken from the ego path, a clean,
continuous polyline covering the whole corridor) so the predicted segment
curves with the road instead of running off it in a straight line. This is
intentionally a simple, explainable model -- there's no ground truth for
what a vehicle did while unobserved, so the goal is "a plausible,
road-following continuation", not a precise reconstruction.

Predictions are still generated per-vehicle, one track at a time, but
(unless `avoid_collisions=False`) each step checks whether the vehicle's
box at the resulting position would overlap the ego or another vehicle's
box at that same instant and, if so, brakes instead of accepting the
overlap -- a simple following-distance governor, not a maneuver: a
predicted vehicle only ever slows down (down to a full stop) or recovers
speed once clear, it never changes heading to steer around a conflict.
Only vehicles already predicted *earlier* in the same `predict_all` call
are visible to a later one -- see `_ConflictBoxFinder` -- so this reduces
but does not guarantee zero overlaps; whatever it doesn't catch is still
surfaced by validation rather than silently resolved.
"""
from __future__ import annotations

import bisect
import math

from trace_fixer.geo.collision import obb_overlap
from trace_fixer.geo.sync import apply_offset
from trace_fixer.geo.transform import EgoInterpolator, global_heading_to_zrot, global_to_ego_relative
from trace_fixer.models import EgoTrace, Trace, VehicleObs, VehicleTrack

DEFAULT_HORIZON_S = 4.0
DEFAULT_STEP_S = 0.2
STEER_BLEND = 0.35  # how strongly heading is pulled toward the local road tangent per step
EDGE_FRAME_MARGIN = 3  # tracks within this many frames of the clip's start/end have nothing missing to predict
# Same box size validation/checks.py uses for the ego -- duplicated rather
# than imported to keep this module's only dependencies geo/models, not
# the validation layer.
EGO_LENGTH_M = 4.9
EGO_WIDTH_M = 1.9
# A predicted step is rejected as "would collide" if the vehicle's box at
# that instant overlaps the ego or another vehicle's box grown by this
# margin on every side -- a little slack so the governor brakes *before*
# an actual overlap, not only once one has already happened.
COLLISION_SAFETY_MARGIN_M = 1.0
# Rate limits, not a speed multiplier: braking (or recovering) by a fixed
# fraction of current speed implies enormous deceleration at highway
# speeds (halving 25 m/s in one 0.2s step is -62 m/s^2) and just traded
# "collision" validation issues for "unrealistic acceleration" ones. These
# stay comfortably under ACCEL_MEDIUM_MPS2 (validation/checks.py, 6.0) so
# the governor's own braking never itself introduces a kinematic issue --
# a conflict closing faster than a real car could brake for still shows up
# as a collision, which is the honest outcome, not one to paper over by
# braking unrealistically hard.
COLLISION_BRAKE_MPS2 = 5.0
COLLISION_RECOVERY_MPS2 = 2.0
# Observations more than this far away in time from the step being checked
# don't count as "at the same instant" -- matches the tolerance
# validation/checks.py's own vehicle-vs-vehicle collision check uses.
CONFLICT_TIME_TOLERANCE_US = 300_000
# A vehicle trailing the ego (x_rel < 0, "behind") is disproportionately
# likely to be near the edge of what the sensor tracks in the first place --
# rear/side coverage is typically shorter-range than the front cone -- so it
# drops out of (or hasn't yet entered) the annotated track sooner, and its
# predicted trail was fading out too quickly to be useful for exactly the
# case that matters most: a tailgater or a vehicle the ego is pulling away
# from. Trailing vehicles get a longer predicted horizon than the base
# value; a vehicle ahead of the ego is unaffected.
REAR_HORIZON_MULTIPLIER = 3.0


def _effective_horizon(anchor: VehicleObs, value: float | None) -> float | None:
    if value is None:
        return None
    return value * REAR_HORIZON_MULTIPLIER if anchor.x_rel < 0 else value


def _wrap_rad(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _nearest_path_tangent(ego: EgoTrace, x: float, y: float) -> float:
    """Local road heading (rad) at the ego-path point nearest to (x, y)."""
    poses = ego.poses
    best_i, best_d2 = 0, float("inf")
    for i, p in enumerate(poses):
        d2 = (p.x_m - x) ** 2 + (p.y_m - y) ** 2
        if d2 < best_d2:
            best_d2, best_i = d2, i
    i0 = max(0, best_i - 1)
    i1 = min(len(poses) - 1, best_i + 1)
    if i0 == i1:
        return math.radians((90 + poses[best_i].heading_deg) % 360)
    dx = poses[i1].x_m - poses[i0].x_m
    dy = poses[i1].y_m - poses[i0].y_m
    return math.atan2(dy, dx)


def _initial_speed_heading(anchor: VehicleObs, neighbor: VehicleObs) -> tuple[float, float]:
    dt = (neighbor.t_us - anchor.t_us) / 1e6
    vx = (neighbor.x_m - anchor.x_m) / dt if dt != 0 else 0.0
    vy = (neighbor.y_m - anchor.y_m) / dt if dt != 0 else 0.0
    speed = math.hypot(vx, vy)
    heading = math.atan2(vy, vx) if speed > 0.1 else math.radians(anchor.heading_deg)
    return speed, heading


class _ConflictBoxFinder:
    """Ego + every other vehicle's box at a given instant, for the
    collision-avoiding speed governor in `_extrapolate` below. Built once
    per predict_backward/predict_forward call (not per step): each other
    track's observation times are sorted once so a per-step lookup is a
    bisect rather than a linear scan over its whole history.

    Only observations already present on a track when this is built are
    visible -- for a track processed earlier in the same `predict_all`
    call, that includes whatever it already predicted, so predicting
    vehicle B after vehicle A lets B react to A's (already-decided)
    predicted path, not just its real one.
    """

    def __init__(self, trace: Trace, ego_interp: EgoInterpolator, exclude_obj_id: int):
        self._trace = trace
        self._ego_interp = ego_interp
        self._others: list[tuple[list[int], dict[int, VehicleObs]]] = []
        for obj_id, track in trace.annotation.vehicles.items():
            if obj_id == exclude_obj_id or not track.observations:
                continue
            by_t = {o.t_us: o for o in track.observations}
            self._others.append((sorted(by_t), by_t))

    def boxes_at(self, t_cursor: int) -> list[tuple[float, float, float, float, float]]:
        t_ego_us = apply_offset(t_cursor, self._trace.sync_offset_us)
        ex, ey, eyaw, _ = self._ego_interp.at(t_ego_us)
        boxes = [(ex, ey, math.degrees(eyaw), EGO_LENGTH_M, EGO_WIDTH_M)]
        for times, by_t in self._others:
            idx = bisect.bisect_left(times, t_cursor)
            candidates = times[max(0, idx - 1) : idx + 1]
            if not candidates:
                continue
            nearest_t = min(candidates, key=lambda t: abs(t - t_cursor))
            if abs(nearest_t - t_cursor) <= CONFLICT_TIME_TOLERANCE_US:
                o = by_t[nearest_t]
                boxes.append((o.x_m, o.y_m, o.heading_deg, o.length, o.width))
        return boxes


def _extrapolate(
    anchor: VehicleObs,
    trace: Trace,
    interp: EgoInterpolator,
    speed: float,
    heading: float,
    direction: int,
    horizon_s: float,
    step_s: float,
    time_bound_us: int,
    horizon_m: float | None = None,
    conflicts: _ConflictBoxFinder | None = None,
) -> list[VehicleObs]:
    """Steps away from `anchor` in time by `direction` (+1 forward, -1
    backward) for up to horizon_s (and, if given, no further than
    horizon_m of distance travelled -- whichever limit is hit first),
    stopping at time_bound_us (the ego trace's start/end). Returned list
    is in the order generated, i.e. chronological for direction=+1 and
    reverse-chronological for -1.

    When `conflicts` is given, each step first checks whether the
    vehicle's box at the resulting position would overlap the ego or
    another vehicle's box (grown by COLLISION_SAFETY_MARGIN_M) at that
    same instant. If so, the step is retaken at a braked speed instead
    of simply accepting an overlap -- see the module docstring: this is a
    following-distance governor, not a maneuver, so the vehicle only ever
    slows down (down to a full stop) or speeds back up once clear; it
    never changes heading to steer around a conflict.
    """
    step_us = int(step_s * 1e6) * direction
    # An integer step count rather than accumulating `elapsed += step_s`:
    # floating-point drift on the latter occasionally added (or dropped)
    # one extra step right at the horizon boundary, most visibly once
    # horizon_s/step_s lands on an exact multiple (e.g. 12.0/0.2).
    total_steps = round(horizon_s / step_s)
    if horizon_m is not None and speed > 1e-6:
        total_steps = min(total_steps, math.floor(horizon_m / (speed * step_s)))
    x, y = anchor.x_m, anchor.y_m
    t_cursor = anchor.t_us + step_us
    current_speed = speed
    distance_travelled = 0.0
    synthetic: list[VehicleObs] = []
    for _ in range(total_steps):
        if not (t_cursor < time_bound_us if direction > 0 else t_cursor > time_bound_us):
            break
        tangent = _nearest_path_tangent(trace.ego, x, y)
        heading = heading + STEER_BLEND * _wrap_rad(tangent - heading)

        def _step_to(v: float) -> tuple[float, float]:
            return x + direction * math.cos(heading) * v * step_s, y + direction * math.sin(heading) * v * step_s

        if conflicts is not None:
            cand_x, cand_y = _step_to(current_speed)
            grown_length, grown_width = anchor.length + 2 * COLLISION_SAFETY_MARGIN_M, anchor.width + 2 * COLLISION_SAFETY_MARGIN_M
            if any(
                obb_overlap(cand_x, cand_y, math.degrees(heading), grown_length, grown_width, *box)
                for box in conflicts.boxes_at(t_cursor)
            ):
                current_speed = max(0.0, current_speed - COLLISION_BRAKE_MPS2 * step_s)
                cand_x, cand_y = _step_to(current_speed)
            else:
                current_speed = min(speed, current_speed + COLLISION_RECOVERY_MPS2 * step_s)
            x, y = cand_x, cand_y
        else:
            x, y = _step_to(current_speed)

        distance_travelled += abs(current_speed) * step_s
        t_ego_us = apply_offset(t_cursor, trace.sync_offset_us)
        ex, ey, eyaw, _ = interp.at(t_ego_us)
        x_rel, y_rel = global_to_ego_relative(x, y, ex, ey, eyaw)
        zrot = global_heading_to_zrot(heading, eyaw)

        synthetic.append(
            VehicleObs(
                t_us=t_cursor,
                frame=-1,
                obj_movement=anchor.obj_movement,
                obj_lane=anchor.obj_lane,
                obj_confidence="Predicted",
                x_rel=x_rel,
                y_rel=y_rel,
                z_rel=anchor.z_rel,
                length=anchor.length,
                width=anchor.width,
                height=anchor.height,
                zrot=zrot,
                x_m=x,
                y_m=y,
                heading_deg=math.degrees(heading) % 360,
                synthetic=True,
            )
        )
        t_cursor += step_us
        if horizon_m is not None and distance_travelled >= horizon_m:
            break

    return synthetic


def _total_frames(trace: Trace) -> int | None:
    if not trace.annotation.frame_meta:
        return None
    return max(m.frame for m in trace.annotation.frame_meta)


def predict_backward(
    track: VehicleTrack,
    trace: Trace,
    horizon_s: float = DEFAULT_HORIZON_S,
    step_s: float = DEFAULT_STEP_S,
    horizon_m: float | None = None,
    avoid_collisions: bool = True,
) -> int:
    """Prepends synthetic observations before the track's first real one
    (pre-FOV: the vehicle was already moving when first observed). Returns
    the number of synthetic observations added.

    `horizon_m`, if given, caps predicted *distance* in addition to
    `horizon_s`'s time cap -- whichever limit is reached first stops the
    prediction. `avoid_collisions` runs the following-distance speed
    governor described on `_extrapolate`; every other vehicle track
    already predicted earlier in the same `predict_all` call is visible
    to it, but tracks predicted later are not (see `_ConflictBoxFinder`).
    """
    real_obs = [o for o in track.observations if not o.synthetic]
    if not real_obs:
        return 0
    first = real_obs[0]
    if first.frame <= EDGE_FRAME_MARGIN:
        return 0

    if len(real_obs) >= 2:
        speed, heading = _initial_speed_heading(first, real_obs[1])
    else:
        speed, heading = 25.0, math.radians(first.heading_deg)  # fallback: typical highway speed

    interp = EgoInterpolator(trace.ego)
    conflicts = _ConflictBoxFinder(trace, interp, track.obj_id) if avoid_collisions else None
    synthetic = _extrapolate(
        first, trace, interp, speed, heading, direction=-1,
        horizon_s=_effective_horizon(first, horizon_s), step_s=step_s, time_bound_us=trace.ego.t0_us,
        horizon_m=_effective_horizon(first, horizon_m), conflicts=conflicts,
    )
    synthetic.reverse()
    track.observations = synthetic + [o for o in track.observations if not (o.synthetic and o.t_us < first.t_us)]
    return len(synthetic)


def predict_forward(
    track: VehicleTrack,
    trace: Trace,
    horizon_s: float = DEFAULT_HORIZON_S,
    step_s: float = DEFAULT_STEP_S,
    horizon_m: float | None = None,
    avoid_collisions: bool = True,
) -> int:
    """Appends synthetic observations after the track's last real one
    (post-FOV: the vehicle dropped out of the front-facing cone -- overtaken
    by, or overtaking, the ego -- while presumably still on the road).
    Returns the number of synthetic observations added. See
    `predict_backward` for `horizon_m`/`avoid_collisions`.
    """
    real_obs = [o for o in track.observations if not o.synthetic]
    if not real_obs:
        return 0
    last = real_obs[-1]
    total_frames = _total_frames(trace)
    if total_frames is not None and last.frame >= total_frames - EDGE_FRAME_MARGIN:
        return 0

    if len(real_obs) >= 2:
        speed, heading = _initial_speed_heading(real_obs[-2], last)
    else:
        speed, heading = 25.0, math.radians(last.heading_deg)

    interp = EgoInterpolator(trace.ego)
    conflicts = _ConflictBoxFinder(trace, interp, track.obj_id) if avoid_collisions else None
    synthetic = _extrapolate(
        last, trace, interp, speed, heading, direction=1,
        horizon_s=_effective_horizon(last, horizon_s), step_s=step_s, time_bound_us=trace.ego.t1_us,
        horizon_m=_effective_horizon(last, horizon_m), conflicts=conflicts,
    )
    track.observations = [o for o in track.observations if not (o.synthetic and o.t_us > last.t_us)] + synthetic
    return len(synthetic)


def predict_all(
    trace: Trace,
    horizon_s: float = DEFAULT_HORIZON_S,
    step_s: float = DEFAULT_STEP_S,
    backward: bool = True,
    forward: bool = True,
    horizon_m: float | None = None,
    avoid_collisions: bool = True,
) -> dict[int, dict[str, int]]:
    """Predicts every vehicle's missing segments. When `avoid_collisions`
    is set, this runs two passes rather than one: a first, ungoverned pass
    gives every vehicle *some* full-length predicted path, so the second,
    governed pass's `_ConflictBoxFinder` has real data for every other
    vehicle to react to -- not just whichever ones happened to be
    processed earlier in a single pass (predict_backward/predict_forward
    only ever see tracks already handled earlier in the same call). The
    second pass's own results replace the first's on every track.
    """
    if avoid_collisions:
        for track in trace.annotation.vehicles.values():
            if backward:
                predict_backward(track, trace, horizon_s=horizon_s, step_s=step_s, horizon_m=horizon_m, avoid_collisions=False)
            if forward:
                predict_forward(track, trace, horizon_s=horizon_s, step_s=step_s, horizon_m=horizon_m, avoid_collisions=False)

    added: dict[int, dict[str, int]] = {}
    for track in trace.annotation.vehicles.values():
        entry = {}
        if backward:
            n = predict_backward(
                track, trace, horizon_s=horizon_s, step_s=step_s, horizon_m=horizon_m, avoid_collisions=avoid_collisions
            )
            if n:
                entry["backward"] = n
        if forward:
            n = predict_forward(
                track, trace, horizon_s=horizon_s, step_s=step_s, horizon_m=horizon_m, avoid_collisions=avoid_collisions
            )
            if n:
                entry["forward"] = n
        if entry:
            added[track.obj_id] = entry
    return added


def clear_predictions(trace: Trace) -> None:
    for track in trace.annotation.vehicles.values():
        track.observations = [o for o in track.observations if not o.synthetic]
