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
road-following continuation", not a precise reconstruction. Predictions are
per-vehicle and don't reason about other traffic, so two independently
predicted segments can end up overlapping; that's surfaced by validation
rather than silently resolved.
"""
from __future__ import annotations

import math

from trace_fixer.geo.sync import apply_offset
from trace_fixer.geo.transform import EgoInterpolator, global_heading_to_zrot, global_to_ego_relative
from trace_fixer.models import EgoTrace, Trace, VehicleObs, VehicleTrack

DEFAULT_HORIZON_S = 4.0
DEFAULT_STEP_S = 0.2
STEER_BLEND = 0.35  # how strongly heading is pulled toward the local road tangent per step
EDGE_FRAME_MARGIN = 3  # tracks within this many frames of the clip's start/end have nothing missing to predict


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
) -> list[VehicleObs]:
    """Steps away from `anchor` in time by `direction` (+1 forward, -1
    backward) for up to horizon_s, stopping at time_bound_us (the ego
    trace's start/end). Returned list is in the order generated, i.e.
    chronological for direction=+1 and reverse-chronological for -1.
    """
    step_us = int(step_s * 1e6) * direction
    x, y = anchor.x_m, anchor.y_m
    t_cursor = anchor.t_us + step_us
    elapsed = 0.0
    synthetic: list[VehicleObs] = []
    while elapsed < horizon_s and (t_cursor < time_bound_us if direction > 0 else t_cursor > time_bound_us):
        tangent = _nearest_path_tangent(trace.ego, x, y)
        heading = heading + STEER_BLEND * _wrap_rad(tangent - heading)
        x += direction * math.cos(heading) * speed * step_s
        y += direction * math.sin(heading) * speed * step_s

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
        elapsed += step_s

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
) -> int:
    """Prepends synthetic observations before the track's first real one
    (pre-FOV: the vehicle was already moving when first observed). Returns
    the number of synthetic observations added.
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
    synthetic = _extrapolate(
        first, trace, interp, speed, heading, direction=-1,
        horizon_s=horizon_s, step_s=step_s, time_bound_us=trace.ego.t0_us,
    )
    synthetic.reverse()
    track.observations = synthetic + [o for o in track.observations if not (o.synthetic and o.t_us < first.t_us)]
    return len(synthetic)


def predict_forward(
    track: VehicleTrack,
    trace: Trace,
    horizon_s: float = DEFAULT_HORIZON_S,
    step_s: float = DEFAULT_STEP_S,
) -> int:
    """Appends synthetic observations after the track's last real one
    (post-FOV: the vehicle dropped out of the front-facing cone -- overtaken
    by, or overtaking, the ego -- while presumably still on the road).
    Returns the number of synthetic observations added.
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
    synthetic = _extrapolate(
        last, trace, interp, speed, heading, direction=1,
        horizon_s=horizon_s, step_s=step_s, time_bound_us=trace.ego.t1_us,
    )
    track.observations = [o for o in track.observations if not (o.synthetic and o.t_us > last.t_us)] + synthetic
    return len(synthetic)


def predict_all(
    trace: Trace,
    horizon_s: float = DEFAULT_HORIZON_S,
    step_s: float = DEFAULT_STEP_S,
    backward: bool = True,
    forward: bool = True,
) -> dict[int, dict[str, int]]:
    added: dict[int, dict[str, int]] = {}
    for track in trace.annotation.vehicles.values():
        entry = {}
        if backward:
            n = predict_backward(track, trace, horizon_s=horizon_s, step_s=step_s)
            if n:
                entry["backward"] = n
        if forward:
            n = predict_forward(track, trace, horizon_s=horizon_s, step_s=step_s)
            if n:
                entry["forward"] = n
        if entry:
            added[track.obj_id] = entry
    return added


def clear_predictions(trace: Trace) -> None:
    for track in trace.annotation.vehicles.values():
        track.observations = [o for o in track.observations if not o.synthetic]
