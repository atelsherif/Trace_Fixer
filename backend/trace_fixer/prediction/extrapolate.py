"""Backward trajectory prediction for vehicles that are already moving when
first observed -- i.e. they entered the Lidar's ~120deg front-bumper field
of view already in motion, so their approach isn't in the annotation at all.

Model: constant-speed extrapolation from the vehicle's initial observed
velocity, with heading gently steered toward the local road-tangent (taken
from the ego path, which is a clean, continuous polyline covering the whole
corridor) so the predicted approach curves with the road instead of running
off it in a straight line. This is intentionally a simple, explainable model
-- there's no ground truth for what a vehicle did before it was ever
observed, so the goal is "a plausible, road-following approach", not a
precise reconstruction.
"""
from __future__ import annotations

import math

from trace_fixer.geo.sync import apply_offset
from trace_fixer.geo.transform import EgoInterpolator, global_heading_to_zrot, global_to_ego_relative
from trace_fixer.models import EgoTrace, Trace, VehicleObs, VehicleTrack

DEFAULT_HORIZON_S = 4.0
DEFAULT_STEP_S = 0.2
STEER_BLEND = 0.35  # how strongly heading is pulled toward the local road tangent per step
MIN_FRAME_TO_PREDICT = 3  # tracks starting at/near frame 1 were visible from the recording start


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


def predict_backward(
    track: VehicleTrack,
    trace: Trace,
    horizon_s: float = DEFAULT_HORIZON_S,
    step_s: float = DEFAULT_STEP_S,
) -> int:
    """Prepends synthetic observations before the track's first real one.
    Returns the number of synthetic observations added.
    """
    obs = track.observations
    if not obs:
        return 0
    # Drop any previously-generated synthetic prefix before recomputing.
    real_obs = [o for o in obs if not o.synthetic]
    if not real_obs:
        return 0
    first = real_obs[0]
    if first.frame <= MIN_FRAME_TO_PREDICT:
        track.observations = real_obs
        return 0

    if len(real_obs) >= 2:
        second = real_obs[1]
        dt = (second.t_us - first.t_us) / 1e6
        vx = (second.x_m - first.x_m) / dt if dt > 0 else 0.0
        vy = (second.y_m - first.y_m) / dt if dt > 0 else 0.0
        speed = math.hypot(vx, vy)
        heading = math.atan2(vy, vx) if speed > 0.1 else math.radians(first.heading_deg)
    else:
        speed = 25.0  # fallback: typical highway speed, m/s
        heading = math.radians(first.heading_deg)

    interp = EgoInterpolator(trace.ego)
    ego_t0 = trace.ego.t0_us
    step_us = int(step_s * 1e6)

    x, y = first.x_m, first.y_m
    t_cursor = first.t_us - step_us
    elapsed = 0.0
    synthetic: list[VehicleObs] = []
    while elapsed < horizon_s and t_cursor > ego_t0:
        tangent = _nearest_path_tangent(trace.ego, x, y)
        heading = heading + STEER_BLEND * _wrap_rad(tangent - heading)
        x -= math.cos(heading) * speed * step_s
        y -= math.sin(heading) * speed * step_s

        t_ego_us = apply_offset(t_cursor, trace.sync_offset_us)
        ex, ey, eyaw, _ = interp.at(t_ego_us)
        x_rel, y_rel = global_to_ego_relative(x, y, ex, ey, eyaw)
        zrot = global_heading_to_zrot(heading, eyaw)

        synthetic.append(
            VehicleObs(
                t_us=t_cursor,
                frame=-1,
                obj_movement=first.obj_movement,
                obj_lane=first.obj_lane,
                obj_confidence="Predicted",
                x_rel=x_rel,
                y_rel=y_rel,
                z_rel=first.z_rel,
                length=first.length,
                width=first.width,
                height=first.height,
                zrot=zrot,
                x_m=x,
                y_m=y,
                heading_deg=math.degrees(heading) % 360,
                synthetic=True,
            )
        )
        t_cursor -= step_us
        elapsed += step_s

    synthetic.reverse()
    track.observations = synthetic + real_obs
    return len(synthetic)


def predict_all(trace: Trace, horizon_s: float = DEFAULT_HORIZON_S, step_s: float = DEFAULT_STEP_S) -> dict[int, int]:
    added = {}
    for track in trace.annotation.vehicles.values():
        n = predict_backward(track, trace, horizon_s=horizon_s, step_s=step_s)
        if n:
            added[track.obj_id] = n
    return added


def clear_predictions(trace: Trace) -> None:
    for track in trace.annotation.vehicles.values():
        track.observations = [o for o in track.observations if not o.synthetic]
