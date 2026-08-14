"""Rule-based validation: kinematic feasibility, inter-vehicle collisions
(including ego), and off-road / lane-boundary checks.

These are deliberately simple, explainable physics/geometry checks rather
than a learned model -- there's no labeled "this annotation is wrong"
training data available, and explainable rules are what an annotation QA
team can actually act on.
"""
from __future__ import annotations

import math

from trace_fixer.geo.collision import obb_overlap
from trace_fixer.geo.road_corridor import corridor_bounds
from trace_fixer.geo.transform import EgoInterpolator
from trace_fixer.models import Issue, Trace, VehicleTrack

# -- thresholds (highway-driving heuristics; tune per fleet/ODD if needed) --
SPEED_MEDIUM_MPS = 45.0  # ~162 km/h
SPEED_HIGH_MPS = 60.0  # ~216 km/h
ACCEL_MEDIUM_MPS2 = 6.0
ACCEL_HIGH_MPS2 = 12.0
YAW_RATE_MEDIUM_DEG_S = 15.0
YAW_RATE_HIGH_DEG_S = 40.0
OFFROAD_MARGIN_M = 0.3

EGO_LENGTH_M = 4.9
EGO_WIDTH_M = 1.9


def _wrap180(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0


def _next_id(counter: list[int]) -> str:
    counter[0] += 1
    return f"issue-{counter[0]}"


def _kinematic_checks(track: VehicleTrack, counter: list[int]) -> list[Issue]:
    issues: list[Issue] = []
    obs = track.observations
    speeds: list[float] = [0.0] * len(obs)
    for i in range(len(obs) - 1):
        o0, o1 = obs[i], obs[i + 1]
        dt = (o1.t_us - o0.t_us) / 1e6
        if dt <= 0:
            continue
        dist = math.hypot(o1.x_m - o0.x_m, o1.y_m - o0.y_m)
        speed = dist / dt
        speeds[i + 1] = speed
        severity = None
        if speed > SPEED_HIGH_MPS:
            severity = "high"
        elif speed > SPEED_MEDIUM_MPS:
            severity = "medium"
        if severity:
            issues.append(
                Issue(
                    issue_id=_next_id(counter),
                    category="kinematic",
                    severity=severity,
                    vehicle_id=track.obj_id,
                    t_start_us=o0.t_us,
                    t_end_us=o1.t_us,
                    description=(
                        f"Vehicle {track.obj_id} implied speed {speed:.1f} m/s "
                        f"({speed * 3.6:.0f} km/h) between frames {o0.frame} and {o1.frame} "
                        "is unrealistic for road traffic."
                    ),
                )
            )
        yaw_rate = abs(_wrap180(o1.heading_deg - o0.heading_deg)) / dt
        severity = None
        if yaw_rate > YAW_RATE_HIGH_DEG_S:
            severity = "high"
        elif yaw_rate > YAW_RATE_MEDIUM_DEG_S:
            severity = "medium"
        if severity:
            issues.append(
                Issue(
                    issue_id=_next_id(counter),
                    category="kinematic",
                    severity=severity,
                    vehicle_id=track.obj_id,
                    t_start_us=o0.t_us,
                    t_end_us=o1.t_us,
                    description=(
                        f"Vehicle {track.obj_id} implied yaw rate {yaw_rate:.1f} deg/s "
                        f"between frames {o0.frame} and {o1.frame} is unrealistic."
                    ),
                )
            )

    for i in range(1, len(obs) - 1):
        dt0 = (obs[i].t_us - obs[i - 1].t_us) / 1e6
        dt1 = (obs[i + 1].t_us - obs[i].t_us) / 1e6
        if dt0 <= 0 or dt1 <= 0:
            continue
        accel = (speeds[i + 1] - speeds[i]) / dt1 if i + 1 < len(speeds) else 0.0
        severity = None
        if abs(accel) > ACCEL_HIGH_MPS2:
            severity = "high"
        elif abs(accel) > ACCEL_MEDIUM_MPS2:
            severity = "medium"
        if severity:
            issues.append(
                Issue(
                    issue_id=_next_id(counter),
                    category="kinematic",
                    severity=severity,
                    vehicle_id=track.obj_id,
                    t_start_us=obs[i].t_us,
                    t_end_us=obs[i + 1].t_us,
                    description=(
                        f"Vehicle {track.obj_id} implied longitudinal acceleration "
                        f"{accel:.1f} m/s^2 near frame {obs[i].frame} is unrealistic."
                    ),
                )
            )
    return issues


def _merge_intervals(flags: list[tuple[int, int, str]]) -> list[tuple[int, int, str]]:
    """Merge adjacent (t_start, t_end, note) tuples that touch/overlap."""
    if not flags:
        return []
    flags = sorted(flags, key=lambda f: f[0])
    merged = [flags[0]]
    for start, end, note in flags[1:]:
        ls, le, lnote = merged[-1]
        if start <= le:
            merged[-1] = (ls, max(le, end), lnote)
        else:
            merged.append((start, end, note))
    return merged


def _collision_checks(trace: Trace, counter: list[int]) -> list[Issue]:
    issues: list[Issue] = []
    tracks = list(trace.annotation.vehicles.values())
    ego_interp = EgoInterpolator(trace.ego)

    def ego_box_at(t_us: int) -> tuple[float, float, float]:
        x, y, yaw, _ = ego_interp.at(t_us)
        return x, y, math.degrees(yaw)

    # vehicle vs ego
    for track in tracks:
        flags: list[tuple[int, int, str]] = []
        for obs in track.observations:
            ex, ey, eh = ego_box_at(obs.t_us + trace.sync_offset_us)
            if obb_overlap(obs.x_m, obs.y_m, obs.heading_deg, obs.length, obs.width, ex, ey, eh, EGO_LENGTH_M, EGO_WIDTH_M):
                flags.append((obs.t_us, obs.t_us, ""))
        for start, end, _ in _merge_intervals(flags):
            issues.append(
                Issue(
                    issue_id=_next_id(counter),
                    category="collision",
                    severity="high",
                    vehicle_id=track.obj_id,
                    t_start_us=start,
                    t_end_us=end,
                    description=f"Vehicle {track.obj_id} bounding box overlaps the ego vehicle.",
                )
            )

    # vehicle vs vehicle
    for i in range(len(tracks)):
        for j in range(i + 1, len(tracks)):
            a, b = tracks[i], tracks[j]
            if not a.observations or not b.observations:
                continue
            b_by_t = {o.t_us: o for o in b.observations}
            b_times = sorted(b_by_t)
            flags = []
            for oa in a.observations:
                ob = _nearest(b_times, b_by_t, oa.t_us, max_dt_us=300_000)
                if ob is None:
                    continue
                if obb_overlap(oa.x_m, oa.y_m, oa.heading_deg, oa.length, oa.width, ob.x_m, ob.y_m, ob.heading_deg, ob.length, ob.width):
                    flags.append((oa.t_us, oa.t_us, ""))
            for start, end, _ in _merge_intervals(flags):
                issues.append(
                    Issue(
                        issue_id=_next_id(counter),
                        category="collision",
                        severity="high",
                        vehicle_id=a.obj_id,
                        t_start_us=start,
                        t_end_us=end,
                        description=f"Vehicle {a.obj_id} bounding box overlaps vehicle {b.obj_id}.",
                    )
                )
    return issues


def _nearest(times: list[int], by_t: dict, t_us: int, max_dt_us: int):
    if not times:
        return None
    import bisect

    i = bisect.bisect_left(times, t_us)
    candidates = [t for t in (times[i - 1] if i > 0 else None, times[i] if i < len(times) else None) if t is not None]
    if not candidates:
        return None
    best = min(candidates, key=lambda t: abs(t - t_us))
    if abs(best - t_us) > max_dt_us:
        return None
    return by_t[best]


def _offroad_checks(trace: Trace, counter: list[int]) -> list[Issue]:
    issues: list[Issue] = []
    for track in trace.annotation.vehicles.values():
        flags: list[tuple[int, int, str]] = []
        for obs in track.observations:
            left, right = corridor_bounds(trace.annotation.border_lines, obs.t_us, obs.x_rel)
            half_w = obs.width / 2.0
            if left is not None and obs.y_rel - half_w > left - OFFROAD_MARGIN_M:
                flags.append((obs.t_us, obs.t_us, "left"))
            elif right is not None and obs.y_rel + half_w < right + OFFROAD_MARGIN_M:
                flags.append((obs.t_us, obs.t_us, "right"))
        for start, end, side in _merge_intervals(flags):
            issues.append(
                Issue(
                    issue_id=_next_id(counter),
                    category="off_road",
                    severity="medium",
                    vehicle_id=track.obj_id,
                    t_start_us=start,
                    t_end_us=end,
                    description=f"Vehicle {track.obj_id} crosses the {side} road edge/guardrail boundary.",
                )
            )
    return issues


def _ego_kinematic_checks(trace: Trace, counter: list[int]) -> list[Issue]:
    """Flags implausible jumps in the ADMA reference trace itself. Flag-only:
    the ego trace is the foundation everything else is built on, so unlike
    vehicle tracks it is not auto-smoothed -- a human should confirm before
    the reference trajectory itself is edited.
    """
    issues: list[Issue] = []
    poses = trace.ego.poses
    # ADMA logs at ~100 Hz; evaluating every single sample amplifies GPS/IMU
    # quantization noise into spurious accel spikes, so stride to a coarser,
    # still-plenty-sensitive ~10 Hz cadence.
    stride = max(1, len(poses) // max(1, int((trace.ego.t1_us - trace.ego.t0_us) / 1e6 * 10)))
    sampled = poses[::stride]
    for i in range(len(sampled) - 1):
        p0, p1 = sampled[i], sampled[i + 1]
        dt = (p1.t_us - p0.t_us) / 1e6
        if dt <= 0:
            continue
        dist = math.hypot(p1.x_m - p0.x_m, p1.y_m - p0.y_m)
        speed = dist / dt
        v0 = math.hypot(p0.vx_mps, p0.vy_mps)
        accel = (math.hypot(p1.vx_mps, p1.vy_mps) - v0) / dt
        severity = None
        if speed > SPEED_HIGH_MPS or abs(accel) > ACCEL_HIGH_MPS2:
            severity = "high"
        elif speed > SPEED_MEDIUM_MPS or abs(accel) > ACCEL_MEDIUM_MPS2:
            severity = "medium"
        if severity:
            issues.append(
                Issue(
                    issue_id=_next_id(counter),
                    category="kinematic",
                    severity=severity,
                    vehicle_id=None,
                    t_start_us=p0.t_us,
                    t_end_us=p1.t_us,
                    description=(
                        f"Ego (ADMA) implied speed {speed:.1f} m/s / accel {accel:.1f} m/s^2 "
                        f"near t={p0.t_us / 1e6:.2f}s is unrealistic."
                    ),
                    fixable=False,
                )
            )
    return issues


def run_validation(trace: Trace) -> list[Issue]:
    counter = [0]
    issues: list[Issue] = []
    issues.extend(_ego_kinematic_checks(trace, counter))
    for track in trace.annotation.vehicles.values():
        issues.extend(_kinematic_checks(track, counter))
    issues.extend(_collision_checks(trace, counter))
    issues.extend(_offroad_checks(trace, counter))
    issues.sort(key=lambda i: (i.t_start_us, i.vehicle_id or 0))
    trace.issues = issues
    return issues
