"""Trace-level behavioral summary: scene composition (object counts by
type), ego hard-braking events (AEB-like), and overtake events (a vehicle
passing the ego, or the ego passing a vehicle). Used by export.report to
produce a "what's in this trace" summary rather than only a list of data
quality issues.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from trace_fixer.models import Trace

# Real, intentional braking maneuvers, not annotation noise -- tuned against
# typical passenger-vehicle deceleration: comfortable braking is ~1-2 m/s^2,
# firm braking ~3-5 m/s^2, hard/AEB-level braking upwards of ~6 m/s^2 (~0.6g).
MODERATE_DECEL_MPS2 = 3.0
HARD_DECEL_MPS2 = 6.0
MIN_BRAKING_DURATION_S = 0.3

# An "overtake" requires the vehicle to end up clearly on the other side of
# x_rel=0 from where it started (not just noise wobbling around zero), and to
# stay within roughly two lane-widths laterally (otherwise it's not really
# passing alongside the ego -- e.g. a vehicle several lanes over).
OVERTAKE_LONGITUDINAL_MARGIN_M = 2.0
OVERTAKE_LATERAL_MAX_M = 8.0


@dataclass
class BrakingEvent:
    t_start_us: int
    t_end_us: int
    peak_decel_mps2: float  # negative
    severity: str  # "moderate" | "hard"


@dataclass
class OvertakeEvent:
    vehicle_id: int
    t_us: int
    direction: str  # "vehicle_overtakes_ego" | "ego_overtakes_vehicle"
    lateral_offset_m: float


@dataclass
class TraceSummary:
    object_counts: dict[str, int]
    braking_events: list[BrakingEvent]
    overtake_events: list[OvertakeEvent]


def count_objects_by_type(trace: Trace) -> dict[str, int]:
    counts: dict[str, int] = {}
    for track in trace.annotation.vehicles.values():
        counts[track.obj_type] = counts.get(track.obj_type, 0) + 1
    return counts


def detect_braking_events(trace: Trace) -> list[BrakingEvent]:
    poses = trace.ego.poses
    if len(poses) < 2:
        return []
    duration_s = (poses[-1].t_us - poses[0].t_us) / 1e6
    if duration_s <= 0:
        return []
    # ~10 Hz stride: same de-noising rationale as the ego kinematic
    # validation check -- raw ~100 Hz GPS/IMU quantization noise otherwise
    # amplifies into spurious accel spikes.
    stride = max(1, len(poses) // max(1, int(duration_s * 10)))
    sampled = poses[::stride]

    flagged: list[tuple[int, int, float]] = []
    for i in range(len(sampled) - 1):
        p0, p1 = sampled[i], sampled[i + 1]
        dt = (p1.t_us - p0.t_us) / 1e6
        if dt <= 0:
            continue
        v0 = math.hypot(p0.vx_mps, p0.vy_mps)
        v1 = math.hypot(p1.vx_mps, p1.vy_mps)
        accel = (v1 - v0) / dt
        if accel < -MODERATE_DECEL_MPS2:
            flagged.append((p0.t_us, p1.t_us, accel))

    if not flagged:
        return []

    merged: list[list] = [list(flagged[0])]
    for start, end, decel in flagged[1:]:
        last = merged[-1]
        if start <= last[1]:
            last[1] = max(last[1], end)
            last[2] = min(last[2], decel)  # more negative = stronger braking
        else:
            merged.append([start, end, decel])

    events = []
    for start, end, peak in merged:
        if (end - start) / 1e6 < MIN_BRAKING_DURATION_S:
            continue
        severity = "hard" if abs(peak) >= HARD_DECEL_MPS2 else "moderate"
        events.append(BrakingEvent(t_start_us=start, t_end_us=end, peak_decel_mps2=peak, severity=severity))
    return events


def detect_overtake_events(trace: Trace) -> list[OvertakeEvent]:
    """A crossing of x_rel=0 (ego-relative forward axis) counts as an
    overtake once the track is confidently on each side -- confidently
    meaning further than OVERTAKE_LONGITUDINAL_MARGIN_M from zero, not
    "the very next sampled point," since annotation keyframes are sparse
    and a genuine, smooth crossing can easily land its nearest neighbor
    within the margin on one side without that being noise.
    """
    events: list[OvertakeEvent] = []
    for track in trace.annotation.vehicles.values():
        obs = sorted(track.observations, key=lambda o: o.t_us)
        confident = [o for o in obs if abs(o.x_rel) >= OVERTAKE_LONGITUDINAL_MARGIN_M]
        for o0, o1 in zip(confident, confident[1:]):
            if (o0.x_rel > 0) == (o1.x_rel > 0):
                continue  # no sign change
            lateral = (o0.y_rel + o1.y_rel) / 2.0
            if abs(lateral) > OVERTAKE_LATERAL_MAX_M:
                continue  # not close enough alongside the ego to be "passing" it
            direction = "vehicle_overtakes_ego" if o0.x_rel < 0 < o1.x_rel else "ego_overtakes_vehicle"
            t_us = o0.t_us + (o1.t_us - o0.t_us) // 2
            events.append(
                OvertakeEvent(vehicle_id=track.obj_id, t_us=t_us, direction=direction, lateral_offset_m=lateral)
            )
    events.sort(key=lambda e: e.t_us)
    return events


def build_trace_summary(trace: Trace) -> TraceSummary:
    return TraceSummary(
        object_counts=count_objects_by_type(trace),
        braking_events=detect_braking_events(trace),
        overtake_events=detect_overtake_events(trace),
    )
