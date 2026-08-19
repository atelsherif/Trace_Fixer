"""Trace-level behavioral summary: scene composition (moving-object and
static-object counts by type), ego hard-braking events (AEB-like), and
overtake events (a vehicle passing the ego, or the ego passing a vehicle).
Used by export.report to produce a "what's in this trace" summary rather
than only a list of data quality issues.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from trace_fixer.geo.transform import EgoInterpolator
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

# Same-lane test used by both short-headway and cut-in detection: half a
# typical lane width either side of the ego's forward axis.
LANE_HALF_WIDTH_M = 1.9

# A gap under ~1s to the vehicle directly ahead is the standard "too close"
# threshold in ADAS/AEB literature; below ~0.5s it's a near-miss rather than
# just tailgating. Speeds below MIN_SPEED_FOR_HEADWAY_MPS are excluded since
# headway (distance / speed) blows up and becomes meaningless near a stop.
SHORT_HEADWAY_S = 1.0
NEAR_MISS_HEADWAY_S = 0.5
MIN_SPEED_FOR_HEADWAY_MPS = 1.0
MIN_HEADWAY_DURATION_S = 0.3

# A "cut-in": a vehicle that was outside the ego's lane crosses into it
# (|y_rel| drops under LANE_HALF_WIDTH_M) while already close ahead.
CUT_IN_RANGE_M = 25.0

# Ego considered stopped/crawling below this speed; a standstill needs to
# last a few seconds to count as a real stop rather than a brief slow-down.
STANDSTILL_MPS = 0.5
MIN_STANDSTILL_DURATION_S = 3.0

# A sustained heading change bigger than this within one window is a turn,
# not steering noise -- tuned well above what a lane-keeping wobble produces.
SHARP_TURN_DEG = 45.0
SHARP_TURN_WINDOW_S = 3.0


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
class ShortHeadwayEvent:
    vehicle_id: int
    t_start_us: int
    t_end_us: int
    min_headway_s: float
    near_miss: bool  # min_headway_s < NEAR_MISS_HEADWAY_S


@dataclass
class CutInEvent:
    vehicle_id: int
    t_us: int
    range_m: float  # forward distance at the moment it entered the ego's lane


@dataclass
class StandstillEvent:
    t_start_us: int
    t_end_us: int


@dataclass
class SharpTurnEvent:
    t_start_us: int
    t_end_us: int
    heading_change_deg: float  # signed


@dataclass
class TraceSummary:
    object_counts: dict[str, int]
    static_object_counts: dict[str, int]
    braking_events: list[BrakingEvent]
    overtake_events: list[OvertakeEvent]
    short_headway_events: list[ShortHeadwayEvent]
    cut_in_events: list[CutInEvent]
    standstill_events: list[StandstillEvent]
    sharp_turn_events: list[SharpTurnEvent]


def count_objects_by_type(trace: Trace) -> dict[str, int]:
    counts: dict[str, int] = {}
    for track in trace.annotation.vehicles.values():
        counts[track.obj_type] = counts.get(track.obj_type, 0) + 1
    return counts


def count_static_objects_by_type(trace: Trace) -> dict[str, int]:
    counts: dict[str, int] = {}
    for static_obj in trace.annotation.static_objects.values():
        counts[static_obj.obj_type] = counts.get(static_obj.obj_type, 0) + 1
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


def detect_short_headway_events(trace: Trace) -> list[ShortHeadwayEvent]:
    """A vehicle ahead in roughly the ego's own lane, closer than
    SHORT_HEADWAY_S seconds at the ego's current speed -- annotation
    keyframes are sparse, so this is evaluated observation-by-observation
    (not resampled) and adjacent flagged observations for the same vehicle
    are merged into one event, same as detect_braking_events.
    """
    if len(trace.ego.poses) < 2:
        return []
    interp = EgoInterpolator(trace.ego)
    events: list[ShortHeadwayEvent] = []
    for track in trace.annotation.vehicles.values():
        flagged: list[tuple[int, float]] = []
        for obs in sorted(track.observations, key=lambda o: o.t_us):
            if obs.x_rel <= 0 or abs(obs.y_rel) > LANE_HALF_WIDTH_M:
                continue
            _, _, _, speed = interp.at(obs.t_us)
            if speed < MIN_SPEED_FOR_HEADWAY_MPS:
                continue
            headway_s = obs.x_rel / speed
            if headway_s < SHORT_HEADWAY_S:
                flagged.append((obs.t_us, headway_s))

        if not flagged:
            continue
        merged: list[list] = [[flagged[0][0], flagged[0][0], flagged[0][1]]]
        for t_us, headway_s in flagged[1:]:
            last = merged[-1]
            last[1] = t_us
            last[2] = min(last[2], headway_s)
        for start, end, min_headway in merged:
            if (end - start) / 1e6 < MIN_HEADWAY_DURATION_S:
                continue
            events.append(
                ShortHeadwayEvent(
                    vehicle_id=track.obj_id,
                    t_start_us=start,
                    t_end_us=end,
                    min_headway_s=min_headway,
                    near_miss=min_headway < NEAR_MISS_HEADWAY_S,
                )
            )
    events.sort(key=lambda e: e.t_start_us)
    return events


def detect_cut_in_events(trace: Trace) -> list[CutInEvent]:
    """A vehicle that was outside the ego's lane crosses into it
    (|y_rel| drops under LANE_HALF_WIDTH_M) while already close ahead --
    distinct from an overtake, which crosses the ego's forward axis
    (x_rel=0) rather than merging into the lane ahead of it.
    """
    events: list[CutInEvent] = []
    for track in trace.annotation.vehicles.values():
        obs = sorted(track.observations, key=lambda o: o.t_us)
        for o0, o1 in zip(obs, obs[1:]):
            was_outside = abs(o0.y_rel) > LANE_HALF_WIDTH_M
            now_inside = abs(o1.y_rel) <= LANE_HALF_WIDTH_M
            if was_outside and now_inside and 0 < o1.x_rel <= CUT_IN_RANGE_M:
                events.append(CutInEvent(vehicle_id=track.obj_id, t_us=o1.t_us, range_m=o1.x_rel))
    events.sort(key=lambda e: e.t_us)
    return events


def detect_standstill_events(trace: Trace) -> list[StandstillEvent]:
    poses = trace.ego.poses
    if len(poses) < 2:
        return []
    flagged: list[tuple[int, int]] = []
    for p in poses:
        speed = math.hypot(p.vx_mps, p.vy_mps)
        if speed < STANDSTILL_MPS:
            flagged.append((p.t_us, p.t_us))

    if not flagged:
        return []
    merged: list[list] = [list(flagged[0])]
    for start, end in flagged[1:]:
        last = merged[-1]
        if start <= last[1] + 1_000_000:  # tolerate up to 1s gaps between flagged samples
            last[1] = max(last[1], end)
        else:
            merged.append([start, end])

    return [
        StandstillEvent(t_start_us=start, t_end_us=end)
        for start, end in merged
        if (end - start) / 1e6 >= MIN_STANDSTILL_DURATION_S
    ]


def detect_sharp_turn_events(trace: Trace) -> list[SharpTurnEvent]:
    """Sliding-window heading change over SHARP_TURN_WINDOW_S; overlapping
    flagged windows are merged into a single event spanning the maneuver,
    keeping the largest heading change seen.
    """
    poses = trace.ego.poses
    if len(poses) < 2:
        return []

    def wrap_deg(a: float) -> float:
        return (a + 180.0) % 360.0 - 180.0

    window_us = int(SHARP_TURN_WINDOW_S * 1e6)
    flagged: list[tuple[int, int, float]] = []
    j = 0
    for i, p0 in enumerate(poses):
        j = max(j, i)
        while j + 1 < len(poses) and poses[j + 1].t_us - p0.t_us <= window_us:
            j += 1
        if j == i:
            continue
        p1 = poses[j]
        delta = wrap_deg(p1.heading_deg - p0.heading_deg)
        if abs(delta) >= SHARP_TURN_DEG:
            flagged.append((p0.t_us, p1.t_us, delta))

    if not flagged:
        return []
    merged: list[list] = [list(flagged[0])]
    for start, end, delta in flagged[1:]:
        last = merged[-1]
        if start <= last[1]:
            last[1] = max(last[1], end)
            if abs(delta) > abs(last[2]):
                last[2] = delta
        else:
            merged.append([start, end, delta])

    return [SharpTurnEvent(t_start_us=start, t_end_us=end, heading_change_deg=delta) for start, end, delta in merged]


def build_trace_summary(trace: Trace) -> TraceSummary:
    return TraceSummary(
        object_counts=count_objects_by_type(trace),
        static_object_counts=count_static_objects_by_type(trace),
        braking_events=detect_braking_events(trace),
        overtake_events=detect_overtake_events(trace),
        short_headway_events=detect_short_headway_events(trace),
        cut_in_events=detect_cut_in_events(trace),
        standstill_events=detect_standstill_events(trace),
        sharp_turn_events=detect_sharp_turn_events(trace),
    )
