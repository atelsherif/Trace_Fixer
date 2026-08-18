"""Reconstructs road geometry -- a reference polyline with real curvature,
per-section lane count/width, and static-object placements -- from a
trace's own ego trajectory and annotation data. Feeds export/opendrive.py;
see that module's docstring for the overall scope and what this
deliberately doesn't attempt (a full HD map).

Two things drive this design:

  - The reference line always follows the ego's own recorded path, using
    its *real* IMU/GPS heading (not a heading derived from chord direction
    between sparse samples) so the plan-view geometry can be emitted as
    curvature-continuous arcs instead of straight-line segments. This has
    no dependency on annotation quality -- it's the same trusted ego trace
    used throughout the rest of the tool.

  - Lane count/width, in contrast, comes from annotation data (lane-marking
    polylines, cross-checked against vehicles' obj_lane labels), which is
    sparser and can be noisy or gappy. Every windowed estimate is sanity-
    checked (lane count bounds, width bounds, minimum sample support,
    obj_lane agreement) before use; a window that fails any check falls
    back to the trace's overall majority lane count and a constant default
    width. This matters: a jaggedly-wrong road (an unbounded width spike
    from one bad frame) is worse for a downstream simulator/planner than a
    smoothly-wrong one (constant width everywhere), so a low-confidence
    window should degrade to the old constant-width behavior rather than
    emit whatever the noisy estimate says.
"""
from __future__ import annotations

import bisect
import math
from dataclasses import dataclass, field

from trace_fixer.geo.transform import global_to_ego_relative, heading_to_yaw_rad
from trace_fixer.models import Trace

MIN_SEGMENT_LEN_M = 5.0

# Real lane widths worldwide are roughly 2.5-4.5m; a "lane" estimated
# outside this range is almost certainly a clustering artifact (a
# shoulder, a gap in markings, noise), not a real lane.
MIN_LANE_WIDTH_M = 2.5
MAX_LANE_WIDTH_M = 4.5
DEFAULT_LANE_WIDTH_M = 3.5
MIN_LANES = 1
MAX_LANES = 6

# Local marking points closer together (laterally) than this are treated as
# the same physical boundary; further apart, a different one. Comfortably
# smaller than MIN_LANE_WIDTH_M so two adjacent real boundaries never merge.
BOUNDARY_CLUSTER_GAP_M = 1.5
# Points further than this from the reference line are excluded before
# clustering -- a basic outlier guard against unrelated markings (opposite
# carriageway, a parallel service road) rather than a claim about where the
# ego sits within its own lane (it does *not* reliably sit at the edge --
# see estimate_lane_sections' bracketing logic).
MAX_LATERAL_CORRIDOR_M = 20.0

WINDOW_LEN_M = 30.0  # along-road window for estimating local lane geometry
MIN_MARKING_POINTS_PER_WINDOW = 20
# A lane-count/width change must persist for this many consecutive windows
# before it's accepted as real, so one noisy window can't fragment the road
# into a flickering sequence of spurious laneSections.
MIN_PERSISTENCE_WINDOWS = 2

# obj_lane label -> how many lanes to the right of the ego's own lane
# (0 = the ego's own lane). Only right-of-ego labels are meaningful here
# since this model doesn't represent oncoming/left carriageways.
_OBJ_LANE_INDEX = {"EGO lane": 0, "1st Right": 1, "2nd Right": 2, "3rd Right": 3}

# Best-effort mapping from the annotation's free-text static-object type to
# a conservative, well-established subset of the ASAM OpenDRIVE e_objectType
# enum. This is approximate -- there's no dedicated ASAM category for some
# of these (e.g. a reflective delineator post) -- so the original label is
# always preserved verbatim in the emitted object's `name` attribute too.
STATIC_OBJECT_TYPE_MAP = {
    "Traffic Sign": "pole",
    "Highly Reflective Marker": "pole",
    "Highway Accessories": "none",
}
DEFAULT_STATIC_OBJECT_TYPE = "none"


def _wrap_rad(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


@dataclass
class RefPoint:
    s: float
    x: float
    y: float
    heading_rad: float  # real IMU/GPS-derived heading at this point


@dataclass
class LaneSectionPlan:
    s_start: float
    num_lanes: int
    lane_widths_m: list[float]  # one per lane, index 0 = the lane nearest the reference line
    source: str  # "annotation" | "default" -- for transparency/debugging


@dataclass
class StaticObjectPlacement:
    obj_id: int
    label: str  # original annotation obj_type, e.g. "Traffic Sign"
    odr_type: str  # best-effort ASAM e_objectType
    s: float
    t: float
    length: float
    width: float
    height: float


@dataclass
class RoadGeometryPlan:
    ref_points: list[RefPoint]
    total_length: float
    lane_sections: list[LaneSectionPlan]
    static_objects: list[StaticObjectPlacement] = field(default_factory=list)


def build_reference_polyline(trace: Trace, min_segment_len_m: float = MIN_SEGMENT_LEN_M) -> list[RefPoint]:
    """Downsamples the ego path for plan-view emission, but accumulates `s`
    from *every* consecutive pose (not just the sampled ones) so it's a
    true arc-length coordinate -- needed for the emitted arc lengths to be
    self-consistent (see opendrive.py) and for lane-marking/object
    projection to land at the right place along the road.
    """
    poses = trace.ego.poses
    if not poses:
        return []
    ref = [RefPoint(s=0.0, x=poses[0].x_m, y=poses[0].y_m, heading_rad=heading_to_yaw_rad(poses[0].heading_deg))]
    s = 0.0
    s_at_last_append = 0.0
    for i in range(1, len(poses)):
        p0, p1 = poses[i - 1], poses[i]
        s += math.hypot(p1.x_m - p0.x_m, p1.y_m - p0.y_m)
        is_last = i == len(poses) - 1
        if is_last or (s - s_at_last_append) >= min_segment_len_m:
            ref.append(RefPoint(s=s, x=p1.x_m, y=p1.y_m, heading_rad=heading_to_yaw_rad(p1.heading_deg)))
            s_at_last_append = s
    return ref


def project_to_road(ref_points: list[RefPoint], x: float, y: float) -> tuple[float, float]:
    """World (x, y) -> road (s, t): nearest reference point by straight-line
    distance, then a local ego-relative rotation at that point for the
    within-segment correction. t follows OpenDRIVE convention (positive =
    left of the reference line's direction of travel), which is exactly
    the existing ego-relative "left = +y" convention used throughout the
    codebase for annotation coordinates.
    """
    best_i, best_d2 = 0, math.inf
    for i, r in enumerate(ref_points):
        d2 = (r.x - x) ** 2 + (r.y - y) ** 2
        if d2 < best_d2:
            best_d2, best_i = d2, i
    r = ref_points[best_i]
    x_rel, y_rel = global_to_ego_relative(x, y, r.x, r.y, r.heading_rad)
    return r.s + x_rel, y_rel


def _majority_num_lanes(trace: Trace, default: int = 2) -> int:
    counts: dict[int, int] = {}
    for m in trace.annotation.frame_meta:
        if m.num_lanes:
            counts[m.num_lanes] = counts.get(m.num_lanes, 0) + 1
    if not counts:
        return default
    return max(counts, key=lambda k: counts[k])


def _cluster_boundaries(t_values: list[float]) -> list[float]:
    """Gap-based 1D clustering: sort, then split wherever consecutive
    values are further apart than BOUNDARY_CLUSTER_GAP_M. Returns cluster
    medians -- candidate lane-boundary offsets, in no particular relation
    to t=0 (the ego does *not* reliably drive at its lane's edge; see
    estimate_lane_sections for how the ego's own lane is identified among
    these).
    """
    candidates = sorted(v for v in t_values if abs(v) <= MAX_LATERAL_CORRIDOR_M)
    clusters: list[list[float]] = []
    for v in candidates:
        if clusters and v - clusters[-1][-1] <= BOUNDARY_CLUSTER_GAP_M:
            clusters[-1].append(v)
        else:
            clusters.append([v])
    return [c[len(c) // 2] for c in clusters]


def _default_section(s_start: float, trace: Trace) -> LaneSectionPlan:
    n = _majority_num_lanes(trace)
    return LaneSectionPlan(s_start=s_start, num_lanes=n, lane_widths_m=[DEFAULT_LANE_WIDTH_M] * n, source="default")


def _estimate_window(
    window_marking_t: list[float],
    window_vehicles: list[tuple[float, str]],
    declared_num_lanes: int | None,
) -> LaneSectionPlan | None:
    """One window's lane geometry from marking clusters, cross-checked
    against obj_lane labels and (when available) the annotation's own
    directly-authored frame_meta.num_lanes for that stretch -- a stronger
    signal than anything derived from marking geometry, since it isn't
    subject to the same occlusion/tracking noise. Returns None (caller
    falls back to default) if any sanity check fails.
    """
    if len(window_marking_t) < MIN_MARKING_POINTS_PER_WINDOW:
        return None
    bounds = _cluster_boundaries(window_marking_t)

    # Find where the ego's own path (t=0) sits among the detected
    # boundaries -- the two that bracket it are the ego's own lane's
    # edges. This model only covers the ego's carriageway (lanes at or to
    # the right of the ego), so only the right-side boundaries matter.
    pivot = None
    for i in range(len(bounds) - 1):
        if bounds[i] <= 0 <= bounds[i + 1]:
            pivot = i
            break
    if pivot is None:
        return None  # t=0 isn't bracketed by any detected pair -- not enough reliable support

    right_side = bounds[: pivot + 2]  # farthest-right boundary ... ego's own left edge, inclusive
    num_lanes = len(right_side) - 1
    if not (MIN_LANES <= num_lanes <= MAX_LANES):
        return None
    if declared_num_lanes is not None and num_lanes != declared_num_lanes:
        return None
    widths = [right_side[i + 1] - right_side[i] for i in range(num_lanes)]
    # right_side is ascending (rightmost first); reverse so index 0 = the
    # ego's own lane, matching LaneSectionPlan's contract.
    widths = list(reversed(widths))
    if any(not (MIN_LANE_WIDTH_M <= w <= MAX_LANE_WIDTH_M) for w in widths):
        return None

    lane_edges = [right_side[-1]]  # the ego's own lane's left edge, not necessarily 0
    for w in widths:
        lane_edges.append(lane_edges[-1] - w)
    mismatches, checked = 0, 0
    for t_obs, obj_lane in window_vehicles:
        idx = _OBJ_LANE_INDEX.get(obj_lane)
        if idx is None or idx >= num_lanes:
            continue
        checked += 1
        lo, hi = lane_edges[idx + 1] - 1.0, lane_edges[idx] + 1.0  # 1m slack either side
        if not (lo <= t_obs <= hi):
            mismatches += 1
    if checked >= 3 and mismatches / checked > 0.3:
        return None

    return LaneSectionPlan(s_start=0.0, num_lanes=num_lanes, lane_widths_m=widths, source="annotation")


def _frame_meta_num_lanes_by_window(trace: Trace, ref_points: list[RefPoint]) -> dict[int, int]:
    """Maps each frame_meta entry (sparse, but directly authored -- not
    derived) to a window via its nearest ego pose, for the num_lanes
    cross-check in _estimate_window. A window with multiple entries takes
    their majority.
    """
    poses = trace.ego.poses
    if not poses:
        return {}
    times = [p.t_us for p in poses]
    votes: dict[int, dict[int, int]] = {}
    for fm in trace.annotation.frame_meta:
        if not fm.num_lanes:
            continue
        idx = min(bisect.bisect_left(times, fm.t_us), len(poses) - 1)
        p = poses[idx]
        s, _ = project_to_road(ref_points, p.x_m, p.y_m)
        w = int(s // WINDOW_LEN_M)
        votes.setdefault(w, {}).setdefault(fm.num_lanes, 0)
        votes[w][fm.num_lanes] += 1
    return {w: max(counts, key=lambda k: counts[k]) for w, counts in votes.items()}


def estimate_lane_sections(trace: Trace, ref_points: list[RefPoint]) -> list[LaneSectionPlan]:
    if not ref_points:
        return [_default_section(0.0, trace)]
    total_len = ref_points[-1].s

    marking_by_window: dict[int, list[float]] = {}
    for lm in trace.annotation.lane_markings.values():
        for snap in lm.snapshots:
            for gx, gy in snap.points_m:
                s, t = project_to_road(ref_points, gx, gy)
                marking_by_window.setdefault(int(s // WINDOW_LEN_M), []).append(t)

    vehicles_by_window: dict[int, list[tuple[float, str]]] = {}
    for track in trace.annotation.vehicles.values():
        for obs in track.observations:
            s, t = project_to_road(ref_points, obs.x_m, obs.y_m)
            vehicles_by_window.setdefault(int(s // WINDOW_LEN_M), []).append((t, obs.obj_lane))

    declared_by_window = _frame_meta_num_lanes_by_window(trace, ref_points)

    n_windows = max(1, math.ceil(total_len / WINDOW_LEN_M))
    raw: list[LaneSectionPlan] = []
    for w in range(n_windows):
        plan = _estimate_window(marking_by_window.get(w, []), vehicles_by_window.get(w, []), declared_by_window.get(w))
        raw.append(plan or _default_section(w * WINDOW_LEN_M, trace))
        raw[-1].s_start = w * WINDOW_LEN_M

    # Persistence filter: a window that differs from both neighbors and
    # doesn't repeat for MIN_PERSISTENCE_WINDOWS is noise -- flatten it to
    # match its surroundings rather than let it fragment the road.
    def same(a: LaneSectionPlan, b: LaneSectionPlan) -> bool:
        return a.num_lanes == b.num_lanes and all(abs(x - y) < 0.3 for x, y in zip(a.lane_widths_m, b.lane_widths_m))

    i = 0
    while i < len(raw):
        j = i
        while j + 1 < len(raw) and same(raw[j + 1], raw[i]):
            j += 1
        if j - i + 1 < MIN_PERSISTENCE_WINDOWS and i > 0:
            for k in range(i, j + 1):
                raw[k] = LaneSectionPlan(raw[k].s_start, raw[i - 1].num_lanes, raw[i - 1].lane_widths_m, raw[i - 1].source)
        i = j + 1

    # Merge consecutive identical windows into single sections.
    sections: list[LaneSectionPlan] = [raw[0]]
    for plan in raw[1:]:
        if same(plan, sections[-1]):
            continue
        sections.append(plan)
    return sections


def aggregate_static_objects(trace: Trace, ref_points: list[RefPoint]) -> list[StaticObjectPlacement]:
    """One placement per annotated static object, from a single
    representative observation (the middle one by time order) rather than
    per-field medians across all of them -- since the object is static,
    any one observation's position/dimensions are as good as another, and
    picking one keeps position and size self-consistent instead of
    stitching them together from independently-sorted arrays.
    """
    if not ref_points:
        return []
    placements = []
    for so in trace.annotation.static_objects.values():
        if not so.observations:
            continue
        obs = so.observations[len(so.observations) // 2]
        s, t = project_to_road(ref_points, obs.x_m, obs.y_m)
        odr_type = STATIC_OBJECT_TYPE_MAP.get(so.obj_type, DEFAULT_STATIC_OBJECT_TYPE)
        placements.append(
            StaticObjectPlacement(
                obj_id=so.obj_id,
                label=so.obj_type,
                odr_type=odr_type,
                s=s,
                t=t,
                length=obs.length,
                width=obs.width,
                height=obs.height,
            )
        )
    return placements


def build_road_geometry_plan(trace: Trace) -> RoadGeometryPlan:
    ref_points = build_reference_polyline(trace)
    if len(ref_points) < 2:
        raise ValueError("Ego path too short to build a road")
    return RoadGeometryPlan(
        ref_points=ref_points,
        total_length=ref_points[-1].s,
        lane_sections=estimate_lane_sections(trace, ref_points),
        static_objects=aggregate_static_objects(trace, ref_points),
    )
