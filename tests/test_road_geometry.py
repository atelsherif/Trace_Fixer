"""Tests for trace_fixer.export.road_geometry: the reference polyline,
world<->road projection, and robust lane-section estimation (with fallback
to a constant-width default when annotation data doesn't pass sanity
checks) that feeds export/opendrive.py.
"""
import math
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
SAMPLE1_DIR = REPO_ROOT / "data" / "traces" / "sample1"
SAMPLE2_DIR = REPO_ROOT / "data" / "traces" / "sample2"


@pytest.fixture()
def sample1():
    from trace_fixer.scene import load_trace

    return load_trace("sample1", SAMPLE1_DIR / "adma.csv", SAMPLE1_DIR / "annotation.xml")


@pytest.fixture()
def sample2():
    from trace_fixer.scene import load_trace

    return load_trace("sample2", SAMPLE2_DIR / "adma.csv", SAMPLE2_DIR / "annotation.xml")


def test_reference_polyline_uses_true_arc_length_not_chord_length(sample1):
    """A polyline through a curve has true arc length >= chord length; s
    must track the former (see module docstring on why) or emitted arc
    segments and lane-marking/object projections would be inconsistent.
    """
    from trace_fixer.export.road_geometry import build_reference_polyline

    ref = build_reference_polyline(sample1)
    assert len(ref) >= 2
    assert ref[0].s == 0.0
    # s is non-decreasing and strictly increasing between distinct points
    for a, b in zip(ref, ref[1:]):
        assert b.s > a.s
        chord = math.hypot(b.x - a.x, b.y - a.y)
        assert b.s - a.s >= chord - 1e-6  # true arc length can't be shorter than the chord


def test_reference_polyline_heading_matches_real_ego_heading(sample1):
    from trace_fixer.export.road_geometry import build_reference_polyline
    from trace_fixer.geo.transform import heading_to_yaw_rad

    ref = build_reference_polyline(sample1)
    first_pose = sample1.ego.poses[0]
    assert ref[0].heading_rad == pytest.approx(heading_to_yaw_rad(first_pose.heading_deg))


def test_project_to_road_round_trips_a_point_on_the_reference_line(sample1):
    from trace_fixer.export.road_geometry import build_reference_polyline, project_to_road

    ref = build_reference_polyline(sample1)
    mid = ref[len(ref) // 2]
    s, t = project_to_road(ref, mid.x, mid.y)
    assert s == pytest.approx(mid.s, abs=0.5)
    assert t == pytest.approx(0.0, abs=0.1)


def test_project_to_road_lateral_offset_sign(sample1):
    """A point offset to the left of the reference line (in its direction
    of travel) must project to positive t -- OpenDRIVE convention, matched
    to the existing ego-relative "left = +y" convention used elsewhere."""
    from trace_fixer.export.road_geometry import build_reference_polyline, project_to_road

    ref = build_reference_polyline(sample1)
    mid = ref[len(ref) // 2]
    left_x = mid.x - 5.0 * math.sin(mid.heading_rad)
    left_y = mid.y + 5.0 * math.cos(mid.heading_rad)
    _, t = project_to_road(ref, left_x, left_y)
    assert t > 4.0


def test_estimate_lane_sections_falls_back_without_annotation_signal():
    """No lane markings/vehicles at all -> a single default section using
    the trace's majority declared lane count."""
    from trace_fixer.export.road_geometry import DEFAULT_LANE_WIDTH_M, estimate_lane_sections
    from trace_fixer.models import Annotation, EgoPose, EgoTrace, Trace

    poses = [EgoPose(t_us=i * 100_000, lat_deg=0, lon_deg=0, heading_deg=0, vx_mps=10, vy_mps=0, vz_mps=0) for i in range(50)]
    for i, p in enumerate(poses):
        p.x_m, p.y_m = i * 10.0, 0.0
    ego = EgoTrace(poses=poses)
    annotation = Annotation(country_code=None)
    trace = Trace(trace_id="synthetic", ego=ego, annotation=annotation)

    from trace_fixer.export.road_geometry import build_reference_polyline

    ref = build_reference_polyline(trace)
    sections = estimate_lane_sections(trace, ref)
    assert len(sections) == 1
    assert sections[0].source == "default"
    assert sections[0].lane_widths_m == [DEFAULT_LANE_WIDTH_M] * sections[0].num_lanes


def test_real_samples_never_produce_out_of_bounds_lane_geometry(sample1, sample2):
    """Whatever the estimator concludes for real, messy data, it must
    always respect the sanity bounds -- that's the whole point of the
    fallback design."""
    from trace_fixer.export.road_geometry import (
        MAX_LANE_WIDTH_M,
        MAX_LANES,
        MIN_LANE_WIDTH_M,
        MIN_LANES,
        build_road_geometry_plan,
    )

    for trace in (sample1, sample2):
        plan = build_road_geometry_plan(trace)
        assert plan.lane_sections
        for section in plan.lane_sections:
            assert MIN_LANES <= section.num_lanes <= MAX_LANES
            assert len(section.lane_widths_m) == section.num_lanes
            for w in section.lane_widths_m:
                assert MIN_LANE_WIDTH_M <= w <= MAX_LANE_WIDTH_M


def test_annotation_derived_sections_agree_with_frame_meta_where_both_exist(sample1):
    """Regression guard for the bug this was built to catch: an
    annotation-derived section overlapping a window where frame_meta.
    num_lanes is directly declared must agree with it -- see
    _frame_meta_num_lanes_by_window and its use in _estimate_window."""
    from trace_fixer.export.road_geometry import (
        WINDOW_LEN_M,
        _frame_meta_num_lanes_by_window,
        build_reference_polyline,
        build_road_geometry_plan,
    )

    ref = build_reference_polyline(sample1)
    declared = _frame_meta_num_lanes_by_window(sample1, ref)
    assert declared  # sanity: sample1 does have frame_meta.num_lanes coverage

    plan = build_road_geometry_plan(sample1)
    for section, next_section in zip(plan.lane_sections, plan.lane_sections[1:] + [None]):
        s_end = next_section.s_start if next_section else plan.total_length
        if section.source != "annotation":
            continue
        covered_windows = range(int(section.s_start // WINDOW_LEN_M), int(s_end // WINDOW_LEN_M) + 1)
        for w in covered_windows:
            if w in declared:
                assert section.num_lanes == declared[w]


def test_aggregate_static_objects_covers_every_annotated_object(sample1):
    from trace_fixer.export.road_geometry import aggregate_static_objects, build_reference_polyline

    ref = build_reference_polyline(sample1)
    placements = aggregate_static_objects(sample1, ref)
    assert len(placements) == len(sample1.annotation.static_objects)
    labels = {p.label for p in placements}
    assert "Traffic Sign" in labels
    for p in placements:
        assert p.odr_type  # every placement gets *some* type, even if best-effort/"none"


def test_static_object_type_mapping_preserves_original_label():
    from trace_fixer.export.road_geometry import DEFAULT_STATIC_OBJECT_TYPE, STATIC_OBJECT_TYPE_MAP

    assert STATIC_OBJECT_TYPE_MAP["Traffic Sign"] != ""
    assert DEFAULT_STATIC_OBJECT_TYPE  # a real fallback value, not empty


def _matching_enrichment(trace, tags):
    """A synthetic enrichment result whose one way is built directly from
    the trace's own GPS path, so it's guaranteed to match."""
    from trace_fixer.export.map_enrichment import BBox, MapEnrichmentResult, MapWay

    points = [(p.lat_deg, p.lon_deg) for p in trace.ego.poses[::200]]
    way = MapWay(id=1, points=points, tags=tags)
    return MapEnrichmentResult(provider="osm", bbox=BBox(0, 0, 0, 0), ways=[way])


def test_build_road_geometry_plan_without_enrichment_is_unaffected(sample1):
    """The default (no enrichment argument) must produce exactly the same
    plan as before enrichment support existed -- this is the "online mode
    never touches anything unless explicitly requested" guarantee."""
    from trace_fixer.export.road_geometry import build_road_geometry_plan

    plan = build_road_geometry_plan(sample1)
    assert plan.road_name is None


def test_matching_enrichment_sets_road_name_and_lanes_hint(sample1):
    from trace_fixer.export.road_geometry import build_road_geometry_plan

    enrichment = _matching_enrichment(sample1, {"name": "Test Highway", "lanes": "2"})
    plan = build_road_geometry_plan(sample1, enrichment=enrichment)
    assert plan.road_name == "Test Highway"
    # every "default" section (no trustworthy annotation estimate) should
    # now be sourced from the OSM hint rather than the blind majority guess
    assert any(s.source == "osm_default" for s in plan.lane_sections)


def test_unrelated_enrichment_way_is_not_matched(sample1):
    """A way nowhere near the trace's own GPS path (a different city, say)
    must not be treated as "this road" just because it was returned in
    some bounding-box query."""
    from trace_fixer.export.map_enrichment import BBox, MapEnrichmentResult, MapWay
    from trace_fixer.export.road_geometry import build_road_geometry_plan

    far_away_way = MapWay(id=99, points=[(40.7128, -74.0060), (40.72, -74.01)], tags={"name": "Somewhere Else"})
    enrichment = MapEnrichmentResult(provider="osm", bbox=BBox(0, 0, 0, 0), ways=[far_away_way])
    plan = build_road_geometry_plan(sample1, enrichment=enrichment)
    assert plan.road_name is None


def test_lanes_hint_out_of_bounds_falls_back_to_annotation_majority(sample1):
    from trace_fixer.export.road_geometry import build_road_geometry_plan

    enrichment = _matching_enrichment(sample1, {"name": "Weird Road", "lanes": "99"})
    plan = build_road_geometry_plan(sample1, enrichment=enrichment)
    assert plan.road_name == "Weird Road"
    assert all(s.num_lanes != 99 for s in plan.lane_sections)


def test_default_section_puts_the_ego_mid_lane_not_on_a_lane_edge(sample1):
    """The reference line is the ego's driven path, so the fallback
    cross-section must offset the lane stack by half a lane -- otherwise
    the ego sits on its own lane's boundary and everything is shifted."""
    from trace_fixer.export.road_geometry import DEFAULT_LANE_WIDTH_M, _default_section

    section = _default_section(0.0, sample1)
    assert section.center_offset_m == pytest.approx(DEFAULT_LANE_WIDTH_M / 2)


def test_default_section_places_the_ego_using_frame_meta_ego_lane(sample1):
    """frame_meta.ego_lane is 1-based from the *rightmost* lane, so it says
    how many lanes belong on each side of the ego rather than assuming it's
    in the leftmost one."""
    from trace_fixer.export.road_geometry import _default_section, _majority_ego_lane, _majority_num_lanes

    ego_lane = _majority_ego_lane(sample1)
    assert ego_lane is not None, "sample1 is expected to carry frame_meta.ego_lane"
    total = _majority_num_lanes(sample1)

    section = _default_section(0.0, sample1)
    assert section.num_lanes == min(ego_lane, total)
    assert len(section.left_lane_widths_m) == max(0, total - min(ego_lane, total))


def test_window_estimate_carries_the_measured_lane_edge_as_the_offset():
    """The ego's own lane's left edge was previously computed for a sanity
    check and then thrown away; it must survive into the plan."""
    from trace_fixer.export.road_geometry import _estimate_window

    # boundaries at -7, -3.5, 0.7, 4.2: ego bracketed by -3.5 and 0.7
    marking_t = [-7.0] * 8 + [-3.5] * 8 + [0.7] * 8 + [4.2] * 8
    section = _estimate_window(marking_t, [], declared_num_lanes=None)
    assert section is not None
    assert section.center_offset_m == pytest.approx(0.7, abs=1e-6)
    assert len(section.left_lane_widths_m) == 1  # the 0.7 -> 4.2 lane
    assert section.left_lane_widths_m[0] == pytest.approx(3.5, abs=1e-6)


def test_frame_meta_cross_check_accepts_the_ego_outside_the_leftmost_lane():
    """declared num_lanes may count the whole carriageway. Comparing it
    against the right-side count alone rejected every window where the ego
    wasn't leftmost -- which silently forced those traces onto the default
    cross-section."""
    from trace_fixer.export.road_geometry import _estimate_window

    marking_t = [-7.0] * 8 + [-3.5] * 8 + [0.7] * 8 + [4.2] * 8  # 2 right + 1 left
    accepted = _estimate_window(marking_t, [], declared_num_lanes=3)
    assert accepted is not None
    assert accepted.num_lanes == 2
    assert len(accepted.left_lane_widths_m) == 1

    # a declared count matching only the right side keeps the old reading
    right_only = _estimate_window(marking_t, [], declared_num_lanes=2)
    assert right_only is not None
    assert right_only.num_lanes == 2
    assert right_only.left_lane_widths_m == []

    assert _estimate_window(marking_t, [], declared_num_lanes=5) is None


def test_persistence_filter_tolerates_width_noise_between_matching_windows():
    """Regression guard for a real-corpus bug: five consecutive 30m windows
    of the same physical lane (structurally identical -- same lane count,
    same left/right split) but with independently-estimated widths that
    differ by more than the old 0.3m 'same' tolerance (real marking-
    detection noise routinely does) used to each look like an isolated
    one-off to the persistence filter and get discarded in favor of
    whatever state preceded them -- observed on a real trace where this
    silently erased annotation-derived lane data across roughly half the
    road, in turn leaving no left lane modeled for a genuinely-present
    passing lane and putting a real vehicle observed there off-road by
    several meters. Grouping on structure (lane counts) rather than exact
    width fixes it: a persistent stretch survives as several honest
    annotation-sourced sections instead of being erased or averaged away.
    """
    from trace_fixer.export.road_geometry import build_reference_polyline, estimate_lane_sections
    from trace_fixer.models import Annotation, EgoPose, EgoTrace, LaneMarking, LaneSnapshot, Trace

    # heading_deg=270 -> math-convention yaw 0 (see geo.transform.heading_to_yaw_rad),
    # i.e. "pointing along +x" -- matching the poses' own x_m progression below.
    poses = [
        EgoPose(t_us=i * 100_000, lat_deg=0, lon_deg=0, heading_deg=270.0, vx_mps=10, vy_mps=0, vz_mps=0)
        for i in range(700)
    ]
    for i, p in enumerate(poses):
        p.x_m, p.y_m = float(i), 0.0
    ego = EgoTrace(poses=poses)

    # Windows 0-2 (s 0-90): no markings at all -> default fallback.
    # Windows 3-7 (s 90-240): one lane, boundary at t=0 (ego's own edge) and
    # t=-width, alternating 3.2/3.75m window to window -- same structure
    # (1 lane, 0 left) throughout, never numerically "same" under the old
    # 0.3m threshold.
    snapshots = []
    widths_by_window = {3: 3.2, 4: 3.75, 5: 3.2, 6: 3.75, 7: 3.2}
    for w, width in widths_by_window.items():
        s0 = w * 30.0
        pts = [(s0 + k, side) for k in range(25) for side in (-width, 0.05)]
        snapshots.append(LaneSnapshot(t_us=int(s0 * 1e5), frame=w, points_rel=[], points_m=pts))
    lane_marking = LaneMarking(obj_id=1, lane_type="solid", width=0.12, group=1, snapshots=snapshots)
    trace = Trace(
        trace_id="synthetic-persistence",
        ego=ego,
        annotation=Annotation(country_code=None, lane_markings={1: lane_marking}),
    )

    ref = build_reference_polyline(trace)
    sections = estimate_lane_sections(trace, ref)

    windowed = [s for s in sections if 90.0 <= s.s_start < 240.0]
    assert len(windowed) == 5, [(s.s_start, s.source) for s in sections]
    assert all(s.source == "annotation" for s in windowed)
    assert all(s.num_lanes == 1 and s.left_lane_widths_m == [] for s in windowed)
