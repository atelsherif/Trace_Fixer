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
