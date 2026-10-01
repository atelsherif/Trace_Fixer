"""Tests that smoothing is targeted, bounded and optional.

Smoothing used to reshape every track unconditionally, which flattened real
driving: a vehicle not travelling in a perfectly straight line is not a
defect, and rewriting every heading from the path tangent threw away the
annotated box orientation across the whole clip. It now touches only the
stretches validation flagged as kinematically implausible.
"""
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
SAMPLE1_DIR = REPO_ROOT / "data" / "traces" / "sample1_baseline_1"
SAMPLE2_DIR = REPO_ROOT / "data" / "traces" / "sample2_baseline_2"
# sample2's observations all carry a numbered lane label, so the corridor
# check (and the clamp) correctly leave them alone -- it only exercises
# smoothing now. This one still has genuinely off-lane vehicles.
OFFROAD_DIR = REPO_ROOT / "data" / "traces" / "LB-VS-271_20200722_split_068_MERGED_baseline_6"


@pytest.fixture()
def sample1():
    from trace_fixer.scene import load_trace

    return load_trace("sample1", SAMPLE1_DIR / "adma.csv", SAMPLE1_DIR / "annotation.xml")


@pytest.fixture()
def sample2():
    from trace_fixer.scene import load_trace

    return load_trace("sample2", SAMPLE2_DIR / "adma.csv", SAMPLE2_DIR / "annotation.xml")


def _positions(trace):
    """Keyed by object identity, not (vehicle, timestamp): sample2 repeats a
    timestamp within one track (see test_pipeline's duplicate-timestamp
    regression), so a (vehicle, t_us) key silently collapses four distinct
    observations into one and reports their differing headings as changes.
    The fix engine mutates these objects in place, so identity holds.
    """
    return {
        id(o): (o.x_m, o.y_m, o.heading_deg)
        for track in trace.annotation.vehicles.values()
        for o in track.observations
    }


def test_a_clean_trace_comes_back_untouched(sample1):
    """sample1 is genuinely issue-free highway driving. Fixing it must be a
    no-op -- not a silent re-shaping of five perfectly good tracks."""
    from trace_fixer.validation.checks import run_validation
    from trace_fixer.validation.fixes import apply_fixes

    assert run_validation(sample1) == []
    before = _positions(sample1)
    summary = apply_fixes(sample1)

    assert summary == []
    assert _positions(sample1) == before
    assert sample1.provenance() == []


@pytest.fixture()
def offroad():
    from trace_fixer.scene import load_trace

    return load_trace("offroad", OFFROAD_DIR / "adma.csv", OFFROAD_DIR / "annotation.xml")


def test_smoothing_off_still_runs_the_corridor_clamp():
    """"off" is a smoothing switch, not a fixing switch.

    Built from a synthetic trace rather than a bundled one, because the
    clamp does not fire anywhere in the 25-trace motorway corpus: every
    candidate is either in a numbered lane (so the corridor's opinion does
    not apply) or more than MAX_CLAMP_M outside it (so the correction is
    declined as unjustifiable). Both are the right call, and both mean a
    real trace cannot cover this path. The case the clamp exists for is a
    box a few tens of centimetres outside the edge with no lane label.
    """
    from trace_fixer.geo.populate import populate_global_coords
    from trace_fixer.models import (
        Annotation, BorderLine, EgoPose, EgoTrace, LaneSnapshot, Trace, VehicleObs, VehicleTrack,
    )
    from trace_fixer.validation.checks import run_validation
    from trace_fixer.validation.fixes import MAX_CLAMP_M, apply_fixes

    poses = []
    for i in range(400):
        p = EgoPose(t_us=i * 100_000, lat_deg=0.0, lon_deg=0.0, heading_deg=270.0,
                    vx_mps=20.0, vy_mps=0.0, vz_mps=0.0)
        poses.append(p)
    ego = EgoTrace(poses=poses)

    def edge(obj_id, y):
        return BorderLine(obj_id=obj_id, obj_type="Road Edge", snapshots=[
            LaneSnapshot(t_us=t, frame=i, points_rel=[(float(x), y) for x in range(-40, 201, 10)])
            for i, t in enumerate(range(0, 40_000_000, 2_000_000))
        ])

    # Sits 0.4 m past the right edge -- noise, not a departure, and small
    # enough that the clamp can justify correcting it.
    obs = []
    for i in range(12):
        obs.append(VehicleObs(
            t_us=5_000_000 + i * 400_000, frame=50 + i, obj_movement="Moving",
            obj_lane="Other", obj_confidence="High",
            x_rel=25.0, y_rel=-5.4, z_rel=0.0,
            length=4.5, width=1.9, height=1.5, zrot=0.0,
        ))
    trace = Trace(
        trace_id="clampable", ego=ego,
        annotation=Annotation(
            country_code=None,
            vehicles={1: VehicleTrack(obj_id=1, obj_type="Car", reflecting_parts=None, observations=obs)},
            border_lines={1: edge(1, 5.0), 2: edge(2, -4.0)},
        ),
    )
    populate_global_coords(trace)

    issues = run_validation(trace)
    assert any(i.category == "off_road" for i in issues), "fixture must be off-road to begin with"

    apply_fixes(trace, smoothing="off")
    moved = [o.moved_from_original_m for o in trace.annotation.vehicles[1].observations]
    assert max(moved) > 0.05, "the corridor clamp must still run with smoothing off"
    assert max(moved) <= MAX_CLAMP_M + 1e-6


def test_the_clamp_leaves_vehicles_the_annotation_placed_in_a_lane(sample2):
    """sample2's observations all carry a numbered lane label, so the
    corridor has no authority over them -- the validator says nothing and
    the clamp must agree. These two used to disagree: the check deferred to
    the lane label while the clamp went on nudging the same boxes."""
    from trace_fixer.geo.road_corridor import ON_ROAD_LANE_LABELS
    from trace_fixer.validation.checks import run_validation
    from trace_fixer.validation.fixes import apply_fixes

    assert all(
        o.obj_lane in ON_ROAD_LANE_LABELS
        for track in sample2.annotation.vehicles.values()
        for o in track.observations
    )
    issues = run_validation(sample2)
    assert not any(i.category in ("off_road", "road_departure") for i in issues)

    apply_fixes(sample2, smoothing="off")
    assert not any(
        o.moved_from_original_m > 0.05
        for track in sample2.annotation.vehicles.values()
        for o in track.observations
    ), "the clamp moved a box the validator had no complaint about"


def test_smoothing_only_touches_what_validation_flagged(sample2):
    """Every moved observation must be near a flagged interval for its own
    vehicle -- no silent edits to stretches nothing was wrong with."""
    from trace_fixer.validation.checks import run_validation
    from trace_fixer.validation.fixes import SMOOTH_BLEND_S, SMOOTH_WINDOW_PAD_S, apply_fixes

    issues = run_validation(sample2)
    reach_us = int((SMOOTH_WINDOW_PAD_S + SMOOTH_BLEND_S) * 1e6)
    windows = {}
    for issue in issues:
        if issue.vehicle_id is not None and issue.category in ("kinematic", "off_road"):
            windows.setdefault(issue.vehicle_id, []).append((issue.t_start_us, issue.t_end_us))

    apply_fixes(sample2)

    for track in sample2.annotation.vehicles.values():
        for o in track.observations:
            if o.moved_from_original_m <= 0.05:
                continue
            spans = windows.get(track.obj_id, [])
            assert any(
                start - reach_us <= o.t_us <= end + reach_us for start, end in spans
            ), f"vehicle {track.obj_id} moved at t={o.t_us} with nothing flagged nearby"


def test_no_observation_is_moved_further_than_the_cap(sample2):
    """A smoother may nudge; it may not relocate."""
    from trace_fixer.validation.checks import run_validation
    from trace_fixer.validation.fixes import MAX_CLAMP_M, MAX_SMOOTH_SHIFT_M, apply_fixes

    run_validation(sample2)
    apply_fixes(sample2, smoothing="strong")
    cap = max(MAX_SMOOTH_SHIFT_M, MAX_CLAMP_M) + 1e-6
    for track in sample2.annotation.vehicles.values():
        for o in track.observations:
            assert o.moved_from_original_m <= cap


def test_headings_outside_a_flagged_window_are_left_as_annotated(sample2):
    """The old engine re-derived *every* heading from the path tangent, which
    discarded the annotated box orientation across the whole clip. Now a
    heading only changes where the position it was derived from did."""
    from trace_fixer.validation.checks import run_validation
    from trace_fixer.validation.fixes import SMOOTH_BLEND_S, SMOOTH_WINDOW_PAD_S, apply_fixes

    before = _positions(sample2)
    issues = run_validation(sample2)
    reach_us = int((SMOOTH_WINDOW_PAD_S + SMOOTH_BLEND_S) * 1e6)
    windows = {}
    for issue in issues:
        if issue.vehicle_id is not None and issue.category in ("kinematic", "off_road"):
            windows.setdefault(issue.vehicle_id, []).append((issue.t_start_us, issue.t_end_us))

    apply_fixes(sample2)

    changed = 0
    for track in sample2.annotation.vehicles.values():
        for o in track.observations:
            if o.heading_deg == before[id(o)][2]:
                continue
            changed += 1
            spans = windows.get(track.obj_id, [])
            assert any(
                start - reach_us <= o.t_us <= end + reach_us for start, end in spans
            ), f"vehicle {track.obj_id} heading rewritten at t={o.t_us} with nothing flagged nearby"
    assert changed, "this trace has real kinematic flags, so some headings must have been corrected"


def test_stronger_smoothing_moves_more_within_the_same_windows(sample2):
    from trace_fixer.scene import load_trace
    from trace_fixer.validation.checks import run_validation
    from trace_fixer.validation.fixes import apply_fixes

    def total_shift(strength):
        trace = load_trace("sample2", SAMPLE2_DIR / "adma.csv", SAMPLE2_DIR / "annotation.xml")
        run_validation(trace)
        apply_fixes(trace, smoothing=strength)
        return sum(
            o.moved_from_original_m
            for t in trace.annotation.vehicles.values()
            for o in t.observations
        )

    assert total_shift("off") < total_shift("light") < total_shift("strong")


def test_an_unknown_smoothing_strength_is_rejected(sample2):
    from trace_fixer.validation.fixes import apply_fixes

    with pytest.raises(ValueError, match="unknown smoothing strength"):
        apply_fixes(sample2, smoothing="aggressive")
