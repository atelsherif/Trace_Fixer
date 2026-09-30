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


def test_smoothing_off_leaves_positions_alone_but_still_clamps(sample2):
    """"off" is a smoothing switch, not a fixing switch: the corridor clamp
    and the trailing-overlap trim still run.

    Measured by what the clamp *moved*, not by the off-road count dropping.
    The clamp is rate-limited along the track (see _rate_limit_corrections),
    so it ramps a correction in and out rather than stepping it -- which
    means an observation at the edge of an excursion is deliberately left
    partly outside the corridor, and still flagged, instead of being yanked
    in and taking a lateral velocity spike with it.
    """
    from trace_fixer.validation.checks import run_validation
    from trace_fixer.validation.fixes import apply_fixes

    run_validation(sample2)
    kinematic_before = sum(1 for i in sample2.issues if i.category == "kinematic")
    apply_fixes(sample2, smoothing="off")

    # Nothing was smoothed, so the kinematic count is no worse than it was...
    assert sum(1 for i in sample2.issues if i.category == "kinematic") <= kinematic_before
    # ...and the clamp still moved boxes, which is the part "off" keeps.
    moved = [
        o.moved_from_original_m
        for track in sample2.annotation.vehicles.values()
        for o in track.observations
        if o.moved_from_original_m > 0.05
    ]
    assert moved, "the corridor clamp must still run with smoothing off"


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
