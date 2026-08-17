"""Regression tests for the vehicle zrot degrees-vs-radians auto-detection.

Background: sample1's annotation export (structurefile minorversion 8)
encodes vehicle zrot in radians. Other exports (minorversion 7, e.g.
sample2 here) encode it in degrees instead, with no explicit unit field --
misreading one as the other produces vehicles that appear to spin/rotate
wildly or point perpendicular to their direction of travel, which is
exactly the bug report this fixture set was built to catch and pin down.
"""
import math
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
SAMPLE1_DIR = REPO_ROOT / "data" / "traces" / "sample1"
SAMPLE2_DIR = REPO_ROOT / "data" / "traces" / "sample2"
FIXTURES_DIR = REPO_ROOT / "tests" / "fixtures"


def _wrap180(a: float) -> float:
    return (a + 180) % 360 - 180


def _mean_heading_error(trace) -> float:
    """Mean absolute error (deg) between stored heading and the direction of
    travel implied by consecutive global positions, across all vehicles."""
    diffs = []
    for track in trace.annotation.vehicles.values():
        obs = track.observations
        for i in range(len(obs) - 1):
            o0, o1 = obs[i], obs[i + 1]
            dx, dy = o1.x_m - o0.x_m, o1.y_m - o0.y_m
            if math.hypot(dx, dy) < 0.5:
                continue
            implied = math.degrees(math.atan2(dy, dx)) % 360
            diffs.append(abs(_wrap180(implied - o0.heading_deg)))
    return sum(diffs) / len(diffs)


def test_detect_unit_radians_for_sample1():
    from trace_fixer.parsers.annotation_xml import parse_annotation_xml

    ann = parse_annotation_xml(SAMPLE1_DIR / "annotation.xml")
    assert ann.vehicle_zrot_unit == "rad"
    max_abs = max(abs(o.zrot) for t in ann.vehicles.values() for o in t.observations)
    assert max_abs <= math.pi


def test_detect_unit_degrees_for_sample2():
    from trace_fixer.parsers.annotation_xml import parse_annotation_xml

    ann = parse_annotation_xml(SAMPLE2_DIR / "annotation.xml")
    assert ann.vehicle_zrot_unit == "deg"
    # after normalization, values must still be plausible radians
    max_abs = max(abs(o.zrot) for t in ann.vehicles.values() for o in t.observations)
    assert max_abs <= math.pi + 0.01


def test_detect_unit_degrees_for_second_degrees_fixture():
    from trace_fixer.parsers.annotation_xml import parse_annotation_xml

    ann = parse_annotation_xml(FIXTURES_DIR / "annotation_degrees_046.xml")
    assert ann.vehicle_zrot_unit == "deg"


def test_raw_degrees_values_would_be_implausible_as_radians():
    """Sanity-checks the premise of the heuristic against the raw file: the
    unconverted zrot text values exceed pi, which cannot be a legitimate
    bounded relative-heading radian value.
    """
    import xml.etree.ElementTree as ET

    tree = ET.parse(SAMPLE2_DIR / "annotation.xml")
    zrots = [float(el.text) for el in tree.getroot().iter("zrot")]
    assert max(abs(z) for z in zrots) > math.pi


def test_sample1_heading_error_before_fixing_is_bounded():
    """sample1's raw zrot is genuinely noisy before fixing (that's what the
    fix engine is for -- see the "held stale keyframe" finding in the
    README), but it should still be bounded to a plausible annotation-noise
    range, not the ~90 degree (effectively random) error a unit mismatch
    would cause.
    """
    from trace_fixer.scene import load_trace

    trace = load_trace("sample1", SAMPLE1_DIR / "adma.csv", SAMPLE1_DIR / "annotation.xml")
    error = _mean_heading_error(trace)
    assert 0 < error < 30.0


def test_sample2_heading_matches_travel_direction_before_fixing():
    from trace_fixer.scene import load_trace

    trace = load_trace("sample2", SAMPLE2_DIR / "adma.csv", SAMPLE2_DIR / "annotation.xml")
    assert trace.annotation.vehicle_zrot_unit == "deg"
    assert _mean_heading_error(trace) < 15.0


def test_misinterpreting_sample2_as_radians_would_be_far_worse():
    """The key regression guard: if sample2's zrot were (incorrectly, as
    before this fix) treated as radians instead of degrees, the resulting
    heading error would be close to random (tens of degrees worse) rather
    than tracking the vehicles' actual direction of travel. This pins down
    *why* auto-detection matters, not just that it fires.
    """
    from trace_fixer.geo.populate import populate_global_coords
    from trace_fixer.models import Trace
    from trace_fixer.parsers.adma_csv import parse_adma_csv
    from trace_fixer.parsers.annotation_xml import _parse_vehicles
    import xml.etree.ElementTree as ET

    ego = parse_adma_csv(SAMPLE2_DIR / "adma.csv")
    root = ET.parse(SAMPLE2_DIR / "annotation.xml").getroot()
    vehicles_as_radians = _parse_vehicles(root)  # raw zrot, deliberately NOT unit-converted

    from trace_fixer.models import Annotation

    broken_trace = Trace(
        trace_id="sample2-forced-radians",
        ego=ego,
        annotation=Annotation(country_code=None, vehicles=vehicles_as_radians, vehicle_zrot_unit="rad"),
    )
    populate_global_coords(broken_trace)
    broken_error = _mean_heading_error(broken_trace)

    from trace_fixer.scene import load_trace

    correct_trace = load_trace("sample2", SAMPLE2_DIR / "adma.csv", SAMPLE2_DIR / "annotation.xml")
    correct_error = _mean_heading_error(correct_trace)

    assert correct_error < 15.0
    assert broken_error > 60.0
    assert broken_error > correct_error + 40.0


def test_sample2_fix_engine_brings_heading_close_to_travel_direction():
    from trace_fixer.scene import load_trace
    from trace_fixer.validation.checks import run_validation
    from trace_fixer.validation.fixes import apply_fixes

    trace = load_trace("sample2", SAMPLE2_DIR / "adma.csv", SAMPLE2_DIR / "annotation.xml")
    run_validation(trace)
    apply_fixes(trace)
    assert _mean_heading_error(trace) < 3.0


def test_export_writes_back_degrees_not_radians(tmp_path):
    from trace_fixer.export.annotation_writer import write_annotation_xml
    from trace_fixer.parsers.annotation_xml import parse_annotation_xml
    from trace_fixer.scene import load_trace
    from trace_fixer.validation.checks import run_validation
    from trace_fixer.validation.fixes import apply_fixes

    trace = load_trace("sample2", SAMPLE2_DIR / "adma.csv", SAMPLE2_DIR / "annotation.xml")
    run_validation(trace)
    apply_fixes(trace)

    out_path = tmp_path / "annotation_out.xml"
    write_annotation_xml(trace, SAMPLE2_DIR / "annotation.xml", out_path)

    # the raw (unconverted) text for a fixed observation should look like a
    # plausible small-magnitude degree value, not a multi-radian one
    import xml.etree.ElementTree as ET

    tree = ET.parse(out_path)
    fixed_track = next(t for t in trace.annotation.vehicles.values() if any(o.fixed for o in t.observations))
    fixed_obs = next(o for o in fixed_track.observations if o.fixed)
    rv = tree.getroot().find(f"vehicles/rect_vehicle[id='{fixed_track.obj_id}']")
    ts = rv.find(f"timestamps/rect_vehicle_timestamp[@frame='{fixed_obs.frame}']")
    raw_text = float(ts.find("bounding_box/vehicle_bb/coordinates/zrot").text)
    assert abs(raw_text - math.degrees(fixed_obs.zrot)) < 1e-6

    # and re-parsing the round-tripped file still detects degrees
    reparsed = parse_annotation_xml(out_path)
    assert reparsed.vehicle_zrot_unit == "deg"
