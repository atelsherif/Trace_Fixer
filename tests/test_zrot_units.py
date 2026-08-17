"""Regression tests for vehicle zrot's unit (always degrees in the raw
file, normalized to radians on parse).

Background: originally believed to vary by export (sample1 "radians",
sample2 "degrees") based on a magnitude-only heuristic -- flag as degrees
when a raw value exceeds +/-pi, since that's impossible for a properly
bounded relative heading as radians. That heuristic is necessary but not
sufficient: it correctly caught sample2 (and every other export tested),
but *missed* sample1, whose raw values happened to stay under pi by
coincidence despite also being degrees. Cross-checking decoded heading
against each vehicle's own position-implied direction of travel (from
position deltas, independent of zrot) against ADMA ground truth proved it:
misread as radians, sample1's mean heading error is 19.9 deg (consistent
with a near-random unit mismatch); correctly read as degrees, it drops to
0.5 deg. No confirmed radians file has been found across every export
tested, so the unit is now treated as always degrees (see
parsers.annotation_xml.detect_vehicle_zrot_unit).
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


def _heading_error_forcing_unit(sample_dir: Path, unit: str) -> float:
    """Builds a trace with vehicle zrot forced to the given unit ("rad" =
    raw file value used as-is, "deg" = converted via math.radians), for
    comparing hypotheses against ADMA ground truth without going through
    the real auto-detected parse path.
    """
    import xml.etree.ElementTree as ET

    from trace_fixer.geo.populate import populate_global_coords
    from trace_fixer.models import Annotation, Trace
    from trace_fixer.parsers.adma_csv import parse_adma_csv
    from trace_fixer.parsers.annotation_xml import _parse_vehicles

    ego = parse_adma_csv(sample_dir / "adma.csv")
    root = ET.parse(sample_dir / "annotation.xml").getroot()
    vehicles = _parse_vehicles(root)  # raw zrot, not unit-converted
    if unit == "deg":
        for track in vehicles.values():
            for obs in track.observations:
                obs.zrot = math.radians(obs.zrot)
    trace = Trace(
        trace_id=f"{sample_dir.name}-forced-{unit}",
        ego=ego,
        annotation=Annotation(country_code=None, vehicles=vehicles, vehicle_zrot_unit=unit),
    )
    populate_global_coords(trace)
    return _mean_heading_error(trace)


def test_detect_unit_is_always_degrees():
    from trace_fixer.parsers.annotation_xml import detect_vehicle_zrot_unit

    assert detect_vehicle_zrot_unit({}) == "deg"


def test_sample1_and_sample2_both_parse_as_degrees():
    from trace_fixer.parsers.annotation_xml import parse_annotation_xml

    for sample_dir in (SAMPLE1_DIR, SAMPLE2_DIR):
        ann = parse_annotation_xml(sample_dir / "annotation.xml")
        assert ann.vehicle_zrot_unit == "deg"
        max_abs = max(abs(o.zrot) for t in ann.vehicles.values() for o in t.observations)
        assert max_abs <= math.pi + 0.01  # normalized values must be plausible radians


def test_degrees_fixture_still_flags_correctly():
    from trace_fixer.parsers.annotation_xml import parse_annotation_xml

    ann = parse_annotation_xml(FIXTURES_DIR / "annotation_degrees_046.xml")
    assert ann.vehicle_zrot_unit == "deg"


def test_raw_sample2_values_would_be_implausible_as_radians():
    """Sanity-checks the premise of the (retired) magnitude heuristic
    against sample2's raw file: some unconverted zrot text values exceed
    pi, which cannot be a legitimate bounded relative-heading radian value.
    Kept as a red flag in case anyone is ever tempted to bring the
    magnitude-only heuristic back as the *sole* signal -- it's still true
    for sample2, but was never sufficient (see the sample1 tests below).
    """
    import xml.etree.ElementTree as ET

    tree = ET.parse(SAMPLE2_DIR / "annotation.xml")
    zrots = [float(el.text) for el in tree.getroot().iter("zrot")]
    assert max(abs(z) for z in zrots) > math.pi


def test_sample1_degrees_fits_far_better_than_radians():
    """The key regression guard for the original miss: sample1's raw zrot
    values are all individually small enough to *look* radian-plausible
    (max ~2.8, under pi), which is exactly why the old magnitude-only
    heuristic missed it. Comparing both hypotheses against ADMA ground
    truth settles it unambiguously.
    """
    rad_error = _heading_error_forcing_unit(SAMPLE1_DIR, "rad")
    deg_error = _heading_error_forcing_unit(SAMPLE1_DIR, "deg")
    assert deg_error < 1.0
    assert rad_error > 15.0
    assert rad_error > deg_error + 15.0


def test_sample2_degrees_fits_far_better_than_radians():
    """Same guard, for the file that was already correctly detected --
    confirms the comparison methodology itself is sound (matches the
    already-verified sample2 behavior)."""
    rad_error = _heading_error_forcing_unit(SAMPLE2_DIR, "rad")
    deg_error = _heading_error_forcing_unit(SAMPLE2_DIR, "deg")
    assert deg_error < 15.0
    assert rad_error > 60.0
    assert rad_error > deg_error + 40.0


def test_sample1_heading_matches_travel_direction_before_fixing():
    from trace_fixer.scene import load_trace

    trace = load_trace("sample1", SAMPLE1_DIR / "adma.csv", SAMPLE1_DIR / "annotation.xml")
    assert trace.annotation.vehicle_zrot_unit == "deg"
    assert _mean_heading_error(trace) < 1.0


def test_sample1_is_issue_free_once_correctly_parsed():
    """Confirms the practical consequence: sample1's previously-flagged
    yaw-rate issues were an artifact of the unit bug, not real annotation
    noise -- with degrees, validation finds nothing to flag."""
    from trace_fixer.scene import load_trace
    from trace_fixer.validation.checks import run_validation

    trace = load_trace("sample1", SAMPLE1_DIR / "adma.csv", SAMPLE1_DIR / "annotation.xml")
    assert run_validation(trace) == []


def test_sample2_heading_matches_travel_direction_before_fixing():
    from trace_fixer.scene import load_trace

    trace = load_trace("sample2", SAMPLE2_DIR / "adma.csv", SAMPLE2_DIR / "annotation.xml")
    assert trace.annotation.vehicle_zrot_unit == "deg"
    assert _mean_heading_error(trace) < 15.0


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
