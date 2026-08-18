"""End-to-end smoke tests for the trace_fixer pipeline, run against the
bundled sample trace (data/traces/sample1)."""
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
SAMPLE_DIR = REPO_ROOT / "data" / "traces" / "sample1"
SAMPLE2_DIR = REPO_ROOT / "data" / "traces" / "sample2"


@pytest.fixture()
def trace():
    from trace_fixer.scene import load_trace

    return load_trace("sample1", SAMPLE_DIR / "adma.csv", SAMPLE_DIR / "annotation.xml")


@pytest.fixture()
def trace2():
    """sample2 -- used for the issue-detection/fix tests instead of sample1,
    since sample1 (once zrot is correctly read as degrees; see
    test_zrot_units.py) turns out to be a genuinely clean, issue-free trace.
    """
    from trace_fixer.scene import load_trace

    return load_trace("sample2", SAMPLE2_DIR / "adma.csv", SAMPLE2_DIR / "annotation.xml")


def test_parses_expected_shape(trace):
    assert len(trace.ego.poses) == 6000
    assert len(trace.annotation.vehicles) == 5
    assert len(trace.annotation.lane_markings) == 35
    assert len(trace.annotation.border_lines) == 15


def test_heading_matches_velocity_direction(trace):
    import math

    p = trace.ego.poses[500]
    yaw_from_heading = math.radians((90 + p.heading_deg) % 360)
    fwd = (math.cos(yaw_from_heading), math.sin(yaw_from_heading))
    speed = math.hypot(p.vx_mps, p.vy_mps)
    vel_unit = (p.vy_mps / speed, p.vx_mps / speed)  # (E, N) -> (x, y)
    dot = fwd[0] * vel_unit[0] + fwd[1] * vel_unit[1]
    assert dot > 0.999  # heading-derived forward direction tracks actual velocity


def test_scene_json_builds(trace):
    from trace_fixer.scene import build_scene_json

    scene = build_scene_json(trace)
    assert scene["vehicles"]
    assert scene["duration_s"] > 59
    assert scene["ego"]["path"]


def test_scene_json_ego_path_has_gps_coordinates(trace):
    from trace_fixer.scene import build_scene_json

    scene = build_scene_json(trace)
    first = scene["ego"]["path"][0]
    assert -90 <= first["lat"] <= 90
    assert -180 <= first["lon"] <= 180


def test_scene_json_events_match_sample1s_known_phenomena(trace):
    """sample1 has one near-miss/cut-in (vehicle 2) and three overtakes
    (vehicles 1, 4, 5) -- see test_analysis.py -- confirm the scene JSON
    surfaces the same events, sorted by time, with vehicle ids attached."""
    from trace_fixer.scene import build_scene_json

    scene = build_scene_json(trace)
    events = scene["events"]
    assert [e["t_start_s"] for e in events] == sorted(e["t_start_s"] for e in events)

    types = {e["type"] for e in events}
    assert {"near_miss", "cut_in", "overtake"} <= types
    assert all(e["vehicle_id"] == 2 for e in events if e["type"] in ("near_miss", "cut_in"))
    assert {e["vehicle_id"] for e in events if e["type"] == "overtake"} == {1, 4, 5}


def test_validation_finds_known_issues(trace2):
    from trace_fixer.validation.checks import run_validation

    issues = run_validation(trace2)
    assert len(issues) > 0
    assert any(i.category == "off_road" for i in issues)
    assert any(i.category == "kinematic" for i in issues)


def test_sample1_has_no_issues_once_correctly_parsed(trace):
    """sample1 turns out to be clean, steady-state highway driving once
    zrot is correctly read as degrees -- the yaw-rate issues it used to
    flag were an artifact of the (now-fixed) unit bug, not real annotation
    noise. See test_zrot_units.py for the underlying investigation.
    """
    from trace_fixer.validation.checks import run_validation

    assert run_validation(trace) == []


def test_fix_engine_resolves_flagged_issues(trace2):
    from trace_fixer.validation.checks import run_validation
    from trace_fixer.validation.fixes import apply_fixes

    before = len(run_validation(trace2))
    apply_fixes(trace2)
    after = len(trace2.issues)
    assert before > 0
    assert after < before  # fixing meaningfully reduces the issue count


def test_prediction_adds_prefix_and_suffix_for_mid_clip_vehicles(trace):
    from trace_fixer.prediction.extrapolate import predict_all

    added = predict_all(trace)
    # vehicles 1 and 3 are visible from frame 1 -> no pre-FOV gap to predict
    assert "backward" not in added[1] and "backward" not in added[3]
    # none of the tracks run to the very end of the clip (frame ~1501) ->
    # every vehicle has a post-FOV gap to predict
    assert all("forward" in added[vid] for vid in added)
    assert set(added.keys()) == {1, 2, 3, 4, 5}

    for track in trace.annotation.vehicles.values():
        real = [o for o in track.observations if not o.synthetic]
        pre = [o for o in track.observations if o.synthetic and o.t_us < real[0].t_us]
        post = [o for o in track.observations if o.synthetic and o.t_us > real[-1].t_us]
        if pre:
            assert pre[-1].t_us < real[0].t_us
        if post:
            assert post[0].t_us > real[-1].t_us
        ts = [o.t_us for o in track.observations]
        assert all(a < b for a, b in zip(ts, ts[1:]))


def test_export_adma_round_trips_when_unmodified(trace, tmp_path):
    from trace_fixer.export.adma_writer import write_adma_csv

    out = tmp_path / "adma_out.csv"
    write_adma_csv(trace.ego, out)
    original = (SAMPLE_DIR / "adma.csv").read_text().replace("\r\n", "\n")
    assert out.read_text() == original


def test_export_annotation_marks_predictions(trace, tmp_path):
    from trace_fixer.prediction.extrapolate import predict_all
    from trace_fixer.export.annotation_writer import write_annotation_xml
    from trace_fixer.parsers.annotation_xml import parse_annotation_xml

    predict_all(trace)
    out = tmp_path / "annotation_out.xml"
    write_annotation_xml(trace, SAMPLE_DIR / "annotation.xml", out)

    ET.parse(out)  # well-formed
    reparsed = parse_annotation_xml(out)
    negative_frame_count = sum(
        1 for t in reparsed.vehicles.values() for o in t.observations if o.frame < 0
    )
    assert negative_frame_count == 160  # 8 (vehicle, direction) pairs * 20-step default horizon


def test_export_opendrive_and_openscenario_are_well_formed(trace):
    from trace_fixer.export.opendrive import generate_opendrive
    from trace_fixer.export.openscenario import generate_openscenario

    xodr = generate_opendrive(trace)
    ET.fromstring(xodr)
    xosc = generate_openscenario(trace, "sample1.xodr")
    ET.fromstring(xosc)
