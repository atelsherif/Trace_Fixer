"""End-to-end smoke tests for the trace_fixer pipeline, run against the
bundled sample trace (data/traces/sample1)."""
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
SAMPLE_DIR = REPO_ROOT / "data" / "traces" / "sample1"


@pytest.fixture()
def trace():
    from trace_fixer.scene import load_trace

    return load_trace("sample1", SAMPLE_DIR / "adma.csv", SAMPLE_DIR / "annotation.xml")


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


def test_validation_finds_known_issues(trace):
    from trace_fixer.validation.checks import run_validation

    issues = run_validation(trace)
    assert len(issues) > 0
    assert any(i.category == "collision" for i in issues)
    assert any(i.category == "kinematic" for i in issues)


def test_fix_engine_resolves_flagged_issues(trace):
    from trace_fixer.validation.checks import run_validation
    from trace_fixer.validation.fixes import apply_fixes

    run_validation(trace)
    apply_fixes(trace)
    assert len(trace.issues) == 0


def test_prediction_adds_prefix_for_mid_clip_vehicles(trace):
    from trace_fixer.prediction.extrapolate import predict_all

    added = predict_all(trace)
    assert set(added.keys()) == {2, 4, 5}  # vehicles 1 and 3 are visible from frame 1
    for track in trace.annotation.vehicles.values():
        synthetic = [o for o in track.observations if o.synthetic]
        real = [o for o in track.observations if not o.synthetic]
        if synthetic:
            assert synthetic[-1].t_us < real[0].t_us
            assert all(a.t_us < b.t_us for a, b in zip(track.observations, track.observations[1:]))


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
    assert negative_frame_count == 60  # 3 mid-clip vehicles * 20-step default horizon


def test_export_opendrive_and_openscenario_are_well_formed(trace):
    from trace_fixer.export.opendrive import generate_opendrive
    from trace_fixer.export.openscenario import generate_openscenario

    xodr = generate_opendrive(trace)
    ET.fromstring(xodr)
    xosc = generate_openscenario(trace, "sample1.xodr")
    ET.fromstring(xosc)
