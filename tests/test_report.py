"""Tests for the trace summary export (trace_fixer.export.report)."""
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
SAMPLE_DIR = REPO_ROOT / "data" / "traces" / "sample1"


@pytest.fixture()
def validated_trace():
    from trace_fixer.scene import load_trace
    from trace_fixer.validation.checks import run_validation

    trace = load_trace("sample1", SAMPLE_DIR / "adma.csv", SAMPLE_DIR / "annotation.xml")
    run_validation(trace)
    return trace


def test_txt_report_includes_scene_composition_and_issues(validated_trace):
    from trace_fixer.export.report import generate_txt_report

    text = generate_txt_report(validated_trace)
    assert validated_trace.trace_id in text
    assert "Scene composition: 5 object(s)" in text
    assert "Truck: 4" in text
    assert "Car: 1" in text
    assert f"Data-quality issues: {len(validated_trace.issues)} " in text
    for issue in validated_trace.issues:
        assert issue.description in text


def test_txt_report_includes_overtake_events(validated_trace):
    from trace_fixer.export.report import generate_txt_report

    text = generate_txt_report(validated_trace)
    # sample1 has three vehicles the ego overtakes over the course of the clip
    assert "Overtake events: 3" in text
    assert "Ego overtakes vehicle (vehicle 1" in text


def test_txt_report_reports_no_braking_when_none_detected(validated_trace):
    from trace_fixer.export.report import generate_txt_report

    text = generate_txt_report(validated_trace)
    assert "Ego braking events: 0" in text
    assert "None detected." in text


def test_xml_report_is_well_formed_and_complete(validated_trace):
    from trace_fixer.export.report import generate_xml_report

    xml_text = generate_xml_report(validated_trace)
    root = ET.fromstring(xml_text)
    assert root.tag == "TraceSummary"
    assert root.attrib["traceId"] == "sample1"

    scene_el = root.find("SceneComposition")
    assert int(scene_el.attrib["totalObjects"]) == 5
    types = {el.attrib["type"]: int(el.attrib["count"]) for el in scene_el.findall("ObjectType")}
    assert types == {"Truck": 4, "Car": 1}

    overtakes = root.find("OvertakeEvents").findall("OvertakeEvent")
    assert len(overtakes) == 3
    assert all(o.attrib["direction"] == "ego_overtakes_vehicle" for o in overtakes)

    issues_el = root.find("DataQualityIssues")
    issue_elements = issues_el.findall("Issue")
    assert len(issue_elements) == len(validated_trace.issues)
    assert int(issues_el.attrib["total"]) == len(validated_trace.issues)


def test_report_reflects_fixed_status(validated_trace):
    from trace_fixer.export.report import generate_txt_report
    from trace_fixer.validation.fixes import apply_fixes

    apply_fixes(validated_trace)  # resolves all issues in the sample trace
    text = generate_txt_report(validated_trace)
    assert "Data-quality issues: 0 " in text
    assert "No data-quality issues found." in text
