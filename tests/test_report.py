"""Tests for the problem-report export (trace_fixer.export.report)."""
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


def test_txt_report_lists_every_issue(validated_trace):
    from trace_fixer.export.report import generate_txt_report

    text = generate_txt_report(validated_trace)
    assert validated_trace.trace_id in text
    assert f"Summary: {len(validated_trace.issues)} issue(s)" in text
    for issue in validated_trace.issues:
        assert issue.description in text


def test_xml_report_is_well_formed_and_complete(validated_trace):
    from trace_fixer.export.report import generate_xml_report

    xml_text = generate_xml_report(validated_trace)
    root = ET.fromstring(xml_text)
    assert root.tag == "TraceProblemReport"
    assert root.attrib["traceId"] == "sample1"
    issue_elements = root.find("Issues").findall("Issue")
    assert len(issue_elements) == len(validated_trace.issues)
    summary = root.find("Summary")
    assert int(summary.attrib["total"]) == len(validated_trace.issues)


def test_report_reflects_fixed_status(validated_trace):
    from trace_fixer.export.report import generate_txt_report
    from trace_fixer.validation.fixes import apply_fixes

    apply_fixes(validated_trace)  # resolves all issues in the sample trace
    text = generate_txt_report(validated_trace)
    assert "Summary: 0 issue(s)" in text
    assert "No issues found." in text
