"""Tests for writing corrected files into the gitignored output/ directory
(trace_fixer.export.batch_output), used by the batch fix+predict action."""
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
SAMPLE_DIR = REPO_ROOT / "data" / "traces" / "sample1"


@pytest.fixture()
def fixed_trace():
    from trace_fixer.scene import load_trace
    from trace_fixer.validation.checks import run_validation
    from trace_fixer.validation.fixes import apply_fixes

    trace = load_trace("sample1", SAMPLE_DIR / "adma.csv", SAMPLE_DIR / "annotation.xml")
    run_validation(trace)
    apply_fixes(trace)
    return trace


def test_output_mirrors_input_layout(fixed_trace, tmp_path):
    from trace_fixer.export.batch_output import write_batch_output

    output_root = tmp_path / "output"
    paths = write_batch_output(fixed_trace, SAMPLE_DIR / "annotation.xml", output_root)

    adma_path = Path(paths["adma_path"])
    annotation_path = Path(paths["annotation_path"])
    assert adma_path == output_root / "adma" / "ADMA" / "sample1" / "adma.csv"
    assert adma_path.exists()
    assert annotation_path == output_root / "annotations" / "Annotations" / "sample1__refQC_IND.xml"
    assert annotation_path.exists()


def test_output_preserves_original_filename_for_scanned_traces(fixed_trace, tmp_path):
    from trace_fixer.export.batch_output import write_batch_output

    output_root = tmp_path / "output"
    original = tmp_path / "some_dir" / "LB-VS-271_20200722_split_038_MERGED__ref-QC_IND.xml"
    original.parent.mkdir(parents=True)
    original.write_text((SAMPLE_DIR / "annotation.xml").read_text(encoding="iso-8859-1"), encoding="iso-8859-1")

    paths = write_batch_output(fixed_trace, original, output_root)
    assert Path(paths["annotation_path"]).name == "LB-VS-271_20200722_split_038_MERGED__ref-QC_IND.xml"


def test_output_content_is_valid_and_reflects_fixes(fixed_trace, tmp_path):
    from trace_fixer.export.batch_output import write_batch_output
    from trace_fixer.parsers.adma_csv import parse_adma_csv
    from trace_fixer.parsers.annotation_xml import parse_annotation_xml

    output_root = tmp_path / "output"
    paths = write_batch_output(fixed_trace, SAMPLE_DIR / "annotation.xml", output_root)

    reparsed_ego = parse_adma_csv(paths["adma_path"])
    assert len(reparsed_ego.poses) == len(fixed_trace.ego.poses)

    reparsed_ann = parse_annotation_xml(paths["annotation_path"])
    assert set(reparsed_ann.vehicles.keys()) == set(fixed_trace.annotation.vehicles.keys())


def test_output_directory_is_gitignored():
    gitignore = (REPO_ROOT / ".gitignore").read_text()
    assert "/output/" in gitignore
