"""Tests for bulk directory scanning/matching (trace_fixer.scan) and the
TraceStore's external-registration path."""
from pathlib import Path

import pytest


def _make_pair(root: Path, trace_name: str, xml_suffix: str) -> None:
    adma_dir = root / "adma" / "ADMA" / trace_name
    adma_dir.mkdir(parents=True, exist_ok=True)
    (adma_dir / "adma.csv").touch()
    ann_dir = root / "annotations" / "Annotations"
    ann_dir.mkdir(parents=True, exist_ok=True)
    (ann_dir / f"{trace_name}{xml_suffix}").touch()


@pytest.fixture()
def corpus(tmp_path):
    root = tmp_path / "corpus"
    _make_pair(root, "LB-VS-271_20200722_split_038_MERGED", "__ref-QC_IND.xml")
    _make_pair(root, "LB-VS-291_20201118170141_C022_split101_MERGED", "__refQC_IND.xml")
    # an XML with no matching ADMA folder, and vice versa
    (root / "annotations" / "Annotations" / "unrelated_notes.xml").touch()
    lonely_adma = root / "adma" / "ADMA" / "LB-VS-999_orphan"
    lonely_adma.mkdir(parents=True)
    (lonely_adma / "adma.csv").touch()
    return root


def test_scan_matches_both_suffix_variants(corpus):
    from trace_fixer.scan import scan_for_trace_pairs

    result = scan_for_trace_pairs(corpus)
    assert result.adma_found == 3
    assert result.xml_found == 3
    assert set(result.matched.keys()) == {
        "LB-VS-271_20200722_split_038_MERGED",
        "LB-VS-291_20201118170141_C022_split101_MERGED",
    }
    assert result.unmatched_adma_count == 1  # the orphan adma dir
    assert result.unmatched_xml_count == 1  # unrelated_notes.xml


def test_scan_result_paths_point_at_real_files(corpus):
    from trace_fixer.scan import scan_for_trace_pairs

    result = scan_for_trace_pairs(corpus)
    adma_path, xml_path = result.matched["LB-VS-271_20200722_split_038_MERGED"]
    assert adma_path.name == "adma.csv"
    assert adma_path.exists()
    assert xml_path.name == "LB-VS-271_20200722_split_038_MERGED__ref-QC_IND.xml"
    assert xml_path.exists()


def test_scan_rejects_non_directory(tmp_path):
    from trace_fixer.scan import scan_for_trace_pairs

    with pytest.raises(NotADirectoryError):
        scan_for_trace_pairs(tmp_path / "does-not-exist")


def test_store_registers_scanned_traces_without_copying(tmp_path, corpus):
    from trace_fixer.store import TraceStore

    store = TraceStore(traces_dir=tmp_path / "traces_dir")
    assert store.count() == 0

    result = store.scan_directory(corpus)
    assert len(result.matched) == 2
    assert store.count() == 2
    assert "LB-VS-271_20200722_split_038_MERGED" in store.list_ids()

    # nothing was copied into the store's own traces_dir
    assert list((tmp_path / "traces_dir").iterdir()) == []


def test_store_list_ids_query_and_limit(tmp_path, corpus):
    from trace_fixer.store import TraceStore

    store = TraceStore(traces_dir=tmp_path / "traces_dir")
    store.scan_directory(corpus)

    assert store.list_ids(query="271") == ["LB-VS-271_20200722_split_038_MERGED"]
    assert len(store.list_ids(limit=1)) == 1
    assert store.list_ids(query="nonexistent") == []


def test_trace_name_comes_from_the_nearest_distinctive_directory(tmp_path):
    """Regression guard: a corpus laid out as <root>/<trace>/adma/adma.csv
    used to collapse to a single trace called "adma", because the trace
    name was taken from adma.csv's immediate parent unconditionally.
    """
    from trace_fixer.scan import scan_for_trace_pairs

    names = [f"LBVS271_20200722_split_{i:03d}_MERGED" for i in range(5)]
    for name in names:
        adma_dir = tmp_path / name / "adma"
        adma_dir.mkdir(parents=True)
        (adma_dir / "adma.csv").write_text("x")
        ann_dir = tmp_path / name / "annotations"
        ann_dir.mkdir(parents=True)
        (ann_dir / f"{name}__refQC_IND.xml").write_text("x")

    result = scan_for_trace_pairs(tmp_path)
    assert result.adma_files_found == 5
    assert result.adma_found == 5
    assert set(result.matched) == set(names)
    assert result.name_collisions == 0


def test_generic_directory_names_are_skipped_all_the_way_up(tmp_path):
    from trace_fixer.scan import scan_for_trace_pairs

    name = "Trace_XYZ"
    deep = tmp_path / name / "data" / "raw"
    deep.mkdir(parents=True)
    (deep / "adma.csv").write_text("x")
    (tmp_path / name / f"{name}__refQC_IND.xml").write_text("x")

    result = scan_for_trace_pairs(tmp_path)
    assert set(result.matched) == {name}


def test_scan_reports_diagnostics_for_a_corpus_that_matches_poorly(tmp_path):
    """Counts and examples so a surprisingly low match count can be
    diagnosed from the GUI instead of guessed at."""
    from trace_fixer.scan import scan_for_trace_pairs

    paired = tmp_path / "Trace_A"
    paired.mkdir()
    (paired / "adma.csv").write_text("x")
    (paired / "Trace_A__refQC_IND.xml").write_text("x")

    lonely = tmp_path / "Trace_B"  # adma with no annotation
    lonely.mkdir()
    (lonely / "adma.csv").write_text("x")

    orphan = tmp_path / "loose"  # annotation matching no trace
    orphan.mkdir()
    (orphan / "Completely_Unrelated__refQC_IND.xml").write_text("x")

    result = scan_for_trace_pairs(tmp_path)
    assert set(result.matched) == {"Trace_A"}
    assert result.unmatched_adma_count == 1
    assert "Trace_B" in result.unmatched_adma_examples
    assert result.unmatched_xml_count == 1
    assert "Completely_Unrelated__refQC_IND.xml" in result.unmatched_xml_examples


def test_colliding_trace_names_are_counted_not_silently_dropped(tmp_path):
    from trace_fixer.scan import scan_for_trace_pairs

    for run in ("run1", "run2"):
        d = tmp_path / run / "SameName"
        d.mkdir(parents=True)
        (d / "adma.csv").write_text("x")
    (tmp_path / "SameName__refQC_IND.xml").write_text("x")

    result = scan_for_trace_pairs(tmp_path)
    assert result.adma_files_found == 2
    assert result.adma_found == 1
    assert result.name_collisions == 1
