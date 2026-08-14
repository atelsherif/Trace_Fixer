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
