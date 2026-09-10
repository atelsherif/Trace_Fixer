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
    assert len(result.adma_collision_examples) == 1
    assert "run1" in result.adma_collision_examples[0] or "run2" in result.adma_collision_examples[0]


def test_duplicate_annotation_for_an_already_matched_trace_is_reported_not_silent(tmp_path):
    """A second annotation file resolving to a trace name a first one
    already claimed is a real match, just not the winning one -- distinct
    from 'unmatched' (it never fails to resolve a trace name at all) and,
    before xml_duplicate_count existed, invisible in every diagnostic."""
    from trace_fixer.scan import scan_for_trace_pairs

    d = tmp_path / "SameName"
    d.mkdir()
    (d / "adma.csv").write_text("x")
    (tmp_path / "SameName__refQC_IND.xml").write_text("x")
    (tmp_path / "reprocessed" / "SameName__refQC_IND.xml").parent.mkdir()
    (tmp_path / "reprocessed" / "SameName__refQC_IND.xml").write_text("x")

    result = scan_for_trace_pairs(tmp_path)
    assert set(result.matched) == {"SameName"}
    assert result.xml_duplicate_count == 1
    assert len(result.xml_duplicate_examples) == 1
    assert "reprocessed" in result.xml_duplicate_examples[0]
    # the duplicate is a real match, so it must not also inflate unmatched_xml_count
    assert result.unmatched_xml_count == 0


def test_scan_descends_into_symlinked_subdirectories(tmp_path):
    """Real corpora are routinely organized with symlinks (a shared-
    storage mount, a dedup layer, a 'latest' pointer tree) -- os.walk does
    not follow them by default, which would silently make everything past
    a symlink invisible to a scan without any error or diagnostic at all.
    """
    from trace_fixer.scan import scan_for_trace_pairs

    real_storage = tmp_path / "real_storage"
    _make_pair(real_storage, "LB-VS-500_real", "__refQC_IND.xml")

    corpus = tmp_path / "corpus"
    corpus.mkdir()
    (corpus / "logstream").symlink_to(real_storage, target_is_directory=True)

    result = scan_for_trace_pairs(corpus)
    assert set(result.matched) == {"LB-VS-500_real"}


def test_scan_does_not_hang_on_a_symlink_cycle(tmp_path):
    """A symlink pointing back at one of its own ancestors must be
    skipped on its second visit, not followed forever -- unlike plain
    os.walk(followlinks=True), which its own docs warn can recurse
    infinitely in exactly this case."""
    from trace_fixer.scan import scan_for_trace_pairs

    root = tmp_path / "corpus"
    _make_pair(root, "LB-VS-600_real", "__refQC_IND.xml")
    (root / "loop").symlink_to(root, target_is_directory=True)

    result = scan_for_trace_pairs(root)  # must return, not hang
    assert set(result.matched) == {"LB-VS-600_real"}


def test_unreadable_directories_are_counted_not_silently_skipped(tmp_path, monkeypatch):
    """A directory the walk can't open (permission denied, a stale/
    unmounted network share) is the most likely reason a whole subtree
    goes missing from a scan -- and both os.walk and this module's walker
    skip one silently. It must at least be counted, with the OS's reason,
    or "found far fewer files than this corpus contains" is unfalsifiable.
    """
    import os as os_module

    from trace_fixer import scan as scan_module

    _make_pair(tmp_path, "LB-VS-700_visible", "__refQC_IND.xml")
    locked = tmp_path / "locked_subtree"
    locked.mkdir()

    real_scandir = os_module.scandir

    def fake_scandir(path):
        if str(path).endswith("locked_subtree"):
            raise PermissionError(13, "Permission denied")
        return real_scandir(path)

    monkeypatch.setattr(scan_module.os, "scandir", fake_scandir)

    result = scan_module.scan_for_trace_pairs(tmp_path)
    # the readable half still scans -- one bad corner must not abort it
    assert set(result.matched) == {"LB-VS-700_visible"}
    assert result.dirs_unreadable == 1
    assert len(result.unreadable_examples) == 1
    assert "locked_subtree" in result.unreadable_examples[0]
    assert "Permission denied" in result.unreadable_examples[0]
    assert result.dirs_visited > 0


def test_scan_reports_adma_counts_per_subtree(tmp_path):
    """Which top-level branch the adma.csv files came from -- so a branch
    that contributed nothing is visible without dumping every path."""
    from trace_fixer.scan import scan_for_trace_pairs

    _make_pair(tmp_path, "LB-VS-800_a", "__refQC_IND.xml")
    _make_pair(tmp_path, "LB-VS-800_b", "__refQC_IND.xml")

    result = scan_for_trace_pairs(tmp_path)
    assert result.adma_files_by_subtree == {"adma": 2}
