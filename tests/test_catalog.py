"""Tests for trace_fixer.catalog: the SQLite trace catalog (identity,
location/metadata, phenomenon tags, issue counts) and its query filters.
"""
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
SAMPLE1_DIR = REPO_ROOT / "data" / "traces" / "sample1"
SAMPLE2_DIR = REPO_ROOT / "data" / "traces" / "sample2"


@pytest.fixture()
def conn(tmp_path):
    from trace_fixer.catalog import connect

    c = connect(tmp_path / "catalog.sqlite")
    yield c
    c.close()


@pytest.fixture()
def sample1():
    from trace_fixer.scene import load_trace

    return load_trace("sample1", SAMPLE1_DIR / "adma.csv", SAMPLE1_DIR / "annotation.xml")


@pytest.fixture()
def sample2():
    from trace_fixer.scene import load_trace

    return load_trace("sample2", SAMPLE2_DIR / "adma.csv", SAMPLE2_DIR / "annotation.xml")


def test_register_scanned_creates_identity_row(conn):
    from trace_fixer import catalog

    catalog.register_scanned(conn, "trace-a", Path("/x/adma.csv"), Path("/x/annotation.xml"))
    rows, total = catalog.query(conn)
    assert total == 1
    assert rows[0]["trace_id"] == "trace-a"
    assert rows[0]["adma_path"] == "/x/adma.csv"
    assert rows[0]["processed_at"] is None
    assert rows[0]["phenomena"] == []
    assert rows[0]["issues"] == []


def test_register_scanned_does_not_clobber_processed_metadata(conn, sample1):
    from trace_fixer import catalog

    catalog.record_trace(conn, sample1, SAMPLE1_DIR / "adma.csv", SAMPLE1_DIR / "annotation.xml")
    # a later re-scan (e.g. the corpus is scanned again) shouldn't erase
    # metadata already computed by a previous "Build Catalog" pass
    catalog.register_scanned(conn, "sample1", SAMPLE1_DIR / "adma.csv", SAMPLE1_DIR / "annotation.xml")
    rows, _total = catalog.query(conn)
    assert rows[0]["duration_s"] is not None
    assert rows[0]["duration_s"] > 0


def test_record_trace_populates_location_and_metadata(conn, sample1):
    from trace_fixer import catalog

    catalog.record_trace(conn, sample1, SAMPLE1_DIR / "adma.csv", SAMPLE1_DIR / "annotation.xml")
    rows, total = catalog.query(conn)
    assert total == 1
    row = rows[0]
    assert row["trace_id"] == "sample1"
    assert row["first_lat"] is not None and row["first_lon"] is not None
    assert row["vehicle_count"] == 5
    assert row["duration_s"] == pytest.approx(60.0, abs=1.0)
    assert row["processed_at"] is not None


def test_record_trace_populates_phenomena_and_issues(conn, sample2):
    from trace_fixer.catalog import record_trace
    from trace_fixer.validation.checks import run_validation

    run_validation(sample2)  # sample2 has real off_road/kinematic issues
    record_trace(conn, sample2, SAMPLE2_DIR / "adma.csv", SAMPLE2_DIR / "annotation.xml")

    rows, _total = query_module_rows(conn)
    row = rows[0]
    issue_categories = {i["category"] for i in row["issues"]}
    assert "off_road" in issue_categories
    assert "kinematic" in issue_categories
    assert sum(i["count"] for i in row["issues"]) == len(sample2.issues)


def test_record_trace_can_skip_phenomena_and_issues(conn, sample1):
    from trace_fixer.catalog import record_trace

    record_trace(
        conn, sample1, SAMPLE1_DIR / "adma.csv", SAMPLE1_DIR / "annotation.xml",
        include_phenomena=False, include_issues=False,
    )
    rows, _total = query_module_rows(conn)
    assert rows[0]["phenomena"] == []
    assert rows[0]["issues"] == []
    assert rows[0]["vehicle_count"] == 5  # identity/location metadata is still written


def test_record_trace_replaces_rather_than_accumulates(conn, sample2):
    from trace_fixer.catalog import record_trace
    from trace_fixer.validation.checks import run_validation

    run_validation(sample2)
    record_trace(conn, sample2, SAMPLE2_DIR / "adma.csv", SAMPLE2_DIR / "annotation.xml")
    first_issue_total = sum(i["count"] for i in query_module_rows(conn)[0][0]["issues"])

    sample2.issues.clear()  # simulate a full fix -- re-processing should reflect that, not add to it
    record_trace(conn, sample2, SAMPLE2_DIR / "adma.csv", SAMPLE2_DIR / "annotation.xml")
    rows, _total = query_module_rows(conn)
    assert rows[0]["issues"] == []
    assert first_issue_total > 0  # sanity: there really was something to clear


def test_query_filters_by_phenomenon_and_issue_category(conn, sample1, sample2):
    from trace_fixer.catalog import record_trace
    from trace_fixer.validation.checks import run_validation

    record_trace(conn, sample1, SAMPLE1_DIR / "adma.csv", SAMPLE1_DIR / "annotation.xml")  # has cut_in
    run_validation(sample2)
    record_trace(conn, sample2, SAMPLE2_DIR / "adma.csv", SAMPLE2_DIR / "annotation.xml")  # has off_road issues

    rows, total = query_module_rows(conn, phenomena=["cut_in"])
    assert total == 1
    assert rows[0]["trace_id"] == "sample1"

    rows, total = query_module_rows(conn, issue_categories=["off_road"])
    assert total == 1
    assert rows[0]["trace_id"] == "sample2"

    # combining a phenomenon sample1 has with a category only sample2 has -> no match
    rows, total = query_module_rows(conn, phenomena=["cut_in"], issue_categories=["off_road"])
    assert total == 0


def test_query_text_search_and_pagination(conn):
    from trace_fixer.catalog import register_scanned

    for name in ["alpha-1", "alpha-2", "beta-1"]:
        register_scanned(conn, name, Path(f"/x/{name}/adma.csv"), Path(f"/x/{name}/annotation.xml"))

    rows, total = query_module_rows(conn, q="alpha")
    assert total == 2
    assert {r["trace_id"] for r in rows} == {"alpha-1", "alpha-2"}

    rows, total = query_module_rows(conn, limit=1, offset=1)
    assert total == 3
    assert len(rows) == 1


def query_module_rows(conn, **kwargs):
    from trace_fixer.catalog import query

    return query(conn, **kwargs)


def test_stats_counts_total_and_processed(conn, sample1):
    from trace_fixer.catalog import record_trace, register_scanned, stats

    register_scanned(conn, "unprocessed", Path("/x/adma.csv"), Path("/x/annotation.xml"))
    record_trace(conn, sample1, SAMPLE1_DIR / "adma.csv", SAMPLE1_DIR / "annotation.xml")

    s = stats(conn)
    assert s["total"] == 2
    assert s["processed"] == 1

