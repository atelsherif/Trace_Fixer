"""API-level tests for trace navigation (neighbor) and batch fix+predict.

Uses a small real corpus (copies of the bundled sample trace under a few
different names) rather than placeholder files, since these endpoints need
traces that actually parse.
"""
import time
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
SAMPLE_DIR = REPO_ROOT / "data" / "traces" / "sample1"


@pytest.fixture()
def client_with_corpus(tmp_path):
    """A fresh FastAPI app + store, pointed at an isolated traces_dir, with
    a 3-trace corpus (real sample data, 3 different names) registered via
    directory scan so tests don't depend on/pollute the module-level store.
    """
    from fastapi.testclient import TestClient

    import trace_fixer.api as api_module
    from trace_fixer.store import TraceStore

    api_module.store = TraceStore(traces_dir=tmp_path / "traces_dir")
    api_module.OUTPUT_DIR = tmp_path / "output"
    api_module.OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    corpus = tmp_path / "corpus"
    names = ["Trace-A", "Trace-B", "Trace-C"]
    for name in names:
        adma_dir = corpus / "adma" / "ADMA" / name
        adma_dir.mkdir(parents=True)
        (adma_dir / "adma.csv").write_bytes((SAMPLE_DIR / "adma.csv").read_bytes())
        ann_dir = corpus / "annotations" / "Annotations"
        ann_dir.mkdir(parents=True, exist_ok=True)
        (ann_dir / f"{name}__refQC_IND.xml").write_bytes((SAMPLE_DIR / "annotation.xml").read_bytes())

    client = TestClient(api_module.app)
    r = client.post("/api/traces/scan", json={"path": str(corpus)})
    assert r.status_code == 200
    assert r.json()["matched"] == 3
    return client, names, api_module.OUTPUT_DIR


def test_neighbor_cycles_through_full_sorted_list(client_with_corpus):
    client, names, _output_dir = client_with_corpus
    sorted_names = sorted(names, key=str.lower)

    r = client.get(f"/api/traces/{sorted_names[0]}/neighbor", params={"direction": "next"})
    assert r.status_code == 200
    assert r.json()["trace_id"] == sorted_names[1]

    r = client.get(f"/api/traces/{sorted_names[0]}/neighbor", params={"direction": "prev"})
    assert r.json()["trace_id"] == sorted_names[-1]  # wraps around

    r = client.get(f"/api/traces/{sorted_names[-1]}/neighbor", params={"direction": "next"})
    assert r.json()["trace_id"] == sorted_names[0]  # wraps around


def test_neighbor_respects_search_query(client_with_corpus):
    client, names, _output_dir = client_with_corpus
    r = client.get(f"/api/traces/{names[0]}/neighbor", params={"direction": "next", "q": names[0]})
    assert r.status_code == 200
    assert r.json()["trace_id"] == names[0]  # only itself matches the query -> stays put
    assert r.json()["total"] == 1


def test_neighbor_rejects_bad_direction(client_with_corpus):
    client, names, _output_dir = client_with_corpus
    r = client.get(f"/api/traces/{names[0]}/neighbor", params={"direction": "up"})
    assert r.status_code == 400


def test_neighbor_404_for_unknown_trace(client_with_corpus):
    client, _names, _output_dir = client_with_corpus
    r = client.get("/api/traces/does-not-exist/neighbor", params={"direction": "next"})
    assert r.status_code == 404


def test_batch_fix_predict_runs_full_pipeline(client_with_corpus):
    client, names, output_dir = client_with_corpus
    r = client.post(f"/api/traces/{names[0]}/batch_fix_predict")
    assert r.status_code == 200
    data = r.json()
    assert data["trace_id"] == names[0]
    assert data["before_issue_count"] == 0  # this corpus is copies of sample1, which is issue-free pre-fix
    assert len(data["fix_summary"]) == 5  # one line per vehicle in the sample trace
    assert set(data["predicted"].keys()) == {"1", "2", "3", "4", "5"}

    # scene reflects the fix: re-fetching and re-validating finds nothing new
    r2 = client.post(f"/api/traces/{names[0]}/validate")
    assert r2.json()["issue_count"] == data["after_issue_count"]

    # every artifact was written into output/, mirroring the input layout
    out = data["output"]
    adma_out = Path(out["adma_path"])
    annotation_out = Path(out["annotation_path"])
    xodr_out = Path(out["xodr_path"])
    xosc_out = Path(out["xosc_path"])
    report_out = Path(out["report_txt_path"])
    assert adma_out == output_dir / "adma" / "ADMA" / names[0] / "adma.csv"
    assert annotation_out.parent == output_dir / "annotations" / "Annotations"
    assert xodr_out == output_dir / "scenarios" / names[0] / f"{names[0]}.xodr"
    assert xosc_out == output_dir / "scenarios" / names[0] / f"{names[0]}.xosc"
    assert report_out == output_dir / "reports" / names[0] / f"{names[0]}_summary.txt"
    for p in (adma_out, annotation_out, xodr_out, xosc_out, report_out):
        assert p.exists()


def test_batch_fix_predict_404_for_unknown_trace(client_with_corpus):
    client, _names, _output_dir = client_with_corpus
    r = client.post("/api/traces/does-not-exist/batch_fix_predict")
    assert r.status_code == 404


def test_individual_export_endpoints_write_to_output_and_report_the_path(client_with_corpus):
    """Every per-trace export endpoint writes into output/ and reports where
    via JSON -- it never streams the file back for a browser download."""
    client, names, output_dir = client_with_corpus
    trace_id = names[0]

    r = client.get(f"/api/traces/{trace_id}/export/adma")
    assert r.status_code == 200
    adma_path = output_dir / "adma" / "ADMA" / trace_id / "adma.csv"
    assert adma_path.exists()
    assert r.json()["output_path"] == str(adma_path)
    assert "attachment" not in r.headers.get("content-disposition", "")

    r = client.get(f"/api/traces/{trace_id}/export/annotation")
    assert r.status_code == 200
    assert any((output_dir / "annotations" / "Annotations").glob(f"{trace_id}*"))
    assert Path(r.json()["output_path"]).exists()

    r = client.get(f"/api/traces/{trace_id}/export/fixed_trace")
    assert r.status_code == 200
    files = r.json()["files"]
    assert str(adma_path) in files
    assert any((output_dir / "annotations" / "Annotations").glob(f"{trace_id}*"))
    assert all(Path(f).exists() for f in files)

    r = client.get(f"/api/traces/{trace_id}/export/opendrive")
    assert r.status_code == 200
    xodr_path = output_dir / "scenarios" / trace_id / f"{trace_id}.xodr"
    assert xodr_path.exists()
    assert r.json()["output_path"] == str(xodr_path)

    r = client.get(f"/api/traces/{trace_id}/export/openscenario")
    assert r.status_code == 200
    xosc_path = output_dir / "scenarios" / trace_id / f"{trace_id}.xosc"
    assert xosc_path.exists()
    assert r.json()["output_path"] == str(xosc_path)

    r = client.get(f"/api/traces/{trace_id}/export/report", params={"format": "xml"})
    assert r.status_code == 200
    report_path = output_dir / "reports" / trace_id / f"{trace_id}_summary.xml"
    assert report_path.exists()
    assert r.json()["output_path"] == str(report_path)
    assert "<TraceSummary" in report_path.read_text()


def test_export_adp_yaml_writes_to_output_and_flags_the_placeholder_map_key(client_with_corpus):
    import yaml

    client, names, output_dir = client_with_corpus
    trace_id = names[0]

    r = client.get(f"/api/traces/{trace_id}/export/adp_yaml")
    assert r.status_code == 200
    out_path = output_dir / "scenarios" / trace_id / f"{trace_id}.scn.yaml"
    assert out_path.exists()
    data = r.json()
    assert data["output_path"] == str(out_path)
    assert data["map_key"] == trace_id
    assert data["map_key_is_placeholder"] is True
    doc = yaml.safe_load(out_path.read_text())
    assert doc["map"]["key"] == trace_id
    assert doc["agents"][0]["ego"] is not None


def test_export_adp_yaml_with_explicit_map_key(client_with_corpus):
    client, names, _output_dir = client_with_corpus
    trace_id = names[0]

    r = client.get(f"/api/traces/{trace_id}/export/adp_yaml", params={"map_key": "REGISTERED_MAP"})
    assert r.status_code == 200
    data = r.json()
    assert data["map_key"] == "REGISTERED_MAP"
    assert data["map_key_is_placeholder"] is False


def test_list_traces_paginates_and_totals_the_filtered_set(client_with_corpus):
    client, names, _output_dir = client_with_corpus
    sorted_names = sorted(names, key=str.lower)

    r = client.get("/api/traces", params={"limit": 2, "offset": 0})
    assert r.status_code == 200
    data = r.json()
    assert data["trace_ids"] == sorted_names[:2]
    assert data["total"] == 3
    assert data["offset"] == 0

    r = client.get("/api/traces", params={"limit": 2, "offset": 2})
    data = r.json()
    assert data["trace_ids"] == sorted_names[2:]
    assert data["total"] == 3

    # total reflects the *filtered* count, not the whole corpus
    r = client.get("/api/traces", params={"q": sorted_names[0]})
    data = r.json()
    assert data["total"] == 1
    assert data["trace_ids"] == [sorted_names[0]]


def test_browse_dir_lists_subdirectories(tmp_path):
    from fastapi.testclient import TestClient

    import trace_fixer.api as api_module

    (tmp_path / "alpha").mkdir()
    (tmp_path / "beta").mkdir()
    (tmp_path / ".hidden").mkdir()
    (tmp_path / "not_a_dir.txt").write_text("x")

    client = TestClient(api_module.app)
    r = client.get("/api/browse_dir", params={"path": str(tmp_path)})
    assert r.status_code == 200
    data = r.json()
    assert data["path"] == str(tmp_path)
    assert data["dirs"] == ["alpha", "beta"]  # hidden dirs and files excluded
    assert data["parent"] == str(tmp_path.parent)


def test_browse_dir_rejects_non_directory(tmp_path):
    from fastapi.testclient import TestClient

    import trace_fixer.api as api_module

    missing = tmp_path / "does-not-exist"
    client = TestClient(api_module.app)
    r = client.get("/api/browse_dir", params={"path": str(missing)})
    assert r.status_code == 400


def _wait_for_batch_done(client, timeout=10):
    deadline = time.time() + timeout
    status = client.get("/api/batch/all/status").json()
    while status["running"] and time.time() < deadline:
        time.sleep(0.1)
        status = client.get("/api/batch/all/status").json()
    return status


def test_batch_all_processes_every_registered_trace(client_with_corpus):
    client, names, output_dir = client_with_corpus

    r = client.post("/api/batch/all")
    assert r.status_code == 200
    assert r.json()["total"] == 3
    assert r.json()["mode"] == "run"

    status = _wait_for_batch_done(client)
    assert status["running"] is False
    assert status["done"] == 3
    assert status["failed"] == []
    for name in names:
        assert (output_dir / "adma" / "ADMA" / name / "adma.csv").exists()
        assert (output_dir / "reports" / name / f"{name}_summary.txt").exists()


def test_batch_all_rejects_concurrent_start(client_with_corpus):
    client, _names, _output_dir = client_with_corpus

    r1 = client.post("/api/batch/all")
    assert r1.status_code == 200
    r2 = client.post("/api/batch/all")
    assert r2.status_code == 409

    status = _wait_for_batch_done(client)
    assert status["running"] is False


def test_batch_all_rejects_unknown_mode(client_with_corpus):
    client, _names, _output_dir = client_with_corpus
    r = client.post("/api/batch/all", params={"mode": "bogus"})
    assert r.status_code == 400


def test_batch_all_catalog_mode_populates_catalog_without_writing_output(client_with_corpus):
    client, names, output_dir = client_with_corpus

    r = client.post("/api/batch/all", params={"mode": "catalog"})
    assert r.status_code == 200
    status = _wait_for_batch_done(client)
    assert status["done"] == 3
    assert status["failed"] == []

    assert not (output_dir / "adma").exists()  # catalog-only: no fix/predict, nothing written to output/

    r = client.get("/api/catalog")
    assert r.status_code == 200
    data = r.json()
    assert data["total"] == 3
    assert {row["trace_id"] for row in data["rows"]} == set(names)
    for row in data["rows"]:
        assert row["processed_at"] is not None
        assert row["vehicle_count"] == 5
        assert row["first_lat"] is not None


def test_batch_all_fix_catalog_mode_does_both(client_with_corpus):
    client, names, output_dir = client_with_corpus

    r = client.post("/api/batch/all", params={"mode": "run"}, json={"also_build_catalog": True})
    assert r.status_code == 200
    status = _wait_for_batch_done(client)
    assert status["done"] == 3
    assert status["failed"] == []

    for name in names:
        assert (output_dir / "adma" / "ADMA" / name / "adma.csv").exists()

    r = client.get("/api/catalog")
    data = r.json()
    assert data["total"] == 3
    for row in data["rows"]:
        assert row["processed_at"] is not None


def test_batch_all_run_mode_only_writes_the_selected_export_types(client_with_corpus):
    """The GUI's Fix and Export checkboxes map 1:1 onto FixExportOptions --
    confirm unchecked export types genuinely don't get written, not just
    that checked ones do."""
    client, names, output_dir = client_with_corpus

    r = client.post("/api/batch/all", json={
        "export_fixed_trace": False,
        "export_opendrive": False,
        "export_openscenario": False,
        "export_adp_yaml": True,
        "export_report_txt": False,
        "export_report_xml": True,
    })
    assert r.status_code == 200
    _wait_for_batch_done(client)

    assert not (output_dir / "adma").exists()
    assert not (output_dir / "annotations").exists()
    for name in names:
        assert not (output_dir / "scenarios" / name / f"{name}.xodr").exists()
        assert not (output_dir / "scenarios" / name / f"{name}.xosc").exists()
        assert (output_dir / "scenarios" / name / f"{name}.scn.yaml").exists()
        assert not (output_dir / "reports" / name / f"{name}_summary.txt").exists()
        assert (output_dir / "reports" / name / f"{name}_summary.xml").exists()


def test_fix_issues_and_predict_trajectories_are_independently_skippable(client_with_corpus):
    """Exercised via the single-trace endpoint (same _fix_predict_and_write_output
    code path /api/batch/all's "run" mode uses) since it echoes fix_summary/
    predicted back, making the skip actually observable."""
    client, names, _output_dir = client_with_corpus
    trace_id = names[0]

    r = client.post(f"/api/traces/{trace_id}/batch_fix_predict", json={
        "fix_issues": False, "predict_trajectories": False, "export_opendrive": False, "export_openscenario": False,
    })
    assert r.status_code == 200
    data = r.json()
    assert data["fix_summary"] == {}
    assert data["predicted"] == {}


def test_catalog_query_filters_and_tags_endpoint(client_with_corpus):
    client, names, _output_dir = client_with_corpus

    r = client.get("/api/catalog/tags")
    assert r.status_code == 200
    tags = r.json()
    assert "off_road" in tags["issue_categories"]
    assert "cut_in" in tags["phenomena"]

    client.post("/api/batch/all", params={"mode": "catalog"})
    _wait_for_batch_done(client)

    r = client.get("/api/catalog", params={"q": names[0]})
    assert r.json()["total"] == 1

    r = client.get("/api/catalog/stats")
    assert r.json() == {"total": 3, "processed": 3}


def test_scan_registers_identity_rows_in_catalog(client_with_corpus):
    """The corpus fixture already scans as part of setup -- confirm that
    alone (before any batch run) creates catalog rows, just unprocessed."""
    client, names, _output_dir = client_with_corpus
    r = client.get("/api/catalog")
    data = r.json()
    assert data["total"] == 3
    assert {row["trace_id"] for row in data["rows"]} == set(names)
    assert all(row["processed_at"] is None for row in data["rows"])


def test_export_opendrive_without_enrich_param_is_unaffected(client_with_corpus):
    client, names, _output_dir = client_with_corpus
    r = client.get(f"/api/traces/{names[0]}/export/opendrive")
    assert r.status_code == 200
    data = r.json()
    assert data["enrichment"] is None
    assert data["enrichment_requested"] is None


def test_export_opendrive_enrich_falls_back_gracefully_on_failure(client_with_corpus, monkeypatch):
    """No real network call: simulate the provider failing, and confirm
    the export still succeeds (falls back to the offline result) rather
    than erroring -- the whole point of fetch_enrichment's contract."""
    import trace_fixer.export.map_enrichment as map_enrichment

    def boom(*a, **kw):
        raise ConnectionError("simulated network failure")

    monkeypatch.setattr(map_enrichment.OSMOverpassProvider, "fetch", boom)

    client, names, _output_dir = client_with_corpus
    r = client.get(f"/api/traces/{names[0]}/export/opendrive", params={"enrich": "osm"})
    assert r.status_code == 200
    data = r.json()
    assert data["enrichment"] is None
    assert data["enrichment_requested"] == "osm"
    assert "simulated network failure" in data["enrichment_error"]


def test_export_openscenario_writes_xosc_referencing_canonical_xodr_filename(client_with_corpus):
    client, names, output_dir = client_with_corpus
    trace_id = names[0]
    r = client.get(f"/api/traces/{trace_id}/export/openscenario")
    assert r.status_code == 200
    xosc_path = output_dir / "scenarios" / trace_id / f"{trace_id}.xosc"
    assert xosc_path.exists()
    assert f"{trace_id}.xodr" in xosc_path.read_text()


def test_map_overlay_converts_ways_into_the_scene_local_frame(client_with_corpus, monkeypatch):
    """No real network call. The returned points must be in the same local
    (x, y) frame the rest of the scene JSON uses, not raw lat/lon -- that's
    the whole point of doing the conversion server-side."""
    import trace_fixer.api as api_module
    from trace_fixer.export.map_enrichment import BBox, MapEnrichmentResult, MapWay

    client, names, _output_dir = client_with_corpus
    trace = api_module.store.get(names[0])
    lat0, lon0 = trace.ego.poses[0].lat_deg, trace.ego.poses[0].lon_deg

    fake_way = MapWay(id=7, points=[(lat0, lon0), (lat0 + 0.001, lon0)], tags={"name": "Test Rd"})
    fake_result = MapEnrichmentResult(provider="osm", bbox=BBox(0, 0, 0, 0), ways=[fake_way])
    monkeypatch.setattr(api_module, "fetch_enrichment", lambda *a, **kw: (fake_result, None))

    r = client.get(f"/api/traces/{names[0]}/map_overlay")
    assert r.status_code == 200
    data = r.json()
    assert data["error"] is None
    assert len(data["ways"]) == 1
    way = data["ways"][0]
    assert way["name"] == "Test Rd"
    # the first point is the trace's own anchor -> must land at local (0, 0)
    assert way["points"][0] == pytest.approx([0.0, 0.0], abs=1e-6)
    assert way["points"][1][1] > 50  # ~0.001 deg north is ~111m -> a real local offset, not a raw lat/lon


def test_map_overlay_reports_error_without_failing(client_with_corpus, monkeypatch):
    import trace_fixer.api as api_module

    client, names, _output_dir = client_with_corpus
    monkeypatch.setattr(api_module, "fetch_enrichment", lambda *a, **kw: (None, "simulated failure"))

    r = client.get(f"/api/traces/{names[0]}/map_overlay")
    assert r.status_code == 200
    data = r.json()
    assert data["ways"] == []
    assert data["error"] == "simulated failure"
