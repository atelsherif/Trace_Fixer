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
    api_module._variant_store = {}  # isolate generated variants across tests too

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


def test_predict_preview_reports_issue_impact_without_committing(client_with_corpus):
    """preview=true must answer 'what would this introduce' without
    actually adding the prediction, so the GUI can warn before commit."""
    client, names, _output_dir = client_with_corpus
    trace_id = names[0]

    r = client.post(f"/api/traces/{trace_id}/predict", json={"preview": True})
    assert r.status_code == 200
    data = r.json()
    assert "scene" not in data  # preview never returns a scene -- nothing was committed
    assert set(data["added"].keys()) == {"1", "2", "3", "4", "5"}
    assert data["new_issue_count"] == data["after_issue_count"] - data["before_issue_count"]

    # nothing was actually added: a fresh scene has no synthetic observations
    scene = client.get(f"/api/traces/{trace_id}/scene").json()
    assert not any(o["synthetic"] for v in scene["vehicles"] for o in v["observations"])

    # the real (committing) call still works normally afterward, and
    # reports the same before/after/new issue counts a preview would --
    # no separate round-trip needed to see what committing just did.
    r2 = client.post(f"/api/traces/{trace_id}/predict", json={})
    assert r2.status_code == 200
    data2 = r2.json()
    assert "scene" in data2
    assert data2["new_issue_count"] == data2["after_issue_count"] - data2["before_issue_count"]
    scene2 = client.get(f"/api/traces/{trace_id}/scene").json()
    assert any(o["synthetic"] for v in scene2["vehicles"] for o in v["observations"])


def test_predict_horizon_m_caps_distance(client_with_corpus):
    client, names, _output_dir = client_with_corpus
    trace_id = names[0]

    r = client.post(f"/api/traces/{trace_id}/predict", json={"horizon_s": 10.0, "horizon_m": 5.0})
    assert r.status_code == 200
    added_capped = r.json()["added"]

    client.post(f"/api/traces/{trace_id}/predict/clear")
    r2 = client.post(f"/api/traces/{trace_id}/predict", json={"horizon_s": 10.0})
    added_uncapped = r2.json()["added"]

    # every vehicle predicted in both calls got fewer (or equal) synthetic
    # observations once a 5m cap was added on top of the same time budget
    for vid in added_capped:
        for direction in added_capped[vid]:
            assert added_capped[vid][direction] <= added_uncapped[vid][direction]


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
    # ...under one provenance suffix shared by every artifact of this run,
    # so the whole set stays identifiable as "the fixed+predicted export"
    sfx = "__fixed__predicted"
    assert data["provenance"] == "fixed + predicted"
    assert adma_out == output_dir / "adma" / "ADMA" / names[0] / f"adma{sfx}.csv"
    assert annotation_out.parent == output_dir / "annotations" / "Annotations"
    assert annotation_out.name.endswith(f"{sfx}.xml")
    assert xodr_out == output_dir / "scenarios" / names[0] / f"{names[0]}{sfx}.xodr"
    assert xosc_out == output_dir / "scenarios" / names[0] / f"{names[0]}{sfx}.xosc"
    assert report_out == output_dir / "reports" / names[0] / f"{names[0]}_summary{sfx}.txt"
    for p in (adma_out, annotation_out, xodr_out, xosc_out, report_out):
        assert p.exists()
    # the .xosc must name the .xodr this same run wrote, not a bare one
    assert f"{names[0]}{sfx}.xodr" in xosc_out.read_text()


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


def test_exports_of_different_trace_states_do_not_overwrite_each_other(client_with_corpus):
    """Exporting, applying fixes, then exporting again must leave two files
    that say which is which -- not one file of ambiguous origin."""
    client, names, output_dir = client_with_corpus
    trace_id = names[0]

    r = client.get(f"/api/traces/{trace_id}/export/opendrive")
    assert r.json()["provenance"] == "original"

    client.post(f"/api/traces/{trace_id}/validate")
    client.post(f"/api/traces/{trace_id}/fix")
    r = client.get(f"/api/traces/{trace_id}/export/opendrive")
    assert r.json()["provenance"] == "fixed"

    scenarios = output_dir / "scenarios" / trace_id
    assert (scenarios / f"{trace_id}.xodr").exists()
    assert (scenarios / f"{trace_id}__fixed.xodr").exists()


def test_exports_endpoint_lists_what_was_written_newest_first(client_with_corpus):
    client, names, _output_dir = client_with_corpus
    trace_id = names[0]

    assert client.get("/api/exports").json()["exports"] == []

    client.get(f"/api/traces/{trace_id}/export/adma")
    client.get(f"/api/traces/{trace_id}/export/opendrive")
    client.get(f"/api/traces/{names[1]}/export/adma")

    entries = client.get("/api/exports").json()["exports"]
    assert [e["kind"] for e in entries] == ["adma", "opendrive", "adma"]
    assert [e["trace_id"] for e in entries] == [names[1], trace_id, trace_id]
    assert all(e["provenance"] == "original" and e["exported_at"] and e["files"] for e in entries)

    # ...and filters to one trace, so the GUI can show "this trace's exports"
    mine = client.get("/api/exports", params={"trace_id": trace_id}).json()["exports"]
    assert [e["kind"] for e in mine] == ["opendrive", "adma"]


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
        assert (output_dir / "adma" / "ADMA" / name / "adma__fixed__predicted.csv").exists()
        assert (output_dir / "reports" / name / f"{name}_summary__fixed__predicted.txt").exists()


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
        assert (output_dir / "adma" / "ADMA" / name / "adma__fixed__predicted.csv").exists()

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

    sfx = "__fixed__predicted"
    assert not (output_dir / "adma").exists()
    assert not (output_dir / "annotations").exists()
    for name in names:
        assert not (output_dir / "scenarios" / name / f"{name}{sfx}.xodr").exists()
        assert not (output_dir / "scenarios" / name / f"{name}{sfx}.xosc").exists()
        assert (output_dir / "scenarios" / name / f"{name}{sfx}.scn.yaml").exists()
        assert not (output_dir / "reports" / name / f"{name}_summary{sfx}.txt").exists()
        assert (output_dir / "reports" / name / f"{name}_summary{sfx}.xml").exists()


def test_batch_all_run_mode_threads_enrich_through_to_opendrive_only(client_with_corpus, monkeypatch):
    """The GUI's batch-panel OSM checkbox sets `enrich`, which must only
    take effect when OpenDRIVE is also being written -- confirm the
    provider is actually invoked once per trace when both are set, and not
    invoked at all when OpenDRIVE is unchecked even with enrich set."""
    import trace_fixer.export.map_enrichment as map_enrichment

    calls = []

    def fake_fetch(self, bbox, timeout=map_enrichment.DEFAULT_TIMEOUT_S):
        calls.append(1)
        return map_enrichment.MapEnrichmentResult(provider="osm", bbox=bbox, ways=[])

    monkeypatch.setattr(map_enrichment.OSMOverpassProvider, "fetch", fake_fetch)

    client, names, _output_dir = client_with_corpus
    r = client.post("/api/batch/all", json={"export_opendrive": False, "enrich": "osm"})
    assert r.status_code == 200
    _wait_for_batch_done(client)
    assert calls == []

    r = client.post("/api/batch/all", json={"export_opendrive": True, "enrich": "osm"})
    assert r.status_code == 200
    _wait_for_batch_done(client)
    # >=1 rather than ==len(names): this fixture's traces are copies of the
    # same sample data, so they share a disk cache key (see
    # export.map_enrichment) and only the first genuinely hits the provider.
    assert len(calls) >= 1


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


# ---------- Alternative scenarios (variants) ----------

def test_generate_variants_preset_mode_returns_a_summary_list(client_with_corpus):
    client, names, _output_dir = client_with_corpus
    trace_id = names[0]

    r = client.post(f"/api/traces/{trace_id}/variants/generate", json={"count": 5, "mode": "preset"})
    assert r.status_code == 200
    variants = r.json()["variants"]
    assert len(variants) == 5
    for v in variants:
        assert v["variant_id"]
        assert v["name"]
        assert v["vehicle_id"] in (1, 2, 3, 4, 5)
        assert v["source"] in ("preset", "randomized")


def test_list_variants_reflects_the_last_generate_call(client_with_corpus):
    client, names, _output_dir = client_with_corpus
    trace_id = names[0]

    assert client.get(f"/api/traces/{trace_id}/variants").json()["variants"] == []

    client.post(f"/api/traces/{trace_id}/variants/generate", json={"count": 3, "mode": "preset"})
    first = client.get(f"/api/traces/{trace_id}/variants").json()["variants"]
    assert len(first) == 3

    # a second generate call replaces, not accumulates
    client.post(f"/api/traces/{trace_id}/variants/generate", json={"count": 2, "mode": "randomized", "seed": 1})
    second = client.get(f"/api/traces/{trace_id}/variants").json()["variants"]
    assert len(second) == 2


def test_generate_variants_randomized_mode_is_reproducible_with_a_seed(client_with_corpus):
    client, names, _output_dir = client_with_corpus
    trace_id = names[0]

    r1 = client.post(f"/api/traces/{trace_id}/variants/generate", json={"count": 3, "mode": "randomized", "seed": 7})
    r2 = client.post(f"/api/traces/{trace_id}/variants/generate", json={"count": 3, "mode": "randomized", "seed": 7})
    kinds1 = [(v["kind"], v["vehicle_id"]) for v in r1.json()["variants"]]
    kinds2 = [(v["kind"], v["vehicle_id"]) for v in r2.json()["variants"]]
    assert kinds1 == kinds2


def test_generate_variants_rejects_an_unknown_mode(client_with_corpus):
    client, names, _output_dir = client_with_corpus
    r = client.post(f"/api/traces/{names[0]}/variants/generate", json={"mode": "bogus"})
    assert r.status_code == 400


def test_variant_scene_and_exports_round_trip(client_with_corpus):
    client, names, output_dir = client_with_corpus
    trace_id = names[0]

    variants = client.post(f"/api/traces/{trace_id}/variants/generate", json={"count": 1}).json()["variants"]
    variant_id = variants[0]["variant_id"]

    r = client.get(f"/api/traces/{trace_id}/variants/{variant_id}/scene")
    assert r.status_code == 200
    assert r.json()["vehicles"]

    r = client.get(f"/api/traces/{trace_id}/variants/{variant_id}/export/fixed_trace")
    assert r.status_code == 200
    files = r.json()["files"]
    assert all(Path(f).exists() for f in files)
    assert all(variant_id in f for f in files)

    r = client.get(f"/api/traces/{trace_id}/variants/{variant_id}/export/adp_yaml")
    assert r.status_code == 200
    assert Path(r.json()["output_path"]).exists()

    r = client.get(f"/api/traces/{trace_id}/variants/{variant_id}/export/report", params={"format": "xml"})
    assert r.status_code == 200
    assert Path(r.json()["output_path"]).exists()

    r = client.get(f"/api/traces/{trace_id}/variants/{variant_id}/export/bogus_artifact")
    assert r.status_code == 400


def test_variant_endpoints_404_for_an_unknown_variant(client_with_corpus):
    client, names, _output_dir = client_with_corpus
    trace_id = names[0]
    client.post(f"/api/traces/{trace_id}/variants/generate", json={"count": 1})

    r = client.get(f"/api/traces/{trace_id}/variants/does-not-exist/scene")
    assert r.status_code == 404


# ---------- Vehicle point-of-view export ----------

def test_pov_export_openscenario_and_adp_yaml(client_with_corpus):
    client, names, output_dir = client_with_corpus
    trace_id = names[0]

    r = client.get(f"/api/traces/{trace_id}/export/openscenario", params={"pov_vehicle_id": 1})
    assert r.status_code == 200
    pov_xosc = output_dir / "scenarios" / trace_id / f"{trace_id}__pov1.xosc"
    assert pov_xosc.exists()
    assert Path(r.json()["output_path"]) == pov_xosc

    # the normal (non-POV) file is untouched by the POV export
    assert not (output_dir / "scenarios" / trace_id / f"{trace_id}.xosc").exists()

    r = client.get(f"/api/traces/{trace_id}/export/adp_yaml", params={"pov_vehicle_id": 1})
    assert r.status_code == 200
    pov_yaml = output_dir / "scenarios" / trace_id / f"{trace_id}__pov1.scn.yaml"
    assert pov_yaml.exists()
    assert Path(r.json()["output_path"]) == pov_yaml


def test_pov_export_400_for_an_unknown_vehicle_id(client_with_corpus):
    client, names, _output_dir = client_with_corpus
    trace_id = names[0]

    r = client.get(f"/api/traces/{trace_id}/export/openscenario", params={"pov_vehicle_id": 9999})
    assert r.status_code == 400

    r = client.get(f"/api/traces/{trace_id}/export/adp_yaml", params={"pov_vehicle_id": 9999})
    assert r.status_code == 400
