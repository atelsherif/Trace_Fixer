"""API-level tests for trace navigation (neighbor) and batch fix+predict.

Uses a small real corpus (copies of the bundled sample trace under a few
different names) rather than placeholder files, since these endpoints need
traces that actually parse.
"""
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
    report_out = Path(out["report_path"])
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

    r = client.get(f"/api/traces/{trace_id}/export/scenario")
    assert r.status_code == 200
    xodr_path = output_dir / "scenarios" / trace_id / f"{trace_id}.xodr"
    xosc_path = output_dir / "scenarios" / trace_id / f"{trace_id}.xosc"
    assert xodr_path.exists()
    assert xosc_path.exists()
    data = r.json()
    assert data["output_path"] == str(xodr_path.parent)
    assert set(data["files"]) == {str(xodr_path), str(xosc_path)}

    r = client.get(f"/api/traces/{trace_id}/export/report", params={"format": "xml"})
    assert r.status_code == 200
    report_path = output_dir / "reports" / trace_id / f"{trace_id}_summary.xml"
    assert report_path.exists()
    assert r.json()["output_path"] == str(report_path)
    assert "<TraceSummary" in report_path.read_text()
