from __future__ import annotations

import threading
from pathlib import Path

from fastapi import FastAPI, File, HTTPException, UploadFile
from fastapi.responses import JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from trace_fixer.browse import list_subdirectories
from trace_fixer.export.adma_writer import write_adma_csv
from trace_fixer.export.annotation_writer import write_annotation_xml
from trace_fixer.export.batch_output import (
    adma_output_path,
    annotation_output_path,
    report_output_path,
    scenario_output_paths,
    write_batch_output,
)
from trace_fixer.export.opendrive import generate_opendrive
from trace_fixer.export.openscenario import generate_openscenario
from trace_fixer.export.report import generate_txt_report, generate_xml_report
from trace_fixer.geo.populate import populate_global_coords
from trace_fixer.prediction.extrapolate import clear_predictions, predict_all
from trace_fixer.scene import build_scene_json
from trace_fixer.store import TraceStore
from trace_fixer.validation.checks import run_validation
from trace_fixer.validation.fixes import apply_fixes

REPO_ROOT = Path(__file__).resolve().parents[2]
DATA_DIR = REPO_ROOT / "data" / "traces"
OUTPUT_DIR = REPO_ROOT / "output"
FRONTEND_DIR = REPO_ROOT / "frontend"

app = FastAPI(title="PreTwin")
store = TraceStore(traces_dir=DATA_DIR)
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)


def _get_trace_or_404(trace_id: str):
    try:
        return store.get(trace_id)
    except KeyError:
        raise HTTPException(status_code=404, detail=f"Unknown trace_id '{trace_id}'")


@app.get("/api/traces")
def list_traces(q: str | None = None, limit: int = 200, offset: int = 0):
    matching = store.list_ids(query=q)  # unlimited -- filtered, so pagination math is against the real total
    return {"trace_ids": matching[offset : offset + limit], "total": len(matching), "offset": offset}


@app.get("/api/browse_dir")
def browse_dir(path: str | None = None):
    """Lists the subdirectories of `path` (or the server's home directory)
    for the Scan directory panel's folder browser -- see browse.py.
    """
    try:
        return list_subdirectories(path)
    except NotADirectoryError:
        raise HTTPException(status_code=400, detail=f"Not a directory (on the server running this app): {path}")


@app.get("/api/traces/{trace_id}/neighbor")
def trace_neighbor(trace_id: str, direction: str, q: str | None = None):
    if direction not in ("prev", "next"):
        raise HTTPException(status_code=400, detail="direction must be 'prev' or 'next'")
    ids = store.list_ids(query=q)  # unlimited -- the full matching list, so this scales with the corpus
    if trace_id not in ids:
        raise HTTPException(status_code=404, detail=f"'{trace_id}' is not in the current trace list")
    if len(ids) < 2:
        return {"trace_id": trace_id, "index": 0, "total": len(ids)}
    idx = ids.index(trace_id)
    new_idx = (idx - 1) % len(ids) if direction == "prev" else (idx + 1) % len(ids)
    return {"trace_id": ids[new_idx], "index": new_idx, "total": len(ids)}


class UploadResponse(BaseModel):
    trace_id: str


@app.post("/api/traces", response_model=UploadResponse)
async def upload_trace(
    adma_file: UploadFile = File(...),
    annotation_file: UploadFile = File(...),
    name: str | None = None,
):
    adma_bytes = await adma_file.read()
    annotation_bytes = await annotation_file.read()
    if not adma_bytes or not annotation_bytes:
        raise HTTPException(status_code=400, detail="Both files must be non-empty")
    trace_id = store.add_from_bytes(name or adma_file.filename or "trace", adma_bytes, annotation_bytes)
    return UploadResponse(trace_id=trace_id)


class ScanRequest(BaseModel):
    path: str


class ScanResponse(BaseModel):
    adma_found: int
    xml_found: int
    matched: int
    unmatched_adma_count: int
    unmatched_xml_count: int
    total_traces: int


@app.post("/api/traces/scan", response_model=ScanResponse)
def scan_directory(req: ScanRequest):
    root = Path(req.path).expanduser()
    if not root.is_dir():
        raise HTTPException(status_code=400, detail=f"Not a directory (on the server running this app): {root}")
    result = store.scan_directory(root)
    return ScanResponse(
        adma_found=result.adma_found,
        xml_found=result.xml_found,
        matched=len(result.matched),
        unmatched_adma_count=result.unmatched_adma_count,
        unmatched_xml_count=result.unmatched_xml_count,
        total_traces=store.count(),
    )


@app.get("/api/traces/{trace_id}/scene")
def get_scene(trace_id: str):
    trace = _get_trace_or_404(trace_id)
    return JSONResponse(build_scene_json(trace))


@app.post("/api/traces/{trace_id}/validate")
def validate(trace_id: str):
    trace = _get_trace_or_404(trace_id)
    issues = run_validation(trace)
    return {"issue_count": len(issues), "scene": build_scene_json(trace)}


@app.post("/api/traces/{trace_id}/fix")
def fix(trace_id: str):
    trace = _get_trace_or_404(trace_id)
    if not trace.issues:
        run_validation(trace)
    summary = apply_fixes(trace)
    return {"summary": summary, "scene": build_scene_json(trace)}


class PredictRequest(BaseModel):
    horizon_s: float = 4.0
    step_s: float = 0.2
    backward: bool = True
    forward: bool = True


class BatchFixPredictRequest(PredictRequest):
    include_predictions_in_output: bool = True


@app.post("/api/traces/{trace_id}/predict")
def predict(trace_id: str, req: PredictRequest = PredictRequest()):
    trace = _get_trace_or_404(trace_id)
    added = predict_all(
        trace, horizon_s=req.horizon_s, step_s=req.step_s, backward=req.backward, forward=req.forward
    )
    run_validation(trace)
    return {"added": added, "scene": build_scene_json(trace)}


@app.post("/api/traces/{trace_id}/predict/clear")
def predict_clear(trace_id: str):
    trace = _get_trace_or_404(trace_id)
    clear_predictions(trace)
    run_validation(trace)
    return {"scene": build_scene_json(trace)}


def _fix_predict_and_write_output(trace_id: str, req: BatchFixPredictRequest) -> dict:
    """validate -> fix -> predict -> re-validate for one trace, then writes
    every artifact (corrected ADMA + annotation, OpenDRIVE + OpenSCENARIO,
    and a trace summary report) into output/, mirroring the input corpus
    layout for ADMA/annotation -- see export.batch_output. Shared by the
    single-trace endpoint below and the "fix + predict ALL" background job.
    """
    trace = store.get(trace_id)
    before = run_validation(trace)
    fix_summary = apply_fixes(trace)
    added = predict_all(
        trace, horizon_s=req.horizon_s, step_s=req.step_s, backward=req.backward, forward=req.forward
    )
    after = run_validation(trace)

    output_paths = write_batch_output(
        trace,
        store.original_annotation_path(trace_id),
        OUTPUT_DIR,
        include_predictions=req.include_predictions_in_output,
    )
    xodr_path, xosc_path = scenario_output_paths(trace_id, OUTPUT_DIR)
    xodr_path.write_text(generate_opendrive(trace))
    xosc_path.write_text(generate_openscenario(trace, xodr_path.name))
    output_paths["xodr_path"] = str(xodr_path)
    output_paths["xosc_path"] = str(xosc_path)

    report_path = report_output_path(trace_id, OUTPUT_DIR, "txt")
    report_path.write_text(generate_txt_report(trace))
    output_paths["report_path"] = str(report_path)

    return {
        "trace_id": trace_id,
        "before_issue_count": len(before),
        "after_issue_count": len(after),
        "fix_summary": fix_summary,
        "predicted": added,
        "output": output_paths,
    }


@app.post("/api/traces/{trace_id}/batch_fix_predict")
def batch_fix_predict(trace_id: str, req: BatchFixPredictRequest = BatchFixPredictRequest()):
    """No scene payload in the response -- meant to be called in a loop over
    many trace_ids (see the GUI's multi-select "batch" action) without
    paying for a full scene JSON build on every one.
    """
    try:
        return _fix_predict_and_write_output(trace_id, req)
    except KeyError:
        raise HTTPException(status_code=404, detail=f"Unknown trace_id '{trace_id}'")


_batch_all_lock = threading.Lock()
_batch_all_state: dict = {
    "running": False,
    "total": 0,
    "done": 0,
    "current": None,
    "failed": [],
}


def _run_batch_all(trace_ids: list[str]) -> None:
    req = BatchFixPredictRequest()
    for trace_id in trace_ids:
        with _batch_all_lock:
            _batch_all_state["current"] = trace_id
        try:
            _fix_predict_and_write_output(trace_id, req)
        except Exception as exc:  # noqa: BLE001 -- one bad trace must not stop the run
            with _batch_all_lock:
                _batch_all_state["failed"].append({"trace_id": trace_id, "error": str(exc)})
        finally:
            store.evict(trace_id)  # bounds memory across a corpus of thousands
            with _batch_all_lock:
                _batch_all_state["done"] += 1
    with _batch_all_lock:
        _batch_all_state["running"] = False
        _batch_all_state["current"] = None


@app.post("/api/batch/all")
def start_batch_all():
    """Kicks off fix + predict for *every* registered trace (not just the
    page currently shown in the trace picker) in a background thread, and
    returns immediately -- poll /api/batch/all/status for progress. Meant
    for corpora too large to comfortably multi-select in the GUI.
    """
    with _batch_all_lock:
        if _batch_all_state["running"]:
            raise HTTPException(status_code=409, detail="A batch run is already in progress")
        trace_ids = store.list_ids()
        _batch_all_state.update(running=True, total=len(trace_ids), done=0, current=None, failed=[])
    threading.Thread(target=_run_batch_all, args=(trace_ids,), daemon=True).start()
    return {"started": True, "total": len(trace_ids)}


@app.get("/api/batch/all/status")
def batch_all_status():
    with _batch_all_lock:
        return dict(_batch_all_state)


class SyncOffsetRequest(BaseModel):
    offset_us: int


@app.post("/api/traces/{trace_id}/sync_offset")
def set_sync_offset(trace_id: str, req: SyncOffsetRequest):
    trace = _get_trace_or_404(trace_id)
    trace.sync_offset_us = req.offset_us
    clear_predictions(trace)  # synthetic points are anchored to the pre-change ego alignment
    populate_global_coords(trace)
    run_validation(trace)
    return {"scene": build_scene_json(trace)}


@app.post("/api/traces/{trace_id}/reset")
def reset(trace_id: str):
    trace = store.reload(trace_id)
    return {"scene": build_scene_json(trace)}


def _relative_output_path(path: Path) -> str:
    """Display-friendly path: relative to the repo when possible (the
    normal case), or absolute (e.g. under pytest's tmp_path) otherwise.
    """
    try:
        return str(path.relative_to(REPO_ROOT))
    except ValueError:
        return str(path)


@app.get("/api/traces/{trace_id}/export/adma")
def export_adma(trace_id: str):
    """Writes the corrected ADMA CSV into output/ and reports where. Does
    *not* stream the file back -- every export lands in the project's
    output/ directory rather than the browser's downloads folder; see
    export.batch_output for the layout.
    """
    trace = _get_trace_or_404(trace_id)
    out_path = adma_output_path(trace_id, OUTPUT_DIR)
    write_adma_csv(trace.ego, out_path)
    return {"output_path": _relative_output_path(out_path)}


@app.get("/api/traces/{trace_id}/export/annotation")
def export_annotation(trace_id: str, include_predictions: bool = True):
    trace = _get_trace_or_404(trace_id)
    original_path = store.original_annotation_path(trace_id)
    out_path = annotation_output_path(trace_id, original_path, OUTPUT_DIR)
    write_annotation_xml(trace, original_path, out_path, include_predictions=include_predictions)
    return {"output_path": _relative_output_path(out_path)}


@app.get("/api/traces/{trace_id}/export/scenario")
def export_scenario(trace_id: str):
    trace = _get_trace_or_404(trace_id)
    xodr_path, xosc_path = scenario_output_paths(trace_id, OUTPUT_DIR)
    xodr_path.write_text(generate_opendrive(trace))
    xosc_path.write_text(generate_openscenario(trace, xodr_path.name))
    return {
        "output_path": _relative_output_path(xodr_path.parent),
        "files": [_relative_output_path(xodr_path), _relative_output_path(xosc_path)],
    }


@app.get("/api/traces/{trace_id}/export/report")
def export_report(trace_id: str, format: str = "txt"):
    trace = _get_trace_or_404(trace_id)
    if format == "xml":
        content = generate_xml_report(trace)
    elif format == "txt":
        content = generate_txt_report(trace)
    else:
        raise HTTPException(status_code=400, detail="format must be 'txt' or 'xml'")
    out_path = report_output_path(trace_id, OUTPUT_DIR, format)
    out_path.write_text(content)
    return {"output_path": _relative_output_path(out_path)}


if FRONTEND_DIR.exists():
    app.mount("/", StaticFiles(directory=str(FRONTEND_DIR), html=True), name="frontend")
