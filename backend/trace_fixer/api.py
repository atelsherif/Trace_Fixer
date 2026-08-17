from __future__ import annotations

import io
import tempfile
import zipfile
from pathlib import Path

from fastapi import FastAPI, File, HTTPException, UploadFile
from fastapi.responses import JSONResponse, PlainTextResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from trace_fixer.export.adma_writer import write_adma_csv
from trace_fixer.export.annotation_writer import write_annotation_xml
from trace_fixer.export.batch_output import write_batch_output
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

app = FastAPI(title="Trace Fixer")
store = TraceStore(traces_dir=DATA_DIR)
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)


def _get_trace_or_404(trace_id: str):
    try:
        return store.get(trace_id)
    except KeyError:
        raise HTTPException(status_code=404, detail=f"Unknown trace_id '{trace_id}'")


@app.get("/api/traces")
def list_traces(q: str | None = None, limit: int = 200):
    return {"trace_ids": store.list_ids(query=q, limit=limit), "total": store.count()}


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


@app.post("/api/traces/{trace_id}/batch_fix_predict")
def batch_fix_predict(trace_id: str, req: BatchFixPredictRequest = BatchFixPredictRequest()):
    """One-shot validate -> fix -> predict -> re-validate for a single trace,
    then writes the corrected ADMA + annotation files into output/ (mirroring
    the input corpus layout -- see export.batch_output) so processing many
    traces produces a ready-to-use output corpus. No scene payload in the
    response -- meant to be called in a loop over many trace_ids (see the
    GUI's multi-select "batch" action) without paying for a full scene JSON
    build on every one.
    """
    try:
        trace = store.get(trace_id)
    except KeyError:
        raise HTTPException(status_code=404, detail=f"Unknown trace_id '{trace_id}'")
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
    return {
        "trace_id": trace_id,
        "before_issue_count": len(before),
        "after_issue_count": len(after),
        "fix_summary": fix_summary,
        "predicted": added,
        "output": output_paths,
    }


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


@app.get("/api/traces/{trace_id}/export/adma")
def export_adma(trace_id: str):
    trace = _get_trace_or_404(trace_id)
    with tempfile.NamedTemporaryFile(suffix=".csv", delete=False) as tmp:
        tmp_path = Path(tmp.name)
    try:
        write_adma_csv(trace.ego, tmp_path)
        content = tmp_path.read_text()
    finally:
        tmp_path.unlink(missing_ok=True)
    return PlainTextResponse(
        content,
        media_type="text/csv",
        headers={"Content-Disposition": f'attachment; filename="{trace_id}_adma_fixed.csv"'},
    )


@app.get("/api/traces/{trace_id}/export/annotation")
def export_annotation(trace_id: str, include_predictions: bool = True):
    trace = _get_trace_or_404(trace_id)
    original_path = store.original_annotation_path(trace_id)
    with tempfile.NamedTemporaryFile(suffix=".xml", delete=False) as tmp:
        tmp_path = Path(tmp.name)
    try:
        write_annotation_xml(trace, original_path, tmp_path, include_predictions=include_predictions)
        content = tmp_path.read_bytes()
    finally:
        tmp_path.unlink(missing_ok=True)
    return StreamingResponse(
        io.BytesIO(content),
        media_type="application/xml",
        headers={"Content-Disposition": f'attachment; filename="{trace_id}_annotation_fixed.xml"'},
    )


@app.get("/api/traces/{trace_id}/export/scenario")
def export_scenario(trace_id: str):
    trace = _get_trace_or_404(trace_id)
    xodr_filename = f"{trace_id}.xodr"
    xodr = generate_opendrive(trace)
    xosc = generate_openscenario(trace, xodr_filename)

    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as zf:
        zf.writestr(xodr_filename, xodr)
        zf.writestr(f"{trace_id}.xosc", xosc)
    buf.seek(0)
    return StreamingResponse(
        buf,
        media_type="application/zip",
        headers={"Content-Disposition": f'attachment; filename="{trace_id}_scenario.zip"'},
    )


@app.get("/api/traces/{trace_id}/export/report")
def export_report(trace_id: str, format: str = "txt"):
    trace = _get_trace_or_404(trace_id)
    if format == "xml":
        content = generate_xml_report(trace)
        media_type, ext = "application/xml", "xml"
    elif format == "txt":
        content = generate_txt_report(trace)
        media_type, ext = "text/plain", "txt"
    else:
        raise HTTPException(status_code=400, detail="format must be 'txt' or 'xml'")
    return PlainTextResponse(
        content,
        media_type=media_type,
        headers={"Content-Disposition": f'attachment; filename="{trace_id}_problem_report.{ext}"'},
    )


if FRONTEND_DIR.exists():
    app.mount("/", StaticFiles(directory=str(FRONTEND_DIR), html=True), name="frontend")
