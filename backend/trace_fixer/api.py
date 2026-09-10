from __future__ import annotations

import json
import threading
from pathlib import Path

from fastapi import FastAPI, File, HTTPException, Query, UploadFile
from fastapi.responses import JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from trace_fixer import catalog
from trace_fixer.browse import list_subdirectories
from trace_fixer.export.adma_writer import write_adma_csv
from trace_fixer.export.annotation_writer import write_annotation_xml
from trace_fixer.export.adp_yaml import generate_adp_scenario_yaml
from trace_fixer.export.batch_output import (
    adma_output_path,
    adp_yaml_output_path,
    annotation_output_path,
    MANIFEST_FILENAME,
    provenance_suffix,
    record_export,
    report_output_path,
    scenario_output_paths,
    write_batch_output,
)
from trace_fixer.export.map_enrichment import fetch_enrichment
from trace_fixer.export.opendrive import generate_opendrive
from trace_fixer.export.openscenario import generate_openscenario
from trace_fixer.export.report import generate_txt_report, generate_xml_report
from trace_fixer.geo.populate import populate_global_coords
from trace_fixer.geo.transform import latlon_to_local
from trace_fixer.prediction.extrapolate import clear_predictions, predict_all
from trace_fixer.scene import build_scene_json
from trace_fixer.store import TraceStore
from trace_fixer.validation.checks import run_validation
from trace_fixer.validation.fixes import apply_fixes
from trace_fixer.variants import Variant, generate_preset_variants, generate_randomized_variants

REPO_ROOT = Path(__file__).resolve().parents[2]
DATA_DIR = REPO_ROOT / "data" / "traces"
OUTPUT_DIR = REPO_ROOT / "output"
FRONTEND_DIR = REPO_ROOT / "frontend"

app = FastAPI(title="PreTwinner")
store = TraceStore(traces_dir=DATA_DIR)
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)


def _catalog_connect():
    """A fresh connection per call rather than one shared across threads --
    see catalog.connect's docstring. Cheap: SQLite connection setup is
    negligible next to the trace parsing this always sits alongside. Reads
    OUTPUT_DIR at call time (not a module-level constant) so tests that
    point OUTPUT_DIR at an isolated tmp_path get an isolated catalog too.
    """
    return catalog.connect(OUTPUT_DIR / "catalog.sqlite")


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
    # Diagnostics: a corpus that scans to a surprisingly low number is
    # otherwise impossible to explain from the GUI (see scan.py).
    adma_files_found: int = 0
    name_collisions: int = 0
    unmatched_adma_examples: list[str] = []
    unmatched_xml_examples: list[str] = []


@app.post("/api/traces/scan", response_model=ScanResponse)
def scan_directory(req: ScanRequest):
    root = Path(req.path).expanduser()
    if not root.is_dir():
        raise HTTPException(status_code=400, detail=f"Not a directory (on the server running this app): {root}")
    result = store.scan_directory(root)

    conn = _catalog_connect()
    try:
        for trace_id, (adma_path, annotation_path) in result.matched.items():
            catalog.register_scanned(conn, trace_id, adma_path, annotation_path)
    finally:
        conn.close()

    return ScanResponse(
        adma_found=result.adma_found,
        xml_found=result.xml_found,
        matched=len(result.matched),
        unmatched_adma_count=result.unmatched_adma_count,
        unmatched_xml_count=result.unmatched_xml_count,
        total_traces=store.count(),
        adma_files_found=result.adma_files_found,
        name_collisions=result.name_collisions,
        unmatched_adma_examples=result.unmatched_adma_examples,
        unmatched_xml_examples=result.unmatched_xml_examples,
    )


@app.get("/api/traces/{trace_id}/scene")
def get_scene(trace_id: str):
    trace = _get_trace_or_404(trace_id)
    return JSONResponse(build_scene_json(trace))


@app.get("/api/traces/{trace_id}/map_overlay")
def get_map_overlay(trace_id: str, provider: str = "osm"):
    """On demand only -- the GUI calls this from an explicit "Show map"
    toggle after a trace is already loaded, never during normal scene
    loading/playback/validation/fixing, and never for a batch of traces.
    Fetches nearby roads from the given online provider and converts them
    into the same local (x, y) frame as the rest of the scene JSON, so the
    frontend can draw them as background context with zero extra
    coordinate handling on its side. A failed/unavailable fetch is
    reported in the response, not raised -- see export.map_enrichment.
    """
    trace = _get_trace_or_404(trace_id)
    enrichment, error = fetch_enrichment(trace, provider, cache_dir=OUTPUT_DIR / "map_cache")
    if enrichment is None:
        return {"ways": [], "provider": provider, "error": error}

    lat0, lon0 = trace.ego.poses[0].lat_deg, trace.ego.poses[0].lon_deg
    ways = [
        {
            "id": way.id,
            "name": way.name,
            "points": [list(latlon_to_local(lat, lon, lat0, lon0)) for lat, lon in way.points],
        }
        for way in enrichment.ways
    ]
    return {"ways": ways, "provider": provider, "error": None}


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


class FixExportOptions(PredictRequest):
    """What to do to a trace and what to write to output/ for it -- shared
    by the single-trace "batch_fix_predict" endpoint (defaults reproduce
    its original always-do-everything behavior) and the Scan Directory
    panel's whole-corpus /api/batch/all "run" mode (GUI checkboxes set
    every field explicitly).
    """

    include_predictions_in_output: bool = True
    fix_issues: bool = True
    predict_trajectories: bool = True
    export_fixed_trace: bool = True
    export_opendrive: bool = True
    export_openscenario: bool = True
    export_adp_yaml: bool = False
    export_report_txt: bool = True
    export_report_xml: bool = False
    also_build_catalog: bool = False
    enrich: str | None = None
    map_key: str | None = None
    author_email: str | None = None


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


def _fix_predict_and_write_output(trace_id: str, opts: FixExportOptions) -> dict:
    """validate -> (optionally) fix -> (optionally) predict -> re-validate
    for one trace, then writes whichever artifacts `opts` asks for into
    output/, mirroring the input corpus layout for ADMA/annotation -- see
    export.batch_output. Shared by the single-trace endpoint below and the
    Scan Directory panel's whole-corpus "run" background job.
    """
    trace = store.get(trace_id)
    before = run_validation(trace)
    fix_summary = apply_fixes(trace) if opts.fix_issues else {}
    added = (
        predict_all(trace, horizon_s=opts.horizon_s, step_s=opts.step_s, backward=opts.backward, forward=opts.forward)
        if opts.predict_trajectories
        else {}
    )
    after = run_validation(trace)

    # One suffix for the whole run, computed after fixing/predicting: every
    # artifact this call writes describes the same state of the trace, so
    # they stay grouped under one name instead of the .xodr silently
    # replacing the previous run's while the .xosc lands beside it.
    suffix = provenance_suffix(trace)

    output_paths: dict[str, str] = {}
    if opts.export_fixed_trace:
        output_paths.update(write_batch_output(
            trace, store.original_annotation_path(trace_id), OUTPUT_DIR,
            include_predictions=opts.include_predictions_in_output, suffix=suffix,
        ))

    if opts.export_opendrive or opts.export_openscenario:
        xodr_path, xosc_path = scenario_output_paths(trace_id, OUTPUT_DIR, suffix)
        if opts.export_opendrive:
            enrichment = None
            if opts.enrich:
                enrichment, _err = fetch_enrichment(trace, opts.enrich, cache_dir=OUTPUT_DIR / "map_cache")
            xodr_path.write_text(generate_opendrive(trace, enrichment=enrichment))
            output_paths["xodr_path"] = str(xodr_path)
        if opts.export_openscenario:
            xosc_path.write_text(generate_openscenario(trace, xodr_path.name))
            output_paths["xosc_path"] = str(xosc_path)

    if opts.export_adp_yaml:
        adp_path = adp_yaml_output_path(trace_id, OUTPUT_DIR, suffix)
        adp_path.write_text(generate_adp_scenario_yaml(trace, map_key=opts.map_key, author_email=opts.author_email))
        output_paths["adp_yaml_path"] = str(adp_path)

    if opts.export_report_txt:
        report_path = report_output_path(trace_id, OUTPUT_DIR, "txt", suffix)
        report_path.write_text(generate_txt_report(trace))
        output_paths["report_txt_path"] = str(report_path)

    if opts.export_report_xml:
        report_path = report_output_path(trace_id, OUTPUT_DIR, "xml", suffix)
        report_path.write_text(generate_xml_report(trace))
        output_paths["report_xml_path"] = str(report_path)

    logged = _log_export(
        "batch_fix_predict", trace_id, suffix, [Path(p) for p in output_paths.values()]
    ) if output_paths else {"provenance": _provenance_label(suffix)}

    return {
        "trace_id": trace_id,
        "before_issue_count": len(before),
        "after_issue_count": len(after),
        "fix_summary": fix_summary,
        "predicted": added,
        "provenance": logged["provenance"],
        "output": output_paths,
    }


@app.post("/api/traces/{trace_id}/batch_fix_predict")
def batch_fix_predict(trace_id: str, req: FixExportOptions = FixExportOptions()):
    """No scene payload in the response -- meant to be called in a loop over
    many trace_ids (see the GUI's multi-select "batch" action) without
    paying for a full scene JSON build on every one. Defaults reproduce the
    original fix+predict+export-everything-but-ADP-and-xml pipeline.
    """
    try:
        return _fix_predict_and_write_output(trace_id, req)
    except KeyError:
        raise HTTPException(status_code=404, detail=f"Unknown trace_id '{trace_id}'")


BATCH_ALL_MODES = ("catalog", "run")

_batch_all_lock = threading.Lock()
_batch_all_state: dict = {
    "running": False,
    "mode": None,
    "total": 0,
    "done": 0,
    "current": None,
    "failed": [],
}


def _catalog_trace(trace_id: str, trace) -> None:
    conn = _catalog_connect()
    try:
        catalog.record_trace(conn, trace, store.original_adma_path(trace_id), store.original_annotation_path(trace_id))
    finally:
        conn.close()


def _process_one_for_batch_all(trace_id: str, mode: str, opts: FixExportOptions) -> None:
    if mode == "catalog":
        trace = store.get(trace_id)
        run_validation(trace)
        _catalog_trace(trace_id, trace)
    else:  # "run"
        _fix_predict_and_write_output(trace_id, opts)
        if opts.also_build_catalog:
            _catalog_trace(trace_id, store.get(trace_id))  # already fixed + re-validated, still cached


def _run_batch_all(trace_ids: list[str], mode: str, opts: FixExportOptions) -> None:
    for trace_id in trace_ids:
        with _batch_all_lock:
            _batch_all_state["current"] = trace_id
        try:
            _process_one_for_batch_all(trace_id, mode, opts)
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
def start_batch_all(mode: str = "run", opts: FixExportOptions = FixExportOptions()):
    """Runs one of two pipelines over *every* registered trace (not just the
    page currently shown in the trace picker) in a background thread, and
    returns immediately -- poll /api/batch/all/status for progress. Meant
    for corpora too large to comfortably multi-select in the GUI.

      - "catalog": validate only, then record location/metadata/phenomena/
        issues into the trace catalog (see catalog.py). No output/ files;
        `opts` is ignored.
      - "run": validate -> whichever of fix/predict `opts` asks for ->
        re-validate -> write whichever export artifacts `opts` asks for
        (see FixExportOptions) -- optionally also cataloging the corrected
        trace in the same pass via `opts.also_build_catalog`, one parse
        instead of two.
    """
    if mode not in BATCH_ALL_MODES:
        raise HTTPException(status_code=400, detail=f"mode must be one of {BATCH_ALL_MODES}")
    with _batch_all_lock:
        if _batch_all_state["running"]:
            raise HTTPException(status_code=409, detail="A batch run is already in progress")
        trace_ids = store.list_ids()
        _batch_all_state.update(running=True, mode=mode, total=len(trace_ids), done=0, current=None, failed=[])
    threading.Thread(target=_run_batch_all, args=(trace_ids, mode, opts), daemon=True).start()
    return {"started": True, "mode": mode, "total": len(trace_ids)}


@app.get("/api/batch/all/status")
def batch_all_status():
    with _batch_all_lock:
        return dict(_batch_all_state)


class CatalogQueryResponse(BaseModel):
    rows: list[dict]
    total: int
    offset: int


@app.get("/api/catalog", response_model=CatalogQueryResponse)
def get_catalog(
    q: str | None = None,
    phenomenon: list[str] | None = Query(default=None),
    issue_category: list[str] | None = Query(default=None),
    limit: int = 200,
    offset: int = 0,
):
    conn = _catalog_connect()
    try:
        rows, total = catalog.query(
            conn, q=q, phenomena=phenomenon, issue_categories=issue_category, limit=limit, offset=offset
        )
    finally:
        conn.close()
    return CatalogQueryResponse(rows=rows, total=total, offset=offset)


@app.get("/api/catalog/tags")
def get_catalog_tags():
    return {"phenomena": catalog.KNOWN_PHENOMENA, "issue_categories": catalog.KNOWN_ISSUE_CATEGORIES}


@app.get("/api/catalog/stats")
def get_catalog_stats():
    conn = _catalog_connect()
    try:
        return catalog.stats(conn)
    finally:
        conn.close()


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


class GenerateVariantsRequest(BaseModel):
    count: int = 5
    mode: str = "preset"  # "preset" | "randomized"
    seed: int | None = None


# trace_id -> {variant_id -> Variant}. Ephemeral, in-memory, deliberately
# separate from the TraceStore's own disk-backed cache (see variants.py):
# a variant has no files on disk to reload from, so it must never be
# treated like a normal cached trace.
_variant_store: dict[str, dict[str, Variant]] = {}


def _variant_summary(v: Variant) -> dict:
    return {
        "variant_id": v.variant_id,
        "kind": v.kind,
        "name": v.name,
        "description": v.description,
        "vehicle_id": v.vehicle_id,
        "source": v.source,
        "issue_count": v.issue_count,
        "base_issue_count": v.base_issue_count,
    }


def _get_variant_or_404(trace_id: str, variant_id: str) -> Variant:
    variant = _variant_store.get(trace_id, {}).get(variant_id)
    if variant is None:
        raise HTTPException(status_code=404, detail=f"Unknown variant '{variant_id}' for trace '{trace_id}'")
    return variant


@app.post("/api/traces/{trace_id}/variants/generate")
def generate_variants(trace_id: str, req: GenerateVariantsRequest = GenerateVariantsRequest()):
    """Generates up to `req.count` alternative ("ODD variant") scenarios by
    perturbing one surrounding vehicle's recorded trajectory at a time --
    see variants.py's module docstring. Replaces any previously-generated
    variants for this trace. "preset" tries five fixed, named, explainable
    perturbations (skipping ones that find no matching vehicle in this
    trace, backfilled with randomized ones so `count` is still honored);
    "randomized" always returns `count` random perturbations (seeded, so
    the same seed reproduces the same set).
    """
    trace = _get_trace_or_404(trace_id)
    if req.mode == "randomized":
        result = generate_randomized_variants(trace, req.count, seed=req.seed)
    elif req.mode == "preset":
        result = generate_preset_variants(trace, req.count)
    else:
        raise HTTPException(status_code=400, detail="mode must be 'preset' or 'randomized'")
    _variant_store[trace_id] = {v.variant_id: v for v in result}
    return {"variants": [_variant_summary(v) for v in result]}


@app.get("/api/traces/{trace_id}/variants")
def list_variants(trace_id: str):
    _get_trace_or_404(trace_id)
    result = _variant_store.get(trace_id, {})
    return {"variants": [_variant_summary(v) for v in result.values()]}


@app.get("/api/traces/{trace_id}/variants/{variant_id}/scene")
def get_variant_scene(trace_id: str, variant_id: str):
    variant = _get_variant_or_404(trace_id, variant_id)
    return JSONResponse(build_scene_json(variant.trace))


@app.get("/api/traces/{trace_id}/variants/{variant_id}/export/{artifact}")
def export_variant(
    trace_id: str, variant_id: str, artifact: str,
    enrich: str | None = None, map_key: str | None = None, author_email: str | None = None,
    include_predictions: bool = True, format: str = "txt",
):
    """Exports one generated variant through exactly the same generators
    the normal per-trace export endpoints use -- a variant is a complete,
    independent Trace object (see variants.py), so nothing export-side
    needs to know it's a variant at all. Writes under a path keyed by the
    variant's own descriptive trace_id (`<trace_id>__<variant_id>`), so it
    never collides with the source trace's own exports.
    """
    variant = _get_variant_or_404(trace_id, variant_id)
    trace = variant.trace
    # A variant's whole identity is already in its composite trace_id, so it
    # needs no `fixed`/`predicted` suffix on top -- see provenance_suffix.
    log = lambda kind, paths, **extra: _log_export(  # noqa: E731
        kind, trace.trace_id, "", paths, source_trace_id=trace_id, variant_id=variant_id,
        variant_name=variant.name, **extra,
    )

    if artifact == "fixed_trace":
        # Not write_batch_output: its annotation filename resolution
        # preserves the *original* scanned file's own name verbatim
        # (resolve_annotation_output_filename), which multiple variants of
        # the same source trace would all collide on. Variants always get
        # a name derived from their own composite trace_id instead.
        adma_path = adma_output_path(trace.trace_id, OUTPUT_DIR)
        write_adma_csv(trace.ego, adma_path)
        annotation_path = OUTPUT_DIR / "annotations" / "Annotations" / f"{trace.trace_id}__refQC_IND.xml"
        annotation_path.parent.mkdir(parents=True, exist_ok=True)
        write_annotation_xml(
            trace, store.original_annotation_path(trace_id), annotation_path, include_predictions=include_predictions
        )
        return log("fixed_trace", [adma_path, annotation_path])

    if artifact == "opendrive":
        enrichment, enrichment_error = None, None
        if enrich:
            enrichment, enrichment_error = fetch_enrichment(trace, enrich, cache_dir=OUTPUT_DIR / "map_cache")
        xodr_path, _xosc_path = scenario_output_paths(trace.trace_id, OUTPUT_DIR)
        xodr_path.write_text(generate_opendrive(trace, enrichment=enrichment))
        return {
            "output_path": _relative_output_path(xodr_path),
            "enrichment": enrichment.provider if enrichment else None,
            "enrichment_requested": enrich,
            "enrichment_error": enrichment_error,
            **log("opendrive", [xodr_path], enrichment=enrichment.provider if enrichment else None),
        }

    if artifact == "openscenario":
        xodr_path, xosc_path = scenario_output_paths(trace.trace_id, OUTPUT_DIR)
        xosc_path.write_text(generate_openscenario(trace, xodr_path.name))
        return {"output_path": _relative_output_path(xosc_path), **log("openscenario", [xosc_path])}

    if artifact == "adp_yaml":
        text = generate_adp_scenario_yaml(trace, map_key=map_key, author_email=author_email)
        out_path = adp_yaml_output_path(trace.trace_id, OUTPUT_DIR)
        out_path.write_text(text)
        return {
            "output_path": _relative_output_path(out_path),
            "map_key": map_key or trace.trace_id,
            "map_key_is_placeholder": map_key is None,
            **log("adp_yaml", [out_path], map_key=map_key or trace.trace_id),
        }

    if artifact == "report":
        if format not in ("txt", "xml"):
            raise HTTPException(status_code=400, detail="format must be 'txt' or 'xml'")
        content = generate_xml_report(trace) if format == "xml" else generate_txt_report(trace)
        out_path = report_output_path(trace.trace_id, OUTPUT_DIR, format)
        out_path.write_text(content)
        return {"output_path": _relative_output_path(out_path), **log(f"report_{format}", [out_path])}

    raise HTTPException(status_code=400, detail=f"unknown artifact '{artifact}'")


def _relative_output_path(path: Path) -> str:
    """Display-friendly path: relative to the repo when possible (the
    normal case), or absolute (e.g. under pytest's tmp_path) otherwise.
    """
    try:
        return str(path.relative_to(REPO_ROOT))
    except ValueError:
        return str(path)


def _provenance_label(suffix: str) -> str:
    """The filename suffix, spelled for humans. "" means the trace is
    exactly as it was recorded, which is worth saying out loud rather than
    leaving as an absence.
    """
    if not suffix:
        return "original"
    return " + ".join(
        f"POV vehicle {part[3:]}" if part.startswith("pov") else part for part in suffix.split("__")
    )


def _log_export(kind: str, trace_id: str, suffix: str, paths: list[Path], **extra) -> dict:
    """Records one export in output/exports.jsonl and returns the fields
    every export endpoint echoes back. Both halves answer the same question
    -- which state of which trace produced these files -- one for the GUI's
    immediate feedback, one for whoever opens output/ a week later.
    """
    files = [_relative_output_path(p) for p in paths]
    provenance = _provenance_label(suffix)
    record_export(OUTPUT_DIR, {"kind": kind, "trace_id": trace_id, "provenance": provenance, "files": files, **extra})
    return {"files": files, "provenance": provenance}


@app.get("/api/traces/{trace_id}/export/adma")
def export_adma(trace_id: str):
    """Writes the corrected ADMA CSV into output/ and reports where. Does
    *not* stream the file back -- every export lands in the project's
    output/ directory rather than the browser's downloads folder; see
    export.batch_output for the layout.
    """
    trace = _get_trace_or_404(trace_id)
    suffix = provenance_suffix(trace)
    out_path = adma_output_path(trace_id, OUTPUT_DIR, suffix)
    write_adma_csv(trace.ego, out_path)
    return {"output_path": _relative_output_path(out_path), **_log_export("adma", trace_id, suffix, [out_path])}


@app.get("/api/traces/{trace_id}/export/annotation")
def export_annotation(trace_id: str, include_predictions: bool = True):
    trace = _get_trace_or_404(trace_id)
    suffix = provenance_suffix(trace)
    original_path = store.original_annotation_path(trace_id)
    out_path = annotation_output_path(trace_id, original_path, OUTPUT_DIR, suffix)
    write_annotation_xml(trace, original_path, out_path, include_predictions=include_predictions)
    return {"output_path": _relative_output_path(out_path), **_log_export("annotation", trace_id, suffix, [out_path])}


@app.get("/api/traces/{trace_id}/export/fixed_trace")
def export_fixed_trace(trace_id: str, include_predictions: bool = True):
    """Writes both the corrected ADMA CSV and annotation XML in one call --
    the GUI's single "Fixed Trace (ADMA+Annotation)" button. Equivalent to
    calling /export/adma and /export/annotation separately; those stay
    available individually for API callers that only want one file.
    """
    trace = _get_trace_or_404(trace_id)
    suffix = provenance_suffix(trace)
    paths = write_batch_output(
        trace, store.original_annotation_path(trace_id), OUTPUT_DIR,
        include_predictions=include_predictions, suffix=suffix,
    )
    return _log_export(
        "fixed_trace", trace_id, suffix, [Path(paths["adma_path"]), Path(paths["annotation_path"])]
    )


@app.get("/api/traces/{trace_id}/export/opendrive")
def export_opendrive_only(trace_id: str, enrich: str | None = None):
    """`enrich` is opt-in and off by default (None): pass e.g. "osm" to look
    up the trace's road on OpenStreetMap first (road name, and a lane-count
    hint used only where the annotation itself gives no trustworthy
    estimate -- see export.road_geometry). A failed/unavailable lookup
    never fails the export; it just falls back to the offline result, same
    as if `enrich` had been omitted -- see export.map_enrichment.
    """
    trace = _get_trace_or_404(trace_id)
    enrichment, enrichment_error = None, None
    if enrich:
        enrichment, enrichment_error = fetch_enrichment(trace, enrich, cache_dir=OUTPUT_DIR / "map_cache")

    suffix = provenance_suffix(trace)
    xodr_path, _xosc_path = scenario_output_paths(trace_id, OUTPUT_DIR, suffix)
    xodr_path.write_text(generate_opendrive(trace, enrichment=enrichment))
    return {
        "output_path": _relative_output_path(xodr_path),
        "enrichment": enrichment.provider if enrichment else None,
        "enrichment_requested": enrich,
        "enrichment_error": enrichment_error,
        **_log_export(
            "opendrive", trace_id, suffix, [xodr_path],
            enrichment=enrichment.provider if enrichment else None,
        ),
    }


@app.get("/api/traces/{trace_id}/export/openscenario")
def export_openscenario_only(trace_id: str, pov_vehicle_id: int | None = None):
    """Writes the .xosc referencing the companion .xodr's canonical filename
    (<trace_id>.xodr) -- doesn't require that file to already exist on disk,
    same as OpenSCENARIO's own reference-by-filename convention.

    `pov_vehicle_id`, when given, re-roots the export from that vehicle's
    point of view (it becomes "Ego", the real ego becomes a regular
    vehicle entity, and the export is truncated to that vehicle's own
    observed window) -- see export.openscenario.generate_openscenario.
    It lands under a `__pov<vehicle_id>` suffix so it never clobbers the
    normal export.
    """
    trace = _get_trace_or_404(trace_id)
    # The road is the same from any point of view, so the .xosc references
    # the non-POV .xodr -- but one carrying this trace's own provenance
    # suffix, so a scenario and the road it names stay a matched pair even
    # when the original and the fixed trace have both been exported.
    road_suffix = provenance_suffix(trace)
    suffix = provenance_suffix(trace, pov_vehicle_id=pov_vehicle_id)
    xodr_path, _ = scenario_output_paths(trace_id, OUTPUT_DIR, road_suffix)
    _, xosc_path = scenario_output_paths(trace_id, OUTPUT_DIR, suffix)
    try:
        text = generate_openscenario(trace, xodr_path.name, pov_vehicle_id=pov_vehicle_id)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc))
    xosc_path.write_text(text)
    return {
        "output_path": _relative_output_path(xosc_path),
        **_log_export("openscenario", trace_id, suffix, [xosc_path], references_xodr=xodr_path.name),
    }


@app.get("/api/traces/{trace_id}/export/adp_yaml")
def export_adp_yaml(
    trace_id: str, map_key: str | None = None, author_email: str | None = None, pov_vehicle_id: int | None = None
):
    """ADP (Applied Intuition Simian) `.scn.yaml` -- an alternative to
    OpenSCENARIO for import into ADP, which doesn't read `.xosc`. See
    export.adp_yaml's module docstring for what's derived from real sample
    files vs. an unavoidable guess (most notably `map.key`, which defaults
    to the trace_id here -- matching the .xodr filename ADP registers a map
    key from on import -- and is echoed back in the response so the GUI can
    surface it).

    `pov_vehicle_id`, when given, re-roots the export the same way as
    /export/openscenario's own POV option, under a `__pov<vehicle_id>`
    suffix.
    """
    trace = _get_trace_or_404(trace_id)
    try:
        text = generate_adp_scenario_yaml(
            trace, map_key=map_key, author_email=author_email, pov_vehicle_id=pov_vehicle_id
        )
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc))
    suffix = provenance_suffix(trace, pov_vehicle_id=pov_vehicle_id)
    out_path = adp_yaml_output_path(trace_id, OUTPUT_DIR, suffix)
    out_path.write_text(text)
    return {
        "output_path": _relative_output_path(out_path),
        "map_key": map_key or trace_id,
        "map_key_is_placeholder": map_key is None,
        **_log_export("adp_yaml", trace_id, suffix, [out_path], map_key=map_key or trace_id),
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
    suffix = provenance_suffix(trace)
    out_path = report_output_path(trace_id, OUTPUT_DIR, format, suffix)
    out_path.write_text(content)
    return {
        "output_path": _relative_output_path(out_path),
        **_log_export(f"report_{format}", trace_id, suffix, [out_path]),
    }


@app.get("/api/exports")
def list_exports(limit: int = 50, trace_id: str | None = None):
    """The tail of output/exports.jsonl, newest first -- what the GUI's
    "Recent exports" list reads. Malformed lines are skipped rather than
    failing the whole request: a truncated last line (a batch job killed
    mid-write) shouldn't hide the history before it.
    """
    manifest = OUTPUT_DIR / MANIFEST_FILENAME
    if not manifest.exists():
        return {"exports": []}
    entries = []
    for line in manifest.read_text().splitlines():
        try:
            entry = json.loads(line)
        except json.JSONDecodeError:
            continue
        if trace_id is None or entry.get("trace_id") == trace_id or entry.get("source_trace_id") == trace_id:
            entries.append(entry)
    return {"exports": entries[::-1][:limit]}


if FRONTEND_DIR.exists():
    app.mount("/", StaticFiles(directory=str(FRONTEND_DIR), html=True), name="frontend")
