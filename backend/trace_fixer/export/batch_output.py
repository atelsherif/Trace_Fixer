"""Resolves and writes canonical paths under the project's gitignored
output/ directory, for every artifact this tool can export -- corrected
ADMA + annotation files (mirroring the input corpus layout so the result
can be handed off, or re-scanned as input, the same way the source was),
OpenSCENARIO/OpenDRIVE scenario bundles, and trace summary reports:

    output/adma/ADMA/<trace_id>/adma.csv
    output/annotations/Annotations/<original annotation filename>
    output/scenarios/<trace_id>/<trace_id>.xodr
    output/scenarios/<trace_id>/<trace_id>.xosc
    output/reports/<trace_id>/<trace_id>_summary.<txt|xml>

Every per-trace export (both the individual GUI download buttons and the
batch fix+predict action) writes here, in addition to whatever it streams
back over HTTP.
"""
from __future__ import annotations

from pathlib import Path

from trace_fixer.export.adma_writer import write_adma_csv
from trace_fixer.export.annotation_writer import write_annotation_xml
from trace_fixer.models import Trace

ADMA_SUBDIR = ("adma", "ADMA")
ANNOTATION_SUBDIR = ("annotations", "Annotations")
SCENARIO_SUBDIR = ("scenarios",)
REPORT_SUBDIR = ("reports",)
_GENERIC_ANNOTATION_NAME = "annotation.xml"  # the store's internal name for uploaded (not scanned) traces


def resolve_annotation_output_filename(trace_id: str, original_annotation_path: Path) -> str:
    """Preserves the source file's own name (and suffix convention) when
    it's a real one from a directory scan; falls back to a synthesized name
    for uploads, which are stored server-side under a generic filename.
    """
    name = original_annotation_path.name
    if name == _GENERIC_ANNOTATION_NAME:
        return f"{trace_id}__refQC_IND.xml"
    return name


def adma_output_path(trace_id: str, output_root: Path) -> Path:
    d = output_root.joinpath(*ADMA_SUBDIR, trace_id)
    d.mkdir(parents=True, exist_ok=True)
    return d / "adma.csv"


def annotation_output_path(trace_id: str, original_annotation_path: Path, output_root: Path) -> Path:
    d = output_root.joinpath(*ANNOTATION_SUBDIR)
    d.mkdir(parents=True, exist_ok=True)
    return d / resolve_annotation_output_filename(trace_id, original_annotation_path)


def scenario_output_paths(trace_id: str, output_root: Path) -> tuple[Path, Path]:
    d = output_root.joinpath(*SCENARIO_SUBDIR, trace_id)
    d.mkdir(parents=True, exist_ok=True)
    return d / f"{trace_id}.xodr", d / f"{trace_id}.xosc"


def report_output_path(trace_id: str, output_root: Path, fmt: str) -> Path:
    d = output_root.joinpath(*REPORT_SUBDIR, trace_id)
    d.mkdir(parents=True, exist_ok=True)
    return d / f"{trace_id}_summary.{fmt}"


def write_batch_output(
    trace: Trace,
    original_annotation_path: Path,
    output_root: Path,
    include_predictions: bool = True,
) -> dict[str, str]:
    adma_path = adma_output_path(trace.trace_id, output_root)
    write_adma_csv(trace.ego, adma_path)

    annotation_path = annotation_output_path(trace.trace_id, original_annotation_path, output_root)
    write_annotation_xml(trace, original_annotation_path, annotation_path, include_predictions=include_predictions)

    return {"adma_path": str(adma_path), "annotation_path": str(annotation_path)}
