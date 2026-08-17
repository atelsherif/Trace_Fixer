"""Writes corrected ADMA + annotation files for a trace into the project's
gitignored output/ directory, mirroring the input corpus layout so the
result can be handed off (or re-scanned) the same way the source was:

    output/adma/ADMA/<trace_id>/adma.csv
    output/annotations/Annotations/<original annotation filename>

Used by the batch fix+predict action so processing many traces produces a
ready-to-use output corpus without a manual per-trace download click.
"""
from __future__ import annotations

from pathlib import Path

from trace_fixer.export.adma_writer import write_adma_csv
from trace_fixer.export.annotation_writer import write_annotation_xml
from trace_fixer.models import Trace

ADMA_SUBDIR = ("adma", "ADMA")
ANNOTATION_SUBDIR = ("annotations", "Annotations")
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


def write_batch_output(
    trace: Trace,
    original_annotation_path: Path,
    output_root: Path,
    include_predictions: bool = True,
) -> dict[str, str]:
    adma_dir = output_root.joinpath(*ADMA_SUBDIR, trace.trace_id)
    adma_dir.mkdir(parents=True, exist_ok=True)
    adma_path = adma_dir / "adma.csv"
    write_adma_csv(trace.ego, adma_path)

    annotation_dir = output_root.joinpath(*ANNOTATION_SUBDIR)
    annotation_dir.mkdir(parents=True, exist_ok=True)
    annotation_path = annotation_dir / resolve_annotation_output_filename(trace.trace_id, original_annotation_path)
    write_annotation_xml(trace, original_annotation_path, annotation_path, include_predictions=include_predictions)

    return {"adma_path": str(adma_path), "annotation_path": str(annotation_path)}
