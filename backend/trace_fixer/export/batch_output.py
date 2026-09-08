"""Resolves and writes canonical paths under the project's gitignored
output/ directory, for every artifact this tool can export -- corrected
ADMA + annotation files (mirroring the input corpus layout so the result
can be handed off, or re-scanned as input, the same way the source was),
OpenSCENARIO/OpenDRIVE scenario bundles, and trace summary reports:

    output/adma/ADMA/<trace_id>/adma[__<suffix>].csv
    output/annotations/Annotations/<original annotation stem>[__<suffix>].xml
    output/scenarios/<trace_id>/<trace_id>[__<suffix>].xodr
    output/scenarios/<trace_id>/<trace_id>[__<suffix>].xosc
    output/scenarios/<trace_id>/<trace_id>[__<suffix>].scn.yaml
    output/reports/<trace_id>/<trace_id>_summary[__<suffix>].<txt|xml>
    output/exports.jsonl

`<suffix>` is the export's *provenance* (see provenance_suffix): which
state of the trace it came from -- `fixed`, `predicted`, `fixed__predicted`,
`pov3`, or nothing at all for the trace exactly as recorded. Without it,
exporting a trace, applying fixes, and exporting again silently replaces
the first file with the second, and neither one says which it is.

exports.jsonl is the running log of all of it -- one line per export, with
what was written, from which trace in which state, and when.

Every per-trace export (both the individual GUI download buttons and the
batch fix+predict action) writes here, in addition to whatever it streams
back over HTTP.
"""
from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path

from trace_fixer.export.adma_writer import write_adma_csv
from trace_fixer.export.annotation_writer import write_annotation_xml
from trace_fixer.models import Trace

ADMA_SUBDIR = ("adma", "ADMA")
ANNOTATION_SUBDIR = ("annotations", "Annotations")
SCENARIO_SUBDIR = ("scenarios",)
REPORT_SUBDIR = ("reports",)
_GENERIC_ANNOTATION_NAME = "annotation.xml"  # the store's internal name for uploaded (not scanned) traces


MANIFEST_FILENAME = "exports.jsonl"


def provenance_suffix(trace: Trace, pov_vehicle_id: int | None = None, is_variant: bool = False) -> str:
    """What state of the trace an export represents, as a filename suffix.

    Without this, exporting a trace before and after Apply fixes writes to
    the same path -- the second silently replaces the first, and neither
    file says which it is. A variant's identity already lives in its own
    trace_id, so its per-observation `fixed` flags (an implementation
    detail of how perturbations are applied -- see variants.py) are not
    repeated here.
    """
    parts: list[str] = [] if is_variant else list(trace.provenance())
    if pov_vehicle_id is not None:
        parts.append(f"pov{pov_vehicle_id}")
    return "__".join(parts)


def stamped_name(base: str, suffix: str, extension: str) -> str:
    """`<base>[__<suffix>].<extension>` -- the suffix is what distinguishes
    an export of the fixed trace from an export of the original."""
    return f"{base}__{suffix}{extension}" if suffix else f"{base}{extension}"


def record_export(output_root: Path, entry: dict) -> None:
    """Appends one line to output/exports.jsonl: what was exported, from
    which trace in which state, to which files, when. An append-only log
    rather than a rewritten index, so a long session's history survives
    and concurrent batch writers can't clobber each other's entries.
    """
    output_root.mkdir(parents=True, exist_ok=True)
    entry = {"exported_at": datetime.now(timezone.utc).isoformat(timespec="seconds"), **entry}
    with (output_root / MANIFEST_FILENAME).open("a") as fh:
        fh.write(json.dumps(entry) + "\n")


def resolve_annotation_output_filename(trace_id: str, original_annotation_path: Path) -> str:
    """Preserves the source file's own name (and suffix convention) when
    it's a real one from a directory scan; falls back to a synthesized name
    for uploads, which are stored server-side under a generic filename.
    """
    name = original_annotation_path.name
    if name == _GENERIC_ANNOTATION_NAME:
        return f"{trace_id}__refQC_IND.xml"
    return name


def adma_output_path(trace_id: str, output_root: Path, suffix: str = "") -> Path:
    d = output_root.joinpath(*ADMA_SUBDIR, trace_id)
    d.mkdir(parents=True, exist_ok=True)
    return d / stamped_name("adma", suffix, ".csv")


def annotation_output_path(
    trace_id: str, original_annotation_path: Path, output_root: Path, suffix: str = ""
) -> Path:
    d = output_root.joinpath(*ANNOTATION_SUBDIR)
    d.mkdir(parents=True, exist_ok=True)
    name = resolve_annotation_output_filename(trace_id, original_annotation_path)
    stem, _, ext = name.rpartition(".")
    return d / stamped_name(stem, suffix, f".{ext}")


def scenario_output_paths(trace_id: str, output_root: Path, suffix: str = "") -> tuple[Path, Path]:
    d = output_root.joinpath(*SCENARIO_SUBDIR, trace_id)
    d.mkdir(parents=True, exist_ok=True)
    return d / stamped_name(trace_id, suffix, ".xodr"), d / stamped_name(trace_id, suffix, ".xosc")


def adp_yaml_output_path(trace_id: str, output_root: Path, suffix: str = "") -> Path:
    d = output_root.joinpath(*SCENARIO_SUBDIR, trace_id)
    d.mkdir(parents=True, exist_ok=True)
    return d / stamped_name(trace_id, suffix, ".scn.yaml")


def report_output_path(trace_id: str, output_root: Path, fmt: str, suffix: str = "") -> Path:
    d = output_root.joinpath(*REPORT_SUBDIR, trace_id)
    d.mkdir(parents=True, exist_ok=True)
    return d / stamped_name(f"{trace_id}_summary", suffix, f".{fmt}")


def write_batch_output(
    trace: Trace,
    original_annotation_path: Path,
    output_root: Path,
    include_predictions: bool = True,
    suffix: str | None = None,
) -> dict[str, str]:
    if suffix is None:
        suffix = provenance_suffix(trace)
    adma_path = adma_output_path(trace.trace_id, output_root, suffix)
    write_adma_csv(trace.ego, adma_path)

    annotation_path = annotation_output_path(trace.trace_id, original_annotation_path, output_root, suffix)
    write_annotation_xml(trace, original_annotation_path, annotation_path, include_predictions=include_predictions)

    return {"adma_path": str(adma_path), "annotation_path": str(annotation_path)}
