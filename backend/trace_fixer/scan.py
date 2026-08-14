"""Bulk directory scanning: finds ADMA/annotation pairs across a large
corpus (tens of thousands of files) without copying anything -- traces
found this way are registered in TraceStore by reference (see
store.register_external) and parsed lazily, on first access, just like any
other trace.

Expected layout (matches the customer's export tooling), but only the
*filenames* matter -- the directory nesting is walked recursively, so this
tolerates minor structural variations:

    <root>/.../adma/.../<trace_name>/adma.csv
    <root>/.../annotations/.../<trace_name>__ref-QC_IND.xml   (or __refQC_IND.xml, etc.)

Matching strategy:
  1. Every `adma.csv` file's trace name is its parent directory's name.
  2. Every `*.xml` file is matched to a trace name by stripping known
     annotation-suffix patterns first (fast path, O(1) dict lookup).
  3. Anything left over falls back to a longest-prefix match against the
     known trace names (handles suffix spellings we haven't seen yet).
"""
from __future__ import annotations

import bisect
import os
import re
from dataclasses import dataclass, field
from pathlib import Path

ADMA_FILENAME = "adma.csv"
_SUFFIX_RE = re.compile(r"(__ref-?qc_ind)$", re.IGNORECASE)
_PREFIX_FALLBACK_WINDOW = 5


@dataclass
class ScanResult:
    matched: dict[str, tuple[Path, Path]] = field(default_factory=dict)
    adma_found: int = 0
    xml_found: int = 0
    unmatched_adma_count: int = 0
    unmatched_xml_count: int = 0


def _strip_known_suffix(stem: str) -> str:
    return _SUFFIX_RE.sub("", stem)


def scan_for_trace_pairs(root: Path) -> ScanResult:
    root = Path(root)
    if not root.is_dir():
        raise NotADirectoryError(str(root))

    adma_by_name: dict[str, Path] = {}
    xml_files: list[tuple[str, Path]] = []

    for dirpath, _dirnames, filenames in os.walk(root):
        for fname in filenames:
            lower = fname.lower()
            if lower == ADMA_FILENAME:
                trace_name = Path(dirpath).name
                adma_by_name.setdefault(trace_name, Path(dirpath) / fname)
            elif lower.endswith(".xml"):
                xml_files.append((fname[: -len(".xml")], Path(dirpath) / fname))

    sorted_names = sorted(adma_by_name.keys(), key=str.lower)
    lower_sorted_names = [n.lower() for n in sorted_names]

    matched: dict[str, tuple[Path, Path]] = {}
    unmatched_xml = 0

    for stem, xml_path in xml_files:
        stripped = _strip_known_suffix(stem)
        trace_name = stripped if stripped in adma_by_name else None

        if trace_name is None:
            idx = bisect.bisect_right(lower_sorted_names, stem.lower())
            for j in range(idx - 1, max(-1, idx - 1 - _PREFIX_FALLBACK_WINDOW), -1):
                candidate = sorted_names[j]
                if stem.lower().startswith(candidate.lower()):
                    trace_name = candidate
                    break

        if trace_name is not None and trace_name not in matched:
            matched[trace_name] = (adma_by_name[trace_name], xml_path)
        elif trace_name is None:
            unmatched_xml += 1

    return ScanResult(
        matched=matched,
        adma_found=len(adma_by_name),
        xml_found=len(xml_files),
        unmatched_adma_count=len(adma_by_name) - len(matched),
        unmatched_xml_count=unmatched_xml,
    )
