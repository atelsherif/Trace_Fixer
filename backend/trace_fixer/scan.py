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
  1. Every `adma.csv` file's trace name comes from its nearest *distinctive*
     ancestor directory -- nearest first, skipping generic container names
     like `adma/`, `data/`, `raw/`. Taking the immediate parent
     unconditionally silently collapses an entire corpus laid out as
     `<root>/<trace>/adma/adma.csv` into a single trace called "adma".
  2. Every `*.xml` file is matched to a trace name by stripping known
     annotation-suffix patterns first (fast path, O(1) dict lookup).
  3. Anything left over falls back to a longest-prefix match against the
     known trace names (handles suffix spellings we haven't seen yet).

`ScanResult` reports raw file counts alongside the matched pairs, plus a
sample of what didn't match *and* a sample of what matched but was
discarded as a duplicate (a second `adma.csv` or annotation file resolving
to a trace name already claimed -- e.g. a reprocessed/re-exported copy
living alongside the original elsewhere in the corpus), so a corpus that
scans to a surprisingly low number can be diagnosed from the GUI instead
of guessed at.
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
# Directory names that describe a *kind* of file rather than which trace it
# belongs to, so they can't serve as a trace name.
_GENERIC_DIR_NAMES = {
    "adma", "data", "raw", "input", "inputs", "logs", "log", "csv", "gps", "ins",
    "export", "exports", "output", "outputs", "recording", "recordings", "measurement",
}
_MAX_UNMATCHED_EXAMPLES = 10


@dataclass
class ScanResult:
    matched: dict[str, tuple[Path, Path]] = field(default_factory=dict)
    adma_found: int = 0  # distinct trace names derived from adma.csv files
    xml_found: int = 0
    unmatched_adma_count: int = 0
    unmatched_xml_count: int = 0
    adma_files_found: int = 0  # raw adma.csv file count, before name collapsing
    name_collisions: int = 0  # adma.csv files sharing a derived trace name
    unmatched_adma_examples: list[str] = field(default_factory=list)
    unmatched_xml_examples: list[str] = field(default_factory=list)
    # A second adma.csv/annotation resolving to a trace name a *first* one
    # already claimed -- e.g. a duplicate export of the same trace living
    # under a different subtree -- silently dropped the same way a
    # collision is, but (unlike unmatched_*) it never shows up any other
    # way: the file has a perfectly good match, it's just not the one that
    # won. Each example names both the dropped path and the path that won
    # the slot, so a specific missing pair can be traced to its duplicate.
    adma_collision_examples: list[str] = field(default_factory=list)
    xml_duplicate_count: int = 0
    xml_duplicate_examples: list[str] = field(default_factory=list)


def _strip_known_suffix(stem: str) -> str:
    return _SUFFIX_RE.sub("", stem)


def _trace_name_for_adma(adma_path: Path, root: Path) -> str:
    """The nearest ancestor directory name that identifies a trace rather
    than a file kind. Falls back to the immediate parent when every
    ancestor up to the scan root looks generic.
    """
    root = root.resolve()
    for parent in adma_path.resolve().parents:
        if parent == root or parent == parent.parent:
            break
        if parent.name and parent.name.lower() not in _GENERIC_DIR_NAMES:
            return parent.name
    return adma_path.parent.name


def scan_for_trace_pairs(root: Path) -> ScanResult:
    root = Path(root)
    if not root.is_dir():
        raise NotADirectoryError(str(root))

    adma_by_name: dict[str, Path] = {}
    xml_files: list[tuple[str, Path]] = []
    adma_files_found = 0
    name_collisions = 0
    adma_collision_examples: list[str] = []

    for dirpath, _dirnames, filenames in os.walk(root):
        for fname in filenames:
            lower = fname.lower()
            if lower == ADMA_FILENAME:
                adma_path = Path(dirpath) / fname
                adma_files_found += 1
                trace_name = _trace_name_for_adma(adma_path, root)
                if trace_name in adma_by_name:
                    name_collisions += 1
                    if len(adma_collision_examples) < _MAX_UNMATCHED_EXAMPLES:
                        adma_collision_examples.append(
                            f"{adma_path} (dropped; trace name '{trace_name}' already claimed by {adma_by_name[trace_name]})"
                        )
                else:
                    adma_by_name[trace_name] = adma_path
            elif lower.endswith(".xml"):
                xml_files.append((fname[: -len(".xml")], Path(dirpath) / fname))

    sorted_names = sorted(adma_by_name.keys(), key=str.lower)
    lower_sorted_names = [n.lower() for n in sorted_names]

    matched: dict[str, tuple[Path, Path]] = {}
    unmatched_xml_examples: list[str] = []
    xml_duplicate_count = 0
    xml_duplicate_examples: list[str] = []
    unmatched_xml_count = 0

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

        if trace_name is None:
            unmatched_xml_count += 1
            if len(unmatched_xml_examples) < _MAX_UNMATCHED_EXAMPLES:
                unmatched_xml_examples.append(xml_path.name)
        elif trace_name not in matched:
            matched[trace_name] = (adma_by_name[trace_name], xml_path)
        else:
            # A real match, just not the first one -- e.g. a duplicate/
            # reprocessed annotation export for a trace already claimed.
            # Never surfaced any other way, since it's not "unmatched".
            xml_duplicate_count += 1
            if len(xml_duplicate_examples) < _MAX_UNMATCHED_EXAMPLES:
                xml_duplicate_examples.append(
                    f"{xml_path} (dropped; trace name '{trace_name}' already claimed by {matched[trace_name][1]})"
                )

    unmatched_adma = [name for name in sorted_names if name not in matched]
    return ScanResult(
        matched=matched,
        adma_found=len(adma_by_name),
        xml_found=len(xml_files),
        unmatched_adma_count=len(unmatched_adma),
        unmatched_xml_count=unmatched_xml_count,
        adma_files_found=adma_files_found,
        name_collisions=name_collisions,
        unmatched_adma_examples=unmatched_adma[:_MAX_UNMATCHED_EXAMPLES],
        unmatched_xml_examples=unmatched_xml_examples,
        adma_collision_examples=adma_collision_examples,
        xml_duplicate_count=xml_duplicate_count,
        xml_duplicate_examples=xml_duplicate_examples,
    )
