#!/usr/bin/env python3
"""Standalone diagnostic for "Scan directory finds far fewer pairs than
this corpus obviously contains".

Run it directly against the real path, on the machine that holds the data:

    python3 tools/diagnose_scan.py /path/to/corpus/root

It reports what the walk actually reached, and -- crucially -- runs both
the *current* matcher and the *original* one (trace name = the adma.csv's
immediate parent directory, plain os.walk, no symlink following) side by
side. If the two disagree, the difference is in this tool. If they agree
and both are low, the difference is in the filesystem (a subtree that
isn't under the scanned root, isn't readable, or isn't mounted), and the
per-subtree and unreadable-directory breakdowns below say which.

Nothing is written or modified; this only reads directory listings.
"""
from __future__ import annotations

import bisect
import os
import re
import sys
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "backend"))

ADMA_FILENAME = "adma.csv"
_SUFFIX_RE = re.compile(r"(__ref-?qc_ind)$", re.IGNORECASE)
_PREFIX_FALLBACK_WINDOW = 5


def original_scan(root: Path) -> tuple[int, int, int, int]:
    """The implementation from before any of this session's changes:
    plain os.walk (no symlink following), trace name = immediate parent.
    """
    adma_by_name: dict[str, Path] = {}
    xml_files: list[tuple[str, Path]] = []
    raw_adma = 0
    for dirpath, _dirnames, filenames in os.walk(root):
        for fname in filenames:
            lower = fname.lower()
            if lower == ADMA_FILENAME:
                raw_adma += 1
                adma_by_name.setdefault(Path(dirpath).name, Path(dirpath) / fname)
            elif lower.endswith(".xml"):
                xml_files.append((fname[: -len(".xml")], Path(dirpath) / fname))

    sorted_names = sorted(adma_by_name, key=str.lower)
    lower_sorted = [n.lower() for n in sorted_names]
    matched: dict[str, tuple[Path, Path]] = {}
    for stem, xml_path in xml_files:
        stripped = _SUFFIX_RE.sub("", stem)
        name = stripped if stripped in adma_by_name else None
        if name is None:
            idx = bisect.bisect_right(lower_sorted, stem.lower())
            for j in range(idx - 1, max(-1, idx - 1 - _PREFIX_FALLBACK_WINDOW), -1):
                if stem.lower().startswith(sorted_names[j].lower()):
                    name = sorted_names[j]
                    break
        if name is not None and name not in matched:
            matched[name] = (adma_by_name[name], xml_path)
    return raw_adma, len(adma_by_name), len(xml_files), len(matched)


def raw_find(root: Path) -> tuple[int, int, int]:
    """Ground truth, independent of either matcher: count adma.csv and
    *.xml by brute force, following symlinks, so the matchers' numbers can
    be checked against what is actually on disk."""
    adma = xml = unreadable = 0
    visited: set[str] = set()
    stack = [root]
    while stack:
        current = stack.pop()
        real = os.path.realpath(current)
        if real in visited:
            continue
        visited.add(real)
        try:
            entries = list(os.scandir(current))
        except OSError:
            unreadable += 1
            continue
        for entry in entries:
            try:
                if entry.is_dir(follow_symlinks=True):
                    stack.append(Path(entry.path))
                elif entry.name.lower() == ADMA_FILENAME:
                    adma += 1
                elif entry.name.lower().endswith(".xml"):
                    xml += 1
            except OSError:
                unreadable += 1
    return adma, xml, unreadable


def main() -> int:
    if len(sys.argv) != 2:
        print(__doc__)
        return 2
    root = Path(sys.argv[1]).expanduser()
    if not root.is_dir():
        print(f"Not a directory: {root}")
        return 2

    print(f"Scan root: {root}")
    print(f"  resolves to: {root.resolve()}")
    print(f"  is a symlink: {root.is_symlink()}")
    print()

    print("--- Top-level entries under the scan root ---")
    # Directories individually (that's where a missing/unreadable subtree
    # shows up); files only as a tally by extension -- a corpus with 27k
    # flat annotation files would otherwise bury everything else.
    try:
        file_kinds: Counter[str] = Counter()
        for entry in sorted(os.scandir(root), key=lambda e: e.name):
            if entry.is_dir(follow_symlinks=True):
                link = f"  -> {os.readlink(entry.path)}" if entry.is_symlink() else ""
                readable = os.access(entry.path, os.R_OK | os.X_OK)
                flag = "" if readable else "   [NOT READABLE]"
                print(f"  dir  {entry.name}{link}{flag}")
            else:
                file_kinds[Path(entry.name).suffix.lower() or "(no extension)"] += 1
        for suffix, count in file_kinds.most_common():
            print(f"  {count} file(s) directly here with suffix {suffix}")
    except OSError as exc:
        print(f"  could not list: {exc}")
    print()

    print("--- Ground truth (brute-force count, follows symlinks) ---")
    adma, xml, unreadable = raw_find(root)
    print(f"  adma.csv files on disk: {adma}")
    print(f"  *.xml files on disk:    {xml}")
    print(f"  unreadable directories: {unreadable}")
    print()

    print("--- ORIGINAL matcher (plain os.walk, name = immediate parent) ---")
    o_raw, o_names, o_xml, o_matched = original_scan(root)
    print(f"  adma.csv seen: {o_raw}   distinct names: {o_names}   xml: {o_xml}   MATCHED PAIRS: {o_matched}")
    print()

    print("--- CURRENT matcher ---")
    from trace_fixer.scan import scan_for_trace_pairs

    r = scan_for_trace_pairs(root)
    print(f"  adma.csv seen: {r.adma_files_found}   distinct names: {r.adma_found}   xml: {r.xml_found}")
    print(f"  MATCHED PAIRS: {len(r.matched)}")
    print(f"  dirs visited: {r.dirs_visited}   unreadable: {r.dirs_unreadable}")
    print(f"  symlinked dirs followed: {r.symlinked_dirs_followed}   cycles skipped: {r.symlink_cycles_skipped}")
    print(f"  name collisions: {r.name_collisions}   duplicate annotations: {r.xml_duplicate_count}")
    print()

    if r.adma_files_by_subtree:
        print("--- adma.csv files per top-level subtree ---")
        for subtree, count in sorted(r.adma_files_by_subtree.items(), key=lambda kv: -kv[1]):
            print(f"  {count:>8}  {subtree}")
        print()

    if r.unreadable_examples:
        print("--- Unreadable directories (these subtrees contributed nothing) ---")
        for example in r.unreadable_examples:
            print(f"  {example}")
        print()

    for label, examples in (
        ("Traces with no matching annotation", r.unmatched_adma_examples),
        ("Annotations matching no trace", r.unmatched_xml_examples),
        ("Dropped duplicate adma.csv", r.adma_collision_examples),
        ("Dropped duplicate annotations", r.xml_duplicate_examples),
    ):
        if examples:
            print(f"--- {label} ---")
            for example in examples[:10]:
                print(f"  {example}")
            print()

    print("--- Verdict ---")
    if adma == 0:
        print("  No adma.csv anywhere under this root. The ADMA files are not in")
        print("  this subtree -- scan a parent directory that contains both, or")
        print("  check the top-level listing above for an unreadable/unmounted branch.")
    elif o_matched == len(r.matched):
        print(f"  Both matchers agree ({o_matched} pairs) -- the difference is NOT in")
        print("  the matching logic. Compare 'adma.csv files on disk' against what")
        print("  you expect; if that is also low, the data isn't under this root")
        print("  (or part of it is unreadable -- see above).")
    elif len(r.matched) < o_matched:
        print(f"  REGRESSION in the current matcher: original found {o_matched},")
        print(f"  current finds {len(r.matched)}. Please send this output.")
    else:
        print(f"  Current matcher finds more ({len(r.matched)}) than the original")
        print(f"  ({o_matched}) -- the newer naming/symlink handling is helping.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
