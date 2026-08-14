"""In-memory registry of loaded traces.

Two ways a trace becomes known to the store:
  - Copied in: data/traces/<id>/adma.csv + annotation.xml (uploads land here).
  - Registered by reference: an (adma_path, annotation_path) pair pointing
    anywhere on disk, added via `register_external` / `scan_directory` --
    used for bulk corpora (tens of thousands of traces) where copying
    everything into data/traces/ would be wasteful. Nothing is read until
    a trace is actually opened (`get`), so scanning a big directory is
    just a filename walk, not a parse-everything operation.

Small single-user tool scope: no database, just parse-once-cache-in-memory
with mutation in place (fixes/predictions/sync-offset all mutate the cached
Trace object).
"""
from __future__ import annotations

import re
import uuid
from dataclasses import dataclass, field
from pathlib import Path

from trace_fixer.models import Trace
from trace_fixer.scan import ScanResult, scan_for_trace_pairs
from trace_fixer.scene import load_trace

ADMA_FILENAME = "adma.csv"
ANNOTATION_FILENAME = "annotation.xml"

_SAFE_ID_RE = re.compile(r"[^a-zA-Z0-9_-]+")


def slugify(name: str) -> str:
    stem = Path(name).stem
    slug = _SAFE_ID_RE.sub("-", stem).strip("-")
    return slug or uuid.uuid4().hex[:8]


@dataclass
class TraceStore:
    traces_dir: Path
    _cache: dict[str, Trace] = field(default_factory=dict)
    _external: dict[str, tuple[Path, Path]] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.traces_dir.mkdir(parents=True, exist_ok=True)

    def _folder_ids(self) -> set[str]:
        ids = set()
        for child in self.traces_dir.iterdir():
            if child.is_dir() and (child / ADMA_FILENAME).exists() and (child / ANNOTATION_FILENAME).exists():
                ids.add(child.name)
        return ids

    def all_ids(self) -> set[str]:
        return self._folder_ids() | set(self._external.keys())

    def count(self) -> int:
        return len(self.all_ids())

    def list_ids(self, query: str | None = None, limit: int | None = None) -> list[str]:
        ids = sorted(self.all_ids(), key=str.lower)
        if query:
            q = query.lower()
            ids = [i for i in ids if q in i.lower()]
        if limit is not None:
            ids = ids[:limit]
        return ids

    def _paths(self, trace_id: str) -> tuple[Path, Path]:
        if trace_id in self._external:
            return self._external[trace_id]
        d = self.traces_dir / trace_id
        return d / ADMA_FILENAME, d / ANNOTATION_FILENAME

    def get(self, trace_id: str) -> Trace:
        if trace_id in self._cache:
            return self._cache[trace_id]
        adma_path, annotation_path = self._paths(trace_id)
        if not adma_path.exists() or not annotation_path.exists():
            raise KeyError(trace_id)
        trace = load_trace(trace_id, adma_path, annotation_path)
        self._cache[trace_id] = trace
        return trace

    def original_annotation_path(self, trace_id: str) -> Path:
        _, annotation_path = self._paths(trace_id)
        return annotation_path

    def reload(self, trace_id: str) -> Trace:
        self._cache.pop(trace_id, None)
        return self.get(trace_id)

    def add_from_bytes(self, name_hint: str, adma_bytes: bytes, annotation_bytes: bytes) -> str:
        trace_id = slugify(name_hint)
        base_id = trace_id
        n = 1
        existing = self.all_ids()
        while trace_id in existing:
            n += 1
            trace_id = f"{base_id}-{n}"
        d = self.traces_dir / trace_id
        d.mkdir(parents=True)
        (d / ADMA_FILENAME).write_bytes(adma_bytes)
        (d / ANNOTATION_FILENAME).write_bytes(annotation_bytes)
        return trace_id

    def register_external(self, trace_id: str, adma_path: Path, annotation_path: Path) -> None:
        self._external.setdefault(trace_id, (adma_path, annotation_path))

    def scan_directory(self, root: Path) -> ScanResult:
        result = scan_for_trace_pairs(root)
        for trace_id, (adma_path, annotation_path) in result.matched.items():
            self.register_external(trace_id, adma_path, annotation_path)
        return result
