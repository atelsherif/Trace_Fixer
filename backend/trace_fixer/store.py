"""In-memory registry of loaded traces, backed by a data/traces/<id>/ dir
layout (adma.csv + annotation.xml per trace). Small single-user tool scope:
no database, just parse-once-cache-in-memory with mutation in place (fixes/
predictions/sync-offset all mutate the cached Trace object).
"""
from __future__ import annotations

import re
import uuid
from dataclasses import dataclass, field
from pathlib import Path

from trace_fixer.models import Trace
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

    def __post_init__(self) -> None:
        self.traces_dir.mkdir(parents=True, exist_ok=True)

    def list_ids(self) -> list[str]:
        ids = set(self._cache.keys())
        for child in self.traces_dir.iterdir():
            if child.is_dir() and (child / ADMA_FILENAME).exists() and (child / ANNOTATION_FILENAME).exists():
                ids.add(child.name)
        return sorted(ids)

    def _paths(self, trace_id: str) -> tuple[Path, Path]:
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
        while (self.traces_dir / trace_id).exists():
            n += 1
            trace_id = f"{base_id}-{n}"
        d = self.traces_dir / trace_id
        d.mkdir(parents=True)
        (d / ADMA_FILENAME).write_bytes(adma_bytes)
        (d / ANNOTATION_FILENAME).write_bytes(annotation_bytes)
        return trace_id
