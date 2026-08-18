"""Persistent per-trace catalog: identity/location metadata, behavioral
phenomenon tags, and data-quality issue counts for every trace the tool has
scanned or processed -- stored in a small SQLite database (by default
output/catalog.sqlite) so a large corpus can be browsed, filtered, and
sorted from the GUI without re-parsing every trace on every visit.

Three population paths, from lightest to heaviest -- see api.py:
  - register_scanned(): one row per trace_id right after a directory scan
    registers it -- identity + file paths only, no parsing.
  - record_trace(): called once a trace has actually been opened/parsed
    (and, for phenomena, had validation run) -- location (first GPS fix),
    duration, vehicle count, road/weather/light conditions, phenomenon
    tags, and issue counts, all upserted in one pass.

A trace_id is the primary key throughout; re-processing a trace replaces
its metadata/phenomena/issues rows rather than accumulating history, since
the catalog reflects current state, not a log.
"""
from __future__ import annotations

import sqlite3
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

from trace_fixer.analysis import build_trace_summary
from trace_fixer.models import Trace

SCHEMA = """
CREATE TABLE IF NOT EXISTS traces (
    trace_id TEXT PRIMARY KEY,
    adma_path TEXT,
    annotation_path TEXT,
    first_lat REAL,
    first_lon REAL,
    duration_s REAL,
    vehicle_count INTEGER,
    road_type TEXT,
    weather TEXT,
    light_conditions TEXT,
    scanned_at TEXT,
    processed_at TEXT
);
CREATE TABLE IF NOT EXISTS trace_phenomena (
    trace_id TEXT NOT NULL,
    phenomenon TEXT NOT NULL,
    count INTEGER NOT NULL,
    PRIMARY KEY (trace_id, phenomenon)
);
CREATE TABLE IF NOT EXISTS trace_issues (
    trace_id TEXT NOT NULL,
    category TEXT NOT NULL,
    severity TEXT NOT NULL,
    count INTEGER NOT NULL,
    PRIMARY KEY (trace_id, category, severity)
);
"""

# Every phenomenon name the catalog can populate -- used to build the GUI's
# filter checkboxes (see GET /api/catalog/tags) without a distinct-values
# query returning an empty set before anything's been processed yet.
KNOWN_PHENOMENA = [
    "moderate_braking",
    "hard_braking",
    "vehicle_overtakes_ego",
    "ego_overtakes_vehicle",
    "short_headway",
    "near_miss",
    "cut_in",
    "standstill",
    "sharp_turn",
]
KNOWN_ISSUE_CATEGORIES = ["kinematic", "collision", "off_road", "sync"]


def connect(db_path: Path) -> sqlite3.Connection:
    """Opens a connection usable from any thread (safe as long as each
    caller opens its own -- the API layer opens one per request/job step
    rather than sharing one across threads). busy_timeout lets a writer in
    the background batch job and a reader from a GET request overlap
    briefly without an immediate "database is locked" error.
    """
    db_path.parent.mkdir(parents=True, exist_ok=True)
    conn = sqlite3.connect(str(db_path), check_same_thread=False)
    conn.execute("PRAGMA busy_timeout = 5000")
    conn.row_factory = sqlite3.Row
    conn.executescript(SCHEMA)
    return conn


def _now() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def register_scanned(conn: sqlite3.Connection, trace_id: str, adma_path: Path, annotation_path: Path) -> None:
    """Identity-only row, written right after a directory scan matches a
    trace -- cheap enough to do for an entire corpus (tens of thousands of
    traces) since nothing is parsed. Leaves any previously computed
    metadata/phenomena/issues alone if the trace_id was already known.
    """
    with conn:
        conn.execute(
            """
            INSERT INTO traces (trace_id, adma_path, annotation_path, scanned_at)
            VALUES (?, ?, ?, ?)
            ON CONFLICT(trace_id) DO UPDATE SET
                adma_path = excluded.adma_path,
                annotation_path = excluded.annotation_path,
                scanned_at = excluded.scanned_at
            """,
            (trace_id, str(adma_path), str(annotation_path), _now()),
        )


def _mode(values: list[str | None]) -> str | None:
    non_null = [v for v in values if v]
    if not non_null:
        return None
    return Counter(non_null).most_common(1)[0][0]


def record_trace(
    conn: sqlite3.Connection,
    trace: Trace,
    adma_path: Path,
    annotation_path: Path,
    *,
    include_phenomena: bool = True,
    include_issues: bool = True,
) -> None:
    """Upserts everything the catalog knows about one trace: identity,
    location, road/weather/light conditions, duration, vehicle count, and
    (unless disabled) phenomenon tags and issue counts computed from the
    trace's *current* in-memory state -- so calling this after fix+predict
    catalogs the corrected trace, and after only run_validation() catalogs
    the trace as scanned.
    """
    poses = trace.ego.poses
    first_lat = poses[0].lat_deg if poses else None
    first_lon = poses[0].lon_deg if poses else None
    duration_s = (trace.ego.t1_us - trace.ego.t0_us) / 1e6 if len(poses) >= 2 else 0.0
    frame_meta = trace.annotation.frame_meta

    with conn:
        conn.execute(
            """
            INSERT INTO traces (
                trace_id, adma_path, annotation_path, first_lat, first_lon,
                duration_s, vehicle_count, road_type, weather, light_conditions,
                scanned_at, processed_at
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            ON CONFLICT(trace_id) DO UPDATE SET
                adma_path = excluded.adma_path,
                annotation_path = excluded.annotation_path,
                first_lat = excluded.first_lat,
                first_lon = excluded.first_lon,
                duration_s = excluded.duration_s,
                vehicle_count = excluded.vehicle_count,
                road_type = excluded.road_type,
                weather = excluded.weather,
                light_conditions = excluded.light_conditions,
                processed_at = excluded.processed_at
            """,
            (
                trace.trace_id,
                str(adma_path),
                str(annotation_path),
                first_lat,
                first_lon,
                duration_s,
                len(trace.annotation.vehicles),
                _mode([f.road_type for f in frame_meta]),
                _mode([f.weather for f in frame_meta]),
                _mode([f.light_conditions for f in frame_meta]),
                _now(),
                _now(),
            ),
        )

        if include_phenomena:
            summary = build_trace_summary(trace)
            counts: Counter[str] = Counter()
            for ev in summary.braking_events:
                counts[f"{ev.severity}_braking"] += 1
            for ev in summary.overtake_events:
                counts[ev.direction] += 1
            counts["short_headway"] = len(summary.short_headway_events)
            counts["near_miss"] = sum(1 for e in summary.short_headway_events if e.near_miss)
            counts["cut_in"] = len(summary.cut_in_events)
            counts["standstill"] = len(summary.standstill_events)
            counts["sharp_turn"] = len(summary.sharp_turn_events)

            conn.execute("DELETE FROM trace_phenomena WHERE trace_id = ?", (trace.trace_id,))
            conn.executemany(
                "INSERT INTO trace_phenomena (trace_id, phenomenon, count) VALUES (?, ?, ?)",
                [(trace.trace_id, name, n) for name, n in counts.items() if n > 0],
            )

        if include_issues:
            issue_counts: Counter[tuple[str, str]] = Counter()
            for issue in trace.issues:
                issue_counts[(issue.category, issue.severity)] += 1

            conn.execute("DELETE FROM trace_issues WHERE trace_id = ?", (trace.trace_id,))
            conn.executemany(
                "INSERT INTO trace_issues (trace_id, category, severity, count) VALUES (?, ?, ?, ?)",
                [(trace.trace_id, category, severity, n) for (category, severity), n in issue_counts.items()],
            )


def _tags_for(conn: sqlite3.Connection, table: str, key_cols: str, trace_id: str) -> list[dict]:
    rows = conn.execute(f"SELECT {key_cols}, count FROM {table} WHERE trace_id = ?", (trace_id,)).fetchall()
    return [dict(r) for r in rows]


def query(
    conn: sqlite3.Connection,
    q: str | None = None,
    phenomena: list[str] | None = None,
    issue_categories: list[str] | None = None,
    limit: int = 200,
    offset: int = 0,
) -> tuple[list[dict], int]:
    """Returns (rows, total) -- total is the filtered count, for pagination.
    Each row is the trace's metadata plus its phenomenon and issue tags.
    phenomena/issue_categories filters are AND'd: a trace must have every
    requested phenomenon (count > 0) and at least one issue in every
    requested category to match.
    """
    where = []
    params: list = []
    if q:
        where.append("trace_id LIKE ?")
        params.append(f"%{q}%")
    for phenomenon in phenomena or []:
        where.append(
            "trace_id IN (SELECT trace_id FROM trace_phenomena WHERE phenomenon = ? AND count > 0)"
        )
        params.append(phenomenon)
    for category in issue_categories or []:
        where.append("trace_id IN (SELECT trace_id FROM trace_issues WHERE category = ? AND count > 0)")
        params.append(category)
    where_sql = f"WHERE {' AND '.join(where)}" if where else ""

    total = conn.execute(f"SELECT COUNT(*) FROM traces {where_sql}", params).fetchone()[0]
    rows = conn.execute(
        f"SELECT * FROM traces {where_sql} ORDER BY trace_id COLLATE NOCASE LIMIT ? OFFSET ?",
        [*params, limit, offset],
    ).fetchall()

    result = []
    for row in rows:
        d = dict(row)
        d["phenomena"] = _tags_for(conn, "trace_phenomena", "phenomenon", d["trace_id"])
        d["issues"] = _tags_for(conn, "trace_issues", "category, severity", d["trace_id"])
        result.append(d)
    return result, total


def stats(conn: sqlite3.Connection) -> dict:
    total = conn.execute("SELECT COUNT(*) FROM traces").fetchone()[0]
    processed = conn.execute("SELECT COUNT(*) FROM traces WHERE processed_at IS NOT NULL").fetchone()[0]
    return {"total": total, "processed": processed}
