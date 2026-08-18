"""Optional online map enrichment for OpenDRIVE export -- road name and an
extra lane-count signal from a real map, on top of (never instead of) the
annotation-derived estimate in road_geometry.py.

This is deliberately isolated behind a small provider interface
(MapEnrichmentProvider) so today's OpenStreetMap implementation can be
swapped for a HERE-backed one later without touching anything else: the
caller only ever sees a normalized MapEnrichmentResult, never a provider's
own response shape. Swapping providers is: write a class implementing
`fetch`, register it in `PROVIDERS`, done -- road_geometry.py and the API
layer are unaffected.

Everything here is opt-in and export-only. Nothing in this module is
imported by scene.py or called during normal GUI use (loading a trace,
playback, validation, fixing) -- see api.py's export endpoint, which only
calls fetch_enrichment when a caller explicitly asks for it. That's a
deliberate architectural guarantee, not an informal one: the interactive
path simply has no code path that reaches this module.

fetch_enrichment() never raises -- any failure (network unreachable, DNS,
timeout, malformed response, rate limit) is caught and logged, and callers
get None back, meaning "proceed exactly as if enrichment was never
requested." A slow/unavailable network must never break an export.
"""
from __future__ import annotations

import hashlib
import json
import logging
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Protocol

import httpx

from trace_fixer.models import Trace

logger = logging.getLogger(__name__)

# Margin around the trace's own GPS extent when querying for nearby roads --
# generous enough to catch the road itself even with GPS noise/drift, small
# enough to keep the query (and the number of unrelated ways returned) cheap.
BBOX_MARGIN_M = 100.0
DEFAULT_TIMEOUT_S = 10.0
OVERPASS_URL = "https://overpass-api.de/api/interpreter"


@dataclass(frozen=True)
class BBox:
    min_lat: float
    min_lon: float
    max_lat: float
    max_lon: float

    @classmethod
    def from_trace(cls, trace: Trace, margin_m: float = BBOX_MARGIN_M) -> "BBox":
        lats = [p.lat_deg for p in trace.ego.poses]
        lons = [p.lon_deg for p in trace.ego.poses]
        lat0 = lats[0]
        # Cheap, locally-fine degrees-per-meter conversion (same equirectangular
        # assumption used everywhere else in this codebase for single-trace scale).
        dlat = margin_m / 111_320.0
        dlon = margin_m / (111_320.0 * max(0.1, math.cos(math.radians(lat0))))
        return cls(min(lats) - dlat, min(lons) - dlon, max(lats) + dlat, max(lons) + dlon)

    def cache_key(self) -> str:
        rounded = (round(self.min_lat, 4), round(self.min_lon, 4), round(self.max_lat, 4), round(self.max_lon, 4))
        return hashlib.sha1(str(rounded).encode()).hexdigest()[:16]


@dataclass
class MapWay:
    id: int
    points: list[tuple[float, float]]  # (lat, lon), in way order
    tags: dict[str, str] = field(default_factory=dict)

    @property
    def name(self) -> str | None:
        return self.tags.get("name") or self.tags.get("ref")

    @property
    def lanes(self) -> int | None:
        raw = self.tags.get("lanes")
        try:
            return int(raw) if raw else None
        except ValueError:
            return None

    @property
    def maxspeed_kph(self) -> float | None:
        raw = self.tags.get("maxspeed")
        if not raw:
            return None
        try:
            if "mph" in raw:
                return float(raw.split()[0]) * 1.60934
            return float(raw.split()[0])
        except (ValueError, IndexError):
            return None


@dataclass
class MapEnrichmentResult:
    provider: str
    bbox: BBox
    ways: list[MapWay]


class MapEnrichmentProvider(Protocol):
    name: str

    def fetch(self, bbox: BBox, timeout: float) -> MapEnrichmentResult: ...


class OSMOverpassProvider:
    """OpenStreetMap via the public Overpass API -- free, no API key. Gives
    road centerline geometry and tags (name, lane count, speed limit) but
    *not* lane-level boundary geometry; see road_geometry.py's docstring
    and the README's "Online map enrichment" section for what this is (and
    isn't) used for.
    """

    name = "osm"

    def fetch(self, bbox: BBox, timeout: float = DEFAULT_TIMEOUT_S) -> MapEnrichmentResult:
        query = (
            f"[out:json][timeout:{int(timeout)}];"
            f'way["highway"]({bbox.min_lat},{bbox.min_lon},{bbox.max_lat},{bbox.max_lon});'
            "out geom;"
        )
        response = httpx.post(OVERPASS_URL, data={"data": query}, timeout=timeout)
        response.raise_for_status()
        data = response.json()

        ways = []
        for el in data.get("elements", []):
            if el.get("type") != "way" or "geometry" not in el:
                continue
            points = [(pt["lat"], pt["lon"]) for pt in el["geometry"]]
            ways.append(MapWay(id=el["id"], points=points, tags=el.get("tags", {})))
        return MapEnrichmentResult(provider=self.name, bbox=bbox, ways=ways)


PROVIDERS: dict[str, MapEnrichmentProvider] = {"osm": OSMOverpassProvider()}


def _cache_path(cache_dir: Path, provider: str, bbox: BBox) -> Path:
    return cache_dir / f"{provider}_{bbox.cache_key()}.json"


def _load_cached(path: Path) -> MapEnrichmentResult | None:
    if not path.exists():
        return None
    try:
        raw = json.loads(path.read_text())
        bbox = BBox(**raw["bbox"])
        ways = [MapWay(id=w["id"], points=[tuple(p) for p in w["points"]], tags=w["tags"]) for w in raw["ways"]]
        return MapEnrichmentResult(provider=raw["provider"], bbox=bbox, ways=ways)
    except Exception:  # noqa: BLE001 -- a corrupt cache entry should never break an export
        logger.warning("Ignoring unreadable map enrichment cache entry: %s", path, exc_info=True)
        return None


def _save_cache(path: Path, result: MapEnrichmentResult) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "provider": result.provider,
        "bbox": {"min_lat": result.bbox.min_lat, "min_lon": result.bbox.min_lon, "max_lat": result.bbox.max_lat, "max_lon": result.bbox.max_lon},
        "ways": [{"id": w.id, "points": w.points, "tags": w.tags} for w in result.ways],
    }
    path.write_text(json.dumps(payload))


def fetch_enrichment(
    trace: Trace,
    provider_name: str,
    cache_dir: Path,
    timeout: float = DEFAULT_TIMEOUT_S,
) -> MapEnrichmentResult | None:
    """The one function callers should use. Never raises: any problem
    (unknown provider, network failure, bad response) is logged and
    swallowed, returning None so the caller proceeds without enrichment.
    Caches successful fetches to disk keyed by (provider, rounded bbox) so
    repeated exports of the same trace/corpus don't re-hit the network.
    """
    provider = PROVIDERS.get(provider_name)
    if provider is None:
        logger.warning("Unknown map enrichment provider %r; proceeding without enrichment", provider_name)
        return None

    bbox = BBox.from_trace(trace)
    cache_file = _cache_path(cache_dir, provider.name, bbox)
    cached = _load_cached(cache_file)
    if cached is not None:
        return cached

    try:
        result = provider.fetch(bbox, timeout=timeout)
    except Exception:  # noqa: BLE001 -- network/parsing failures must never break an export
        logger.warning("Map enrichment fetch failed (provider=%s); proceeding without it", provider_name, exc_info=True)
        return None

    _save_cache(cache_file, result)
    return result
