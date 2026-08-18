"""Tests for trace_fixer.export.map_enrichment: the OSM/HERE-swappable
provider interface, its OpenStreetMap implementation, on-disk caching, and
the never-raises fetch_enrichment entry point that export/opendrive.py's
opt-in "enrich" path relies on. No real network calls -- httpx.post is
monkeypatched, matching the module's own promise that a network problem
never breaks an export.
"""
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
SAMPLE1_DIR = REPO_ROOT / "data" / "traces" / "sample1"


@pytest.fixture()
def sample1():
    from trace_fixer.scene import load_trace

    return load_trace("sample1", SAMPLE1_DIR / "adma.csv", SAMPLE1_DIR / "annotation.xml")


class _FakeResponse:
    def __init__(self, payload):
        self._payload = payload

    def raise_for_status(self):
        pass

    def json(self):
        return self._payload


_OVERPASS_PAYLOAD = {
    "elements": [
        {
            "type": "way",
            "id": 42,
            "tags": {"highway": "motorway", "name": "Test Highway", "lanes": "2", "maxspeed": "65 mph"},
            "geometry": [{"lat": 34.844, "lon": -116.827}, {"lat": 34.845, "lon": -116.815}],
        },
        {"type": "node", "id": 1},  # non-way elements must be ignored
    ]
}


def test_bbox_from_trace_covers_the_whole_path_with_margin(sample1):
    from trace_fixer.export.map_enrichment import BBox

    bbox = BBox.from_trace(sample1, margin_m=50.0)
    lats = [p.lat_deg for p in sample1.ego.poses]
    lons = [p.lon_deg for p in sample1.ego.poses]
    assert bbox.min_lat < min(lats) < max(lats) < bbox.max_lat
    assert bbox.min_lon < min(lons) < max(lons) < bbox.max_lon


def test_osm_provider_parses_ways_and_ignores_non_way_elements(monkeypatch):
    from trace_fixer.export.map_enrichment import BBox, OSMOverpassProvider

    monkeypatch.setattr("httpx.post", lambda *a, **kw: _FakeResponse(_OVERPASS_PAYLOAD))
    provider = OSMOverpassProvider()
    result = provider.fetch(BBox(34.8, -116.9, 34.9, -116.8), timeout=5)

    assert result.provider == "osm"
    assert len(result.ways) == 1
    way = result.ways[0]
    assert way.id == 42
    assert way.name == "Test Highway"
    assert way.lanes == 2
    assert way.maxspeed_kph == pytest.approx(104.6, abs=0.5)
    assert way.points[0] == (34.844, -116.827)


def test_way_lanes_and_maxspeed_handle_missing_or_malformed_tags():
    from trace_fixer.export.map_enrichment import MapWay

    way = MapWay(id=1, points=[], tags={})
    assert way.lanes is None
    assert way.maxspeed_kph is None
    assert way.name is None

    bad = MapWay(id=2, points=[], tags={"lanes": "not-a-number", "maxspeed": "fast"})
    assert bad.lanes is None
    assert bad.maxspeed_kph is None


def test_fetch_enrichment_unknown_provider_returns_none_with_reason(sample1, tmp_path):
    from trace_fixer.export.map_enrichment import fetch_enrichment

    result, error = fetch_enrichment(sample1, "bogus_provider", cache_dir=tmp_path)
    assert result is None
    assert "bogus_provider" in error


def test_fetch_enrichment_never_raises_on_network_failure(sample1, tmp_path, monkeypatch):
    """And the failure reason must be surfaced, not just swallowed -- this
    is what the API/GUI status line shows instead of a bare "unavailable"
    so a real deployment failure (offline, corporate firewall, Overpass
    downtime) can actually be diagnosed."""
    from trace_fixer.export.map_enrichment import fetch_enrichment

    def boom(*a, **kw):
        raise ConnectionError("simulated network failure")

    monkeypatch.setattr("httpx.post", boom)
    result, error = fetch_enrichment(sample1, "osm", cache_dir=tmp_path, timeout=1)
    assert result is None
    assert "simulated network failure" in error


def test_fetch_enrichment_caches_and_skips_network_on_second_call(sample1, tmp_path, monkeypatch):
    from trace_fixer.export.map_enrichment import fetch_enrichment

    calls = []

    def fake_post(*a, **kw):
        calls.append(1)
        return _FakeResponse(_OVERPASS_PAYLOAD)

    monkeypatch.setattr("httpx.post", fake_post)

    first, error1 = fetch_enrichment(sample1, "osm", cache_dir=tmp_path)
    assert first is not None
    assert error1 is None
    assert len(first.ways) == 1
    assert len(calls) == 1

    second, error2 = fetch_enrichment(sample1, "osm", cache_dir=tmp_path)
    assert second is not None
    assert error2 is None
    assert len(second.ways) == 1
    assert len(calls) == 1  # cache hit -- no second network call
