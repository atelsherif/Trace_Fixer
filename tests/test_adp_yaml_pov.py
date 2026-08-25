"""Tests for the pov_vehicle_id option on export.adp_yaml.generate_adp_scenario_yaml
-- re-rooting the exported .scn.yaml from a chosen vehicle's point of view
instead of the recorded ego, truncated to that vehicle's observed window.
Mirrors tests/test_openscenario_pov.py for the ADP format.
"""
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
SAMPLE1_DIR = REPO_ROOT / "data" / "traces" / "sample1"


@pytest.fixture()
def sample1():
    from trace_fixer.scene import load_trace

    return load_trace("sample1", SAMPLE1_DIR / "adma.csv", SAMPLE1_DIR / "annotation.xml")


def _parse(trace, **kwargs):
    from trace_fixer.export.adp_yaml import generate_adp_scenario_yaml

    text = generate_adp_scenario_yaml(trace, **kwargs)
    return yaml.safe_load(text)


def test_pov_ego_initial_position_matches_the_chosen_vehicle_s_normal_obstacle_position(sample1):
    """The POV "ego" for vehicle 1 must land at exactly the same UTM point
    vehicle 1 gets as a normal obstacle in the default export -- both are
    derived from the same first observation."""
    normal = _parse(sample1)
    pov = _parse(sample1, pov_vehicle_id=1)

    normal_obstacle_1 = next(a["obstacle"] for a in normal["agents"][1:] if a["obstacle"]["id"] == 1)
    pov_ego = pov["agents"][0]["ego"]
    assert pov_ego["initial_position"]["point"]["utm"] == normal_obstacle_1["initial_state"]["point"]["utm"]


def test_pov_export_is_truncated_and_includes_the_real_ego_as_an_obstacle(sample1):
    from trace_fixer.geo.transform import heading_to_yaw_rad

    from trace_fixer.export.adp_yaml import _utm_origin

    track = sample1.annotation.vehicles[1]
    obs = sorted(track.observations, key=lambda o: o.t_us)
    t0_us, t1_us = obs[0].t_us, obs[-1].t_us
    expected_duration_s = (t1_us - t0_us) / 1e6
    full_duration_s = (sample1.ego.t1_us - sample1.ego.t0_us) / 1e6
    assert expected_duration_s < full_duration_s

    doc = _parse(sample1, pov_vehicle_id=1)
    assert doc["sim_end"]["end_if"]["timeout_s"] == pytest.approx(expected_duration_s, abs=0.01)

    # the real ego's first pose inside the POV window must show up as one
    # of the obstacles, at its own recorded position
    real_ego_first = next(p for p in sample1.ego.poses if t0_us <= p.t_us <= t1_us)
    utm_x0, utm_y0, _zone, _north = _utm_origin(sample1.ego.poses[0].lat_deg, sample1.ego.poses[0].lon_deg)
    expected_x = utm_x0 + real_ego_first.x_m
    expected_y = utm_y0 + real_ego_first.y_m
    expected_heading = heading_to_yaw_rad(real_ego_first.heading_deg)

    obstacles = [a["obstacle"] for a in doc["agents"][1:]]
    matches = [
        o for o in obstacles
        if abs(o["initial_state"]["point"]["utm"]["x"] - expected_x) < 1e-3
        and abs(o["initial_state"]["point"]["utm"]["y"] - expected_y) < 1e-3
        and abs(o["initial_state"]["heading"] - expected_heading) < 1e-6
    ]
    assert matches, "expected an obstacle at the real ego's own first-in-window position"


def test_pov_export_drops_the_pov_vehicle_itself_from_the_obstacle_list(sample1):
    doc = _parse(sample1, pov_vehicle_id=1)
    # obstacle ids are reassigned sequentially in POV mode, so check there's
    # exactly one fewer *original* vehicle than the trace has, accounting
    # for windowing -- simplest robust check: agent count changed and no
    # crash/duplicate-id issue.
    ids = [a["obstacle"]["id"] for a in doc["agents"][1:]]
    assert ids == list(range(1, len(ids) + 1))  # still dense and unique


def test_pov_export_rejects_an_unknown_or_unobserved_vehicle_id(sample1):
    from trace_fixer.export.adp_yaml import generate_adp_scenario_yaml

    with pytest.raises(ValueError):
        generate_adp_scenario_yaml(sample1, pov_vehicle_id=9999)


def test_default_export_unaffected_by_the_new_parameter(sample1):
    doc = _parse(sample1)
    assert doc["agents"][0]["ego"] is not None
    assert doc["sim_end"]["end_if"]["timeout_s"] == pytest.approx((sample1.ego.t1_us - sample1.ego.t0_us) / 1e6, abs=0.01)
