"""Tests for the pov_vehicle_id option on export.openscenario.generate_openscenario
-- re-rooting the exported .xosc from a chosen vehicle's point of view
instead of the recorded ego, truncated to that vehicle's observed window.
"""
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
SAMPLE1_DIR = REPO_ROOT / "data" / "traces" / "sample1"


@pytest.fixture()
def sample1():
    from trace_fixer.scene import load_trace

    return load_trace("sample1", SAMPLE1_DIR / "adma.csv", SAMPLE1_DIR / "annotation.xml")


def test_default_export_is_unaffected_by_the_new_parameter(sample1):
    from trace_fixer.export.openscenario import ENTITY_NAME_EGO, generate_openscenario

    xosc = generate_openscenario(sample1, "sample1.xodr")
    root = ET.fromstring(xosc)
    names = {so.attrib["name"] for so in root.find("Entities").findall("ScenarioObject")}
    assert names == {ENTITY_NAME_EGO, "Vehicle_1", "Vehicle_2", "Vehicle_3", "Vehicle_4", "Vehicle_5"}


def test_pov_export_makes_the_chosen_vehicle_ego_and_the_real_ego_a_vehicle(sample1):
    from trace_fixer.export.openscenario import ENTITY_NAME_EGO, REAL_EGO_AS_VEHICLE_NAME, generate_openscenario

    xosc = generate_openscenario(sample1, "sample1.xodr", pov_vehicle_id=1)
    root = ET.fromstring(xosc)
    names = {so.attrib["name"] for so in root.find("Entities").findall("ScenarioObject")}
    assert ENTITY_NAME_EGO in names
    assert "Vehicle_1" not in names  # the POV vehicle became Ego, not a vehicle entity
    assert REAL_EGO_AS_VEHICLE_NAME in names


def test_pov_export_ego_trajectory_matches_the_chosen_vehicle_s_own_path(sample1):
    from trace_fixer.export.openscenario import generate_openscenario

    track = sample1.annotation.vehicles[1]
    obs = sorted(track.observations, key=lambda o: o.t_us)

    xosc = generate_openscenario(sample1, "sample1.xodr", pov_vehicle_id=1)
    root = ET.fromstring(xosc)
    init_pos = root.find(".//Actions/Private[@entityRef='Ego']/PrivateAction/TeleportAction/Position/WorldPosition")
    assert float(init_pos.attrib["x"]) == pytest.approx(obs[0].x_m, abs=1e-3)
    assert float(init_pos.attrib["y"]) == pytest.approx(obs[0].y_m, abs=1e-3)


def test_pov_export_is_truncated_to_the_vehicle_s_observed_window(sample1):
    from trace_fixer.export.openscenario import generate_openscenario

    track = sample1.annotation.vehicles[1]
    obs = sorted(track.observations, key=lambda o: o.t_us)
    expected_duration_s = (obs[-1].t_us - obs[0].t_us) / 1e6
    full_duration_s = (sample1.ego.t1_us - sample1.ego.t0_us) / 1e6
    assert expected_duration_s < full_duration_s  # sanity: vehicle 1 isn't seen the whole trace

    xosc = generate_openscenario(sample1, "sample1.xodr", pov_vehicle_id=1)
    root = ET.fromstring(xosc)
    stop_cond = root.find(".//StopTrigger/ConditionGroup/Condition/ByValueCondition/SimulationTimeCondition")
    assert float(stop_cond.attrib["value"]) == pytest.approx(expected_duration_s, abs=0.01)


def test_pov_export_drops_vehicles_with_no_overlap_in_the_pov_window(sample1):
    from trace_fixer.export.openscenario import generate_openscenario

    xosc = generate_openscenario(sample1, "sample1.xodr", pov_vehicle_id=1)
    root = ET.fromstring(xosc)
    names = {so.attrib["name"] for so in root.find("Entities").findall("ScenarioObject")}
    # every remaining vehicle entity must genuinely have observations inside vehicle 1's window
    track1 = sample1.annotation.vehicles[1]
    obs1 = sorted(track1.observations, key=lambda o: o.t_us)
    t0_us, t1_us = obs1[0].t_us, obs1[-1].t_us
    for vid, track in sample1.annotation.vehicles.items():
        if vid == 1:
            continue
        overlaps = any(t0_us <= o.t_us <= t1_us for o in track.observations)
        assert (f"Vehicle_{vid}" in names) == overlaps


def test_pov_export_rejects_an_unknown_or_unobserved_vehicle_id(sample1):
    from trace_fixer.export.openscenario import generate_openscenario

    with pytest.raises(ValueError):
        generate_openscenario(sample1, "sample1.xodr", pov_vehicle_id=9999)
