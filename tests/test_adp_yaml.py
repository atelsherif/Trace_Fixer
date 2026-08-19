"""Tests for trace_fixer.export.adp_yaml: the ADP (Applied Intuition Simian)
`.scn.yaml` scenario generator -- an alternative to OpenSCENARIO for import
into ADP, which doesn't read `.xosc`. See the module docstring for what in
this format is derived from real sample files vs. an unavoidable guess.
"""
import math
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
SAMPLE1_DIR = REPO_ROOT / "data" / "traces" / "sample1"
SAMPLE2_DIR = REPO_ROOT / "data" / "traces" / "sample2"


@pytest.fixture()
def sample1():
    from trace_fixer.scene import load_trace

    return load_trace("sample1", SAMPLE1_DIR / "adma.csv", SAMPLE1_DIR / "annotation.xml")


@pytest.fixture()
def sample2():
    from trace_fixer.scene import load_trace

    return load_trace("sample2", SAMPLE2_DIR / "adma.csv", SAMPLE2_DIR / "annotation.xml")


def _parse(trace, **kwargs):
    from trace_fixer.export.adp_yaml import generate_adp_scenario_yaml

    text = generate_adp_scenario_yaml(trace, **kwargs)
    return text, yaml.safe_load(text)


def test_output_is_valid_yaml_with_the_expected_top_level_shape(sample1):
    text, doc = _parse(sample1)
    assert doc["metadata"]["name"] == "sample1"
    assert doc["metadata"]["scenario_version"] == "v0.96"
    assert doc["sim_end"]["end_if"]["timeout_s"] > 0
    assert doc["agents"][0]["ego"] is not None
    assert len(doc["agents"]) > 1


def test_map_key_defaults_to_trace_id_and_flags_it_as_a_guess(sample1):
    """Per the user's own confirmed workflow: importing the companion .xodr
    into ADP registers a map keyed by the .xodr filename, i.e. the trace_id
    -- see export/batch_output.py's scenario_output_paths."""
    _, doc = _parse(sample1)
    assert doc["map"]["key"] == "sample1"
    assert "comments" in doc["metadata"]
    assert "map.key" in doc["metadata"]["comments"][0]["message"]


def test_explicit_map_key_is_used_verbatim_without_a_comment(sample1):
    _, doc = _parse(sample1, map_key="MY_REGISTERED_MAP")
    assert doc["map"]["key"] == "MY_REGISTERED_MAP"
    assert "comments" not in doc["metadata"]


def test_ego_initial_position_matches_the_first_ego_pose_in_utm(sample1):
    from trace_fixer.export.adp_yaml import _utm_origin

    _, doc = _parse(sample1)
    lat0, lon0 = sample1.ego.poses[0].lat_deg, sample1.ego.poses[0].lon_deg
    utm_x0, utm_y0, zone, north = _utm_origin(lat0, lon0)

    ego = doc["agents"][0]["ego"]
    pt = ego["initial_position"]["point"]["utm"]
    assert pt["x"] == pytest.approx(utm_x0, abs=1e-6)
    assert pt["y"] == pytest.approx(utm_y0, abs=1e-6)
    assert doc["projection_settings"]["utm"]["zone"] == zone
    assert doc["projection_settings"]["utm"]["north"] == north


def test_ego_heading_uses_the_same_math_convention_as_the_rest_of_the_tool(sample1):
    from trace_fixer.geo.transform import heading_to_yaw_rad

    _, doc = _parse(sample1)
    ego = doc["agents"][0]["ego"]
    expected = heading_to_yaw_rad(sample1.ego.poses[0].heading_deg)
    assert ego["initial_position"]["heading"] == pytest.approx(expected, abs=1e-9)


def test_ego_pose_b_spline_waypoints_are_roughly_arc_step_apart(sample1):
    from trace_fixer.export.adp_yaml import ARC_STEP_M

    _, doc = _parse(sample1)
    poses = doc["agents"][0]["ego"]["behaviors"][0]["path_following"]["pose_b_spline"]["poses"]
    assert len(poses) >= 2
    # every gap should be close to the target step, except possibly the
    # last (which snaps to the true final point regardless of spacing)
    for a, b in zip(poses[:-2], poses[1:-1]):
        d = math.hypot(b["x"] - a["x"], b["y"] - a["y"])
        assert d == pytest.approx(ARC_STEP_M, rel=0.05)


def test_ego_has_no_bounding_box_or_spectral_model(sample1):
    """Ego uses whatever vehicle is configured by the sensor include file
    -- none of the samples give ego a model block."""
    _, doc = _parse(sample1)
    assert "model" not in doc["agents"][0]["ego"]


def test_vehicle_obstacle_ids_are_dense_and_start_at_one(sample1):
    _, doc = _parse(sample1)
    ids = [a["obstacle"]["id"] for a in doc["agents"][1:]]
    assert ids == list(range(1, len(ids) + 1))


def test_moving_vehicle_gets_hide_until_true_wrapping_and_kinematic_motion_model(sample1):
    _, doc = _parse(sample1)
    moving = [a["obstacle"] for a in doc["agents"][1:] if a["obstacle"]["behaviors"]]
    assert moving, "expected at least one moving vehicle in sample1"
    ob = moving[0]
    assert ob["motion_model"] == "$actor_motion_model"
    assert ob["behavior_config"] == "$behavior_config"
    kinds = [list(b.keys()) for b in ob["behaviors"]]
    assert {"hide", "until_true"} <= set(kinds[0])
    assert "path_following" in kinds[1]
    assert {"hide", "until_true"} <= set(kinds[-1])


def test_first_seen_delay_matches_the_vehicle_s_actual_first_observation(sample1):
    _, doc = _parse(sample1)
    ego_t0 = sample1.ego.t0_us
    ordered_tracks = [t for _id, t in sorted(sample1.annotation.vehicles.items()) if t.observations]
    obj_id, track = 1, ordered_tracks[0]
    expected_delay_s = max(0.0, (track.observations[0].t_us - ego_t0) / 1e6)
    ob = doc["agents"][obj_id]["obstacle"]
    if expected_delay_s > 1e-3:
        assert ob["behaviors"], "expected a hide/until_true wrap for a vehicle first seen after t0"
        threshold = ob["behaviors"][0]["until_true"]["compare"]["greater_than"]["static_number"]
        assert threshold == pytest.approx(expected_delay_s, abs=1e-3)


def test_semi_truck_guess_gets_exact_annotated_length_and_trailer_type(sample1):
    """CONFIGURABLE_SEMI_TRUCK takes an exact truck_length parameter, so a
    long 'Truck'-labeled annotation should map onto it with zero length
    error, unlike the other generic (fixed-mesh, scaled) assets."""
    _, doc = _parse(sample1)
    trucks = [
        a["obstacle"] for a in doc["agents"][1:]
        if "spectral_model_spec" in a["obstacle"].get("model", {})
        and a["obstacle"]["model"]["spectral_model_spec"]["spectral_model"] == "CONFIGURABLE_SEMI_TRUCK"
    ]
    assert trucks
    ob = trucks[0]
    assert ob["type"] == "TRAILER"
    truck_length = ob["model"]["spectral_model_spec"]["vehicle_spec"]["configurable_trailer_properties"]["truck_length"]
    assert truck_length > 9.0


def test_static_objects_are_box_models_with_no_behaviors(sample1):
    _, doc = _parse(sample1)
    statics = [a["obstacle"] for a in doc["agents"][1:] if a["obstacle"]["type"] == "UNKNOWN_UNMOVABLE"]
    assert len(statics) == len(sample1.annotation.static_objects)
    for ob in statics:
        assert ob["behaviors"] == []
        assert "static" in ob["model"]
        assert ob["motion_model"] == {"external": {}}


def test_pedestrian_keyword_maps_to_pedestrian_type_and_box_model():
    """No annotation.xml sample actually contains a pedestrian, so this
    exercises the mapping function directly rather than end-to-end."""
    from trace_fixer.export.adp_yaml import _adp_actor_type, _guess_vehicle_model

    assert _adp_actor_type("Pedestrian") == "PEDESTRIAN"
    model = _guess_vehicle_model("Pedestrian", 0.5, 0.5)
    assert "static" in model  # never guess a spectral asset for a pedestrian


def test_generic_car_falls_back_to_scaled_yaris(sample1):
    from trace_fixer.export.adp_yaml import _guess_vehicle_model

    model = _guess_vehicle_model("Car", 4.5, 1.9)
    assert model["spectral_model_spec"]["spectral_model"] == "GENERIC_YARIS"
    assert model["spectral_model_spec"]["scaling"]["x"] > 1.0  # slightly longer than the reference Yaris


def test_resample_by_arc_length_handles_a_stationary_track():
    from trace_fixer.export.adp_yaml import _TimedPoint, _resample_by_arc_length

    pts = [_TimedPoint(t_us=i * 100_000, x=1.0, y=2.0, yaw_rad=0.0, speed_mps=0.0) for i in range(5)]
    assert _resample_by_arc_length(pts, 15.0) == []


def test_generated_yaml_round_trips_through_pyyaml_safe_load(sample2):
    """sample2 exercises a second, independently-shaped real trace end to
    end (not just structural assertions on sample1)."""
    text, doc = _parse(sample2)
    assert isinstance(doc, dict)
    reparsed = yaml.safe_load(text)
    assert reparsed == doc


def test_map_key_placeholder_note_not_present_when_explicit(sample2):
    _, doc = _parse(sample2, map_key="explicit_key")
    assert "map.key" not in yaml.safe_dump(doc.get("metadata", {}))
