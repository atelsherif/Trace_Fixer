"""Generates an ADP (Applied Intuition "Simian") `.scn.yaml` scenario --
an alternative to OpenSCENARIO for import into ADP, which doesn't read
`.xosc` directly.

There is no public schema for this format; ADP's own documentation lives
behind a login this tool has no access to. This module was reverse-engineered
instead from real `.scn.yaml` files the user exported from their own ADP
project -- including two files ADP's *own* log-extraction pipeline produced
from real driving recordings (identifiable by their `metadata.comments`
mentioning an "Originating simulation run" and tags like "ADP Map Cropped
Success"). Those two are the closest analog to what this tool does (turning
a real ADMA + annotation log into a scenario), so the structural choices
below -- arc-length-resampled `pose_b_spline` paths paired with
`ramp_velocity` phases, `hide`/`until_true` wrapping for actors that enter
or leave the recording partway through, a shared `kinematic_bicycle`
`$actor_motion_model` -- mirror that pipeline's own pattern as closely as
the sample files make legible, rather than being invented from scratch.

Two things are unavoidably best-effort guesses, not derived facts, and are
called out at both the code and API level:

  - `map.key`: ADP resolves this against its own map registry, which this
    tool has no access to. Per how this project's own OpenDRIVE export is
    actually used (confirmed with the user): importing a trace's generated
    `.xodr` into ADP registers a map under a key matching the `.xodr`
    filename -- so the default here is the trace_id (matching
    `export/batch_output.py`'s `<trace_id>.xodr` naming), overridable by
    the caller if the file was renamed on import.
  - `model.spectral_model_spec.spectral_model`: the visual asset name for
    each vehicle. ADP's asset catalog isn't accessible from here either;
    `_guess_vehicle_model` below maps annotation object type + dimensions
    onto a small set of generic asset names actually seen across the
    sample files (GENERIC_YARIS, CONFIGURABLE_SEMI_TRUCK,
    GENERIC_DAELIM_MOTORBIKE), falling back to a plain untextured box
    (`model.static`, the same asset-agnostic shape OpenDRIVE static objects
    already use) whenever nothing in that small set is a plausible match.
    A wrong guess only affects the simulation's visuals, never the
    trajectory geometry, and is a one-line hand-edit in the output file.

Coordinates: ADP scenario points are absolute UTM easting/northing. This
tool's internal local frame (`x_m`=East, `y_m`=North meters, anchored at
the trace's first GPS sample -- see geo/transform.py) is an equirectangular
projection, not UTM, but the two agree to within the grid convergence angle
over the short distance of a single trace log (a few meters at most, far
smaller than the fixed-width lane geometry and generic vehicle models
already in play) -- so this module anchors one UTM point at the trace
origin and adds the existing local-frame offsets directly, rather than
re-projecting every single point.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import yaml
from pyproj import Transformer

from trace_fixer.export.openscenario import EGO_HEIGHT_M, EGO_LENGTH_M, EGO_WIDTH_M
from trace_fixer.geo.transform import heading_to_yaw_rad
from trace_fixer.models import StaticObs, Trace, VehicleTrack

# Matches the spacing observed in real ADP log-extraction output (~14-15m
# between consecutive pose_b_spline waypoints).
ARC_STEP_M = 15.0

# Below this total path length, an actor is treated as effectively
# stationary and emitted as a plain fixed-pose obstacle instead of a
# path_following behavior -- avoids a degenerate 1-point b-spline.
MIN_PATH_LENGTH_M = 2.0

# Identical across every sample file regardless of scenario type -- look
# like ADP's own fixed defaults rather than anything trace-specific.
ADAPTIVE_CRUISE_DEFAULTS = {
    "desired_time_gap": "5s",
    "enforce_max_bounds": True,
    "max_accel": 3,
    "max_decel": 3,
    "min_dist": 5,
}
BEHAVIOR_CONFIG_VALUE = {"predicted_control_config": {"point_specification": {"lookahead_duration": "0.500s"}}}
ACTOR_MOTION_MODEL_VALUE = {
    "kinematic_bicycle": {
        "max_acceleration": 50.0,
        "max_deceleration": 50.0,
        "max_steering_angle": 3.0,
        "max_steering_rate": 3.0,
        "max_velocity": 100.0,
        "num_integration_steps": 1,
        "wheelbase": 3.683,
    }
}

DEFAULT_SENSOR_INCLUDE = "scenario://workspace/valeo_sensors/valeo_car.inc.yaml"
DEFAULT_BEHAVIOR_CONFIG_INCLUDE = "scenario://workspace/includes/behavior_config.inc.yaml"
DEFAULT_SCENARIO_VERSION = "v0.96"

_PEDESTRIAN_KEYWORDS = ("pedestrian", "person")
_TRAILER_KEYWORDS = ("trailer",)
_MOTORCYCLE_KEYWORDS = ("motorcycle", "motorbike", "bike", "bicycle", "scooter")
# Reference dims (m) for the generic assets guessed below, from their own
# static bounding boxes in the sample files -- used to scale the asset to
# this vehicle's actual annotated size.
_YARIS_REF = (4.10, 1.80)
_MOTORBIKE_REF = (1.95, 1.80)


@dataclass
class _TimedPoint:
    t_us: int
    x: float
    y: float
    yaw_rad: float
    speed_mps: float


def _utm_zone(lon_deg: float) -> int:
    return int((lon_deg + 180.0) // 6.0) + 1


def _utm_origin(lat0_deg: float, lon0_deg: float) -> tuple[float, float, int, bool]:
    """Returns (utm_x0, utm_y0, zone, north) for the given anchor point."""
    zone = _utm_zone(lon0_deg)
    north = lat0_deg >= 0
    epsg = (32600 if north else 32700) + zone
    transformer = Transformer.from_crs("EPSG:4326", f"EPSG:{epsg}", always_xy=True)
    utm_x0, utm_y0 = transformer.transform(lon0_deg, lat0_deg)
    return utm_x0, utm_y0, zone, north


def _resample_by_arc_length(points: list[_TimedPoint], step_m: float) -> list[_TimedPoint]:
    """Resamples a chronological polyline at fixed arc-length intervals,
    starting one step in from the first point (matching the offset between
    `initial_position` and the first `pose_b_spline` pose observed in real
    extracted files) and always ending exactly at the final point.
    """
    if len(points) < 2:
        return []
    cum = [0.0]
    for i in range(1, len(points)):
        cum.append(cum[-1] + math.hypot(points[i].x - points[i - 1].x, points[i].y - points[i - 1].y))
    total = cum[-1]
    if total < 1e-6:
        return []

    out: list[_TimedPoint] = []
    target = step_m
    i = 1
    while target < total:
        while i < len(points) - 1 and cum[i] < target:
            i += 1
        p0, p1 = points[i - 1], points[i]
        seg = cum[i] - cum[i - 1]
        frac = 0.0 if seg <= 1e-9 else (target - cum[i - 1]) / seg
        dyaw = math.atan2(math.sin(p1.yaw_rad - p0.yaw_rad), math.cos(p1.yaw_rad - p0.yaw_rad))
        out.append(_TimedPoint(
            t_us=round(p0.t_us + frac * (p1.t_us - p0.t_us)),
            x=p0.x + frac * (p1.x - p0.x),
            y=p0.y + frac * (p1.y - p0.y),
            yaw_rad=p0.yaw_rad + frac * dyaw,
            speed_mps=p0.speed_mps + frac * (p1.speed_mps - p0.speed_mps),
        ))
        target += step_m

    last = points[-1]
    if not out or math.hypot(last.x - out[-1].x, last.y - out[-1].y) > 0.5:
        out.append(last)
    return out


def _path_following_behavior(
    start: _TimedPoint, resampled: list[_TimedPoint], utm_x0: float, utm_y0: float, terminal_hold: bool
) -> dict:
    phases = []
    prior_t = start.t_us
    for p in resampled:
        duration_s = max(0.001, (p.t_us - prior_t) / 1e6)
        phases.append({"ramp_velocity": {"duration": round(duration_s, 6), "target": p.speed_mps}})
        prior_t = p.t_us
    if terminal_hold:
        phases.append({"hold_velocity": {"duration": 99999}})

    return {
        "path_following": {
            "adaptive_cruise": dict(ADAPTIVE_CRUISE_DEFAULTS),
            "motion_profile": {"phases": phases},
            "pose_b_spline": {
                "default_tangent_distance": 2.0,
                "poses": [
                    {"heading": p.yaw_rad, "tangent_distance": 2.0, "x": utm_x0 + p.x, "y": utm_y0 + p.y}
                    for p in resampled
                ],
            },
        }
    }


def _hide_until(threshold_s: float) -> dict:
    return {
        "hide": None,
        "until_true": {"compare": {"greater_than": {"static_number": round(threshold_s, 6)}, "x": {"sim_time": {}}}},
    }


def _adp_actor_type(obj_type: str) -> str:
    low = obj_type.lower()
    if any(k in low for k in _PEDESTRIAN_KEYWORDS):
        return "PEDESTRIAN"
    if any(k in low for k in _TRAILER_KEYWORDS):
        return "TRAILER"
    if "unknown" in low:
        return "UNKNOWN"
    return "VEHICLE"


def _box_model(length: float, width: float, height: float) -> dict:
    hl, hw = length / 2, width / 2
    return {
        "static": {
            "height": height,
            "point": [
                {"x": hl, "y": hw}, {"x": hl, "y": -hw}, {"x": -hl, "y": -hw}, {"x": -hl, "y": hw},
            ],
        }
    }


def _guess_vehicle_model(obj_type: str, length: float, width: float) -> dict:
    """Best-effort spectral asset guess -- see the module docstring. Always
    falls back to an asset-agnostic box when nothing plausible matches.
    """
    low = obj_type.lower()
    if any(k in low for k in _PEDESTRIAN_KEYWORDS):
        return {"static": {"height": 1.8, "point": [
            {"x": length / 2, "y": width / 2}, {"x": length / 2, "y": -width / 2},
            {"x": -length / 2, "y": -width / 2}, {"x": -length / 2, "y": width / 2},
        ]}}
    if "truck" in low or "trailer" in low or "semi" in low or length > 9.0:
        return {
            "spectral_model_spec": {
                "scaling": {"y": round(width / 1.8, 4)},
                "spectral_model": "CONFIGURABLE_SEMI_TRUCK",
                "vehicle_spec": {"configurable_trailer_properties": {"truck_length": length}},
            }
        }
    if any(k in low for k in _MOTORCYCLE_KEYWORDS):
        ref_l, ref_w = _MOTORBIKE_REF
        return {"spectral_model_spec": {
            "scaling": {"x": round(length / ref_l, 4), "y": round(width / ref_w, 4)},
            "spectral_model": "GENERIC_DAELIM_MOTORBIKE",
        }}
    # Generic car-like fallback (car/van/bus/unlabeled).
    ref_l, ref_w = _YARIS_REF
    return {"spectral_model_spec": {
        "scaling": {"x": round(length / ref_l, 4), "y": round(width / ref_w, 4)},
        "spectral_model": "GENERIC_YARIS",
    }}


def _vehicle_timeline(track: VehicleTrack) -> list[_TimedPoint]:
    return [
        _TimedPoint(
            t_us=obs.t_us,
            x=obs.x_m,
            y=obs.y_m,
            yaw_rad=math.radians(obs.heading_deg),  # already math-convention -- see geo/populate.py
            speed_mps=0.0,  # filled below via finite differences; annotation carries no per-obs speed
        )
        for obs in track.observations
    ]


def _fill_speeds_from_positions(points: list[_TimedPoint]) -> None:
    if len(points) < 2:
        return
    for i in range(len(points)):
        j0, j1 = max(0, i - 1), min(len(points) - 1, i + 1)
        if j0 == j1:
            continue
        p0, p1 = points[j0], points[j1]
        dt = (p1.t_us - p0.t_us) / 1e6
        points[i].speed_mps = 0.0 if dt <= 0 else math.hypot(p1.x - p0.x, p1.y - p0.y) / dt


def _ego_points(trace: Trace) -> list[_TimedPoint]:
    return [
        _TimedPoint(
            t_us=p.t_us, x=p.x_m, y=p.y_m,
            yaw_rad=heading_to_yaw_rad(p.heading_deg),
            speed_mps=math.hypot(p.vx_mps, p.vy_mps),
        )
        for p in trace.ego.poses
    ]


def _build_ego_agent_from_points(points: list[_TimedPoint], utm_x0: float, utm_y0: float, arc_step_m: float) -> dict:
    resampled = _resample_by_arc_length(points, arc_step_m)
    first = points[0]
    behaviors = [_path_following_behavior(first, resampled, utm_x0, utm_y0, terminal_hold=False)]
    behaviors.append({"smooth_lane_keeping": {"constant_velocity": None, "params": {"distance": 1000}}})
    return {
        "behavior_config": "$behavior_config",
        "behaviors": behaviors,
        "initial_position": {
            "point": {"utm": {"x": utm_x0 + first.x, "y": utm_y0 + first.y, "z": 0}},
            "heading": first.yaw_rad,
        },
        "initial_velocity_mps": first.speed_mps,
    }


def _build_ego_agent(trace: Trace, utm_x0: float, utm_y0: float, arc_step_m: float) -> dict:
    return _build_ego_agent_from_points(_ego_points(trace), utm_x0, utm_y0, arc_step_m)


def _obstacle_from_points(
    obj_id: int, points: list[_TimedPoint], obj_type: str, length: float, width: float, height: float,
    t0_us: int, trace_duration_s: float, utm_x0: float, utm_y0: float, arc_step_m: float,
) -> dict:
    first = points[0]
    adp_type = _adp_actor_type(obj_type)

    resampled = _resample_by_arc_length(points, arc_step_m)
    first_seen_s = max(0.0, (first.t_us - t0_us) / 1e6)

    model = _guess_vehicle_model(obj_type, length, width)
    is_semi_truck = "spectral_model_spec" in model and model["spectral_model_spec"]["spectral_model"] == "CONFIGURABLE_SEMI_TRUCK"
    obstacle: dict = {
        "id": obj_id,
        "initial_state": {
            "heading": first.yaw_rad,
            "point": {"utm": {"x": utm_x0 + first.x, "y": utm_y0 + first.y, "z": 0}},
            "speed_mps": first.speed_mps,
        },
        "model": model,
        "type": "TRAILER" if is_semi_truck and adp_type == "VEHICLE" else adp_type,
    }

    if not resampled:
        # Effectively stationary for the whole recording -- no path needed.
        obstacle["behaviors"] = []
        obstacle["motion_model"] = {"external": {}}
        return obstacle

    behaviors = []
    if first_seen_s > 1e-3:
        behaviors.append(_hide_until(first_seen_s))
    behaviors.append(_path_following_behavior(first, resampled, utm_x0, utm_y0, terminal_hold=True))
    behaviors.append(_hide_until(trace_duration_s))

    obstacle["behavior_config"] = "$behavior_config"
    obstacle["behaviors"] = behaviors
    obstacle["motion_model"] = "$actor_motion_model"
    return obstacle


def _build_vehicle_obstacle(
    obj_id: int, track: VehicleTrack, t0_us: int, trace_duration_s: float, utm_x0: float, utm_y0: float,
    arc_step_m: float,
) -> dict:
    points = _vehicle_timeline(track)
    _fill_speeds_from_positions(points)
    obs0 = track.observations[0]
    return _obstacle_from_points(
        obj_id, points, track.obj_type, obs0.length, obs0.width, obs0.height,
        t0_us, trace_duration_s, utm_x0, utm_y0, arc_step_m,
    )


def _build_ego_as_obstacle(
    obj_id: int, trace: Trace, t0_us: int, t1_us: int, trace_duration_s: float, utm_x0: float, utm_y0: float,
    arc_step_m: float,
) -> dict:
    """The real ego, re-cast as a regular obstacle for a POV export --
    truncated to the POV vehicle's own observed window like everything
    else in that export.
    """
    points = [p for p in _ego_points(trace) if t0_us <= p.t_us <= t1_us]
    return _obstacle_from_points(
        obj_id, points, "Car", EGO_LENGTH_M, EGO_WIDTH_M, EGO_HEIGHT_M,
        t0_us, trace_duration_s, utm_x0, utm_y0, arc_step_m,
    )


def _build_static_obstacle(obj_id: int, obs: StaticObs, utm_x0: float, utm_y0: float) -> dict:
    return {
        "id": obj_id,
        "behaviors": [],
        "initial_state": {
            "heading": 0.0,
            "point": {"utm": {"x": utm_x0 + obs.x_m, "y": utm_y0 + obs.y_m, "z": 0}},
            "speed_mps": 0.0,
        },
        "model": _box_model(obs.length, obs.width, obs.height),
        "motion_model": {"external": {}},
        "type": "UNKNOWN_UNMOVABLE",
    }


def generate_adp_scenario_yaml(
    trace: Trace,
    *,
    map_key: str | None = None,
    author_email: str | None = None,
    sensor_include: str = DEFAULT_SENSOR_INCLUDE,
    behavior_config_include: str = DEFAULT_BEHAVIOR_CONFIG_INCLUDE,
    arc_step_m: float = ARC_STEP_M,
    pov_vehicle_id: int | None = None,
) -> str:
    """By default, "ego" is the recorded ego. `pov_vehicle_id`, when given,
    re-roots the scenario from that vehicle's point of view instead: its
    own recorded path becomes "ego", the real ego becomes a regular
    obstacle, and the whole scenario is truncated to that vehicle's own
    observed time window -- the only span for which its trajectory is
    actually known. `map.key` and the road (via the companion .xodr,
    unaffected by this option) are unchanged either way.
    """
    if not trace.ego.poses:
        raise ValueError("trace has no ego poses")
    if pov_vehicle_id is not None and (
        pov_vehicle_id not in trace.annotation.vehicles or not trace.annotation.vehicles[pov_vehicle_id].observations
    ):
        raise ValueError(f"vehicle {pov_vehicle_id} has no observations to use as a POV")

    lat0, lon0 = trace.ego.poses[0].lat_deg, trace.ego.poses[0].lon_deg
    utm_x0, utm_y0, zone, north = _utm_origin(lat0, lon0)

    if pov_vehicle_id is not None:
        pov_track = trace.annotation.vehicles[pov_vehicle_id]
        pov_points = _vehicle_timeline(pov_track)
        _fill_speeds_from_positions(pov_points)
        t0_us, t1_us = pov_points[0].t_us, pov_points[-1].t_us
        duration_s = max(0.0, (t1_us - t0_us) / 1e6)
        ego_agent = {"ego": _build_ego_agent_from_points(pov_points, utm_x0, utm_y0, arc_step_m)}
        agents = [ego_agent]

        real_ego_points = [p for p in _ego_points(trace) if t0_us <= p.t_us <= t1_us]
        if len(real_ego_points) >= 2:
            agents.append({"obstacle": _obstacle_from_points(
                1, real_ego_points, "Car", EGO_LENGTH_M, EGO_WIDTH_M, EGO_HEIGHT_M,
                t0_us, duration_s, utm_x0, utm_y0, arc_step_m,
            )})

        for track in trace.annotation.vehicles.values():
            if track.obj_id == pov_vehicle_id:
                continue
            windowed_obs = [o for o in track.observations if t0_us <= o.t_us <= t1_us]
            if not windowed_obs:
                continue
            obj_id = len(agents)  # agents[0] is ego; ids continue from there
            points = _vehicle_timeline(track)
            points = [p for p in points if t0_us <= p.t_us <= t1_us]
            _fill_speeds_from_positions(points)
            obs0 = windowed_obs[0]
            agents.append({"obstacle": _obstacle_from_points(
                obj_id, points, track.obj_type, obs0.length, obs0.width, obs0.height,
                t0_us, duration_s, utm_x0, utm_y0, arc_step_m,
            )})
    else:
        t0_us = trace.ego.t0_us
        duration_s = max(0.0, (trace.ego.t1_us - t0_us) / 1e6)
        ego_agent = {"ego": _build_ego_agent(trace, utm_x0, utm_y0, arc_step_m)}
        agents = [ego_agent]

        for obj_id, (_vid, track) in enumerate(sorted(trace.annotation.vehicles.items()), start=1):
            if not track.observations:
                continue
            agents.append({"obstacle": _build_vehicle_obstacle(
                obj_id, track, t0_us, duration_s, utm_x0, utm_y0, arc_step_m
            )})

    map_key_resolved = map_key or trace.trace_id
    comments = []
    if not map_key:
        comments.append({
            "message": (
                f"map.key defaulted to the trace_id ('{map_key_resolved}'), matching the "
                "OpenDRIVE export's <trace_id>.xodr filename -- confirm this matches the map "
                "key ADP registered when that .xodr was imported, or pass map_key explicitly."
            )
        })

    next_id = len(agents) - 1  # agents[0] is ego; ids continue from the last vehicle id used
    for sid, static_obj in sorted(trace.annotation.static_objects.items()):
        if not static_obj.observations:
            continue
        next_id += 1
        agents.append({"obstacle": _build_static_obstacle(next_id, static_obj.observations[0], utm_x0, utm_y0)})

    doc = {
        "metadata": {
            "name": trace.trace_id,
            "scenario_version": DEFAULT_SCENARIO_VERSION,
            "description": "",
            "author_email": author_email or "",
            **({"comments": comments} if comments else {}),
        },
        "sim_end": {"end_if": {"timeout_s": round(duration_s, 3)}},
        "map": {"key": map_key_resolved},
        "projection_settings": {"utm": {"north": north, "zone": zone}},
        "include": [{"file": sensor_include}, {"file": behavior_config_include}],
        "agents": agents,
        "global_variables": [
            {"name": "behavior_config", "value": BEHAVIOR_CONFIG_VALUE},
            {"name": "actor_motion_model", "value": ACTOR_MOTION_MODEL_VALUE},
        ],
        "extra_data": {"use_ego_behavior": True},
    }
    return yaml.safe_dump(doc, sort_keys=False, default_flow_style=False, allow_unicode=True)
