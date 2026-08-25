"""Minimal OpenSCENARIO 1.x (.xosc) generator: replays the (fixed/predicted)
ego + vehicle tracks as recorded-trajectory FollowTrajectoryAction entities,
referencing the companion .xodr road network. This is the standard pattern
simulators such as esmini use for "log replay" scenarios.
"""
from __future__ import annotations

import math
from datetime import datetime, timezone
from xml.dom import minidom
from xml.etree.ElementTree import Element, SubElement, tostring

from trace_fixer.models import Trace

EGO_LENGTH_M = 4.9
EGO_WIDTH_M = 1.9
EGO_HEIGHT_M = 1.5
ENTITY_NAME_EGO = "Ego"

_CATEGORY_MAP = {"car": "car", "truck": "truck", "van": "van", "bus": "bus", "motorbike": "motorbike"}


def _vehicle_category(obj_type: str) -> str:
    return _CATEGORY_MAP.get(obj_type.strip().lower(), "car")


def _downsample_time(items: list, stride: int) -> list:
    if stride <= 1 or len(items) <= 2:
        return items
    out = items[::stride]
    if out[-1] is not items[-1]:
        out.append(items[-1])
    return out


def _bounding_box(el: Element, length: float, width: float, height: float) -> None:
    bb = SubElement(el, "BoundingBox")
    SubElement(bb, "Center", {"x": "0", "y": "0", "z": f"{height / 2:.3f}"})
    SubElement(bb, "Dimensions", {"length": f"{length:.3f}", "width": f"{width:.3f}", "height": f"{height:.3f}"})


def _performance(el: Element) -> None:
    SubElement(el, "Performance", {"maxSpeed": "70", "maxAcceleration": "10", "maxDeceleration": "10"})


def _axles(el: Element, length: float) -> None:
    axles = SubElement(el, "Axles")
    SubElement(axles, "FrontAxle", {
        "maxSteering": "0.5", "wheelDiameter": "0.6", "trackWidth": "1.8",
        "positionX": f"{length * 0.3:.3f}", "positionZ": "0.3",
    })
    SubElement(axles, "RearAxle", {
        "maxSteering": "0.0", "wheelDiameter": "0.6", "trackWidth": "1.8",
        "positionX": f"{-length * 0.3:.3f}", "positionZ": "0.3",
    })


def _vehicle_scenario_object(entities: Element, name: str, category: str, length: float, width: float, height: float) -> None:
    so = SubElement(entities, "ScenarioObject", {"name": name})
    vehicle = SubElement(so, "Vehicle", {"name": f"{name}_vehicle", "vehicleCategory": category})
    SubElement(vehicle, "ParameterDeclarations")
    _bounding_box(vehicle, length, width, height)
    _performance(vehicle)
    _axles(vehicle, length)


def _world_position(parent: Element, x: float, y: float, heading_deg: float) -> Element:
    pos = SubElement(parent, "Position")
    SubElement(pos, "WorldPosition", {
        "x": f"{x:.3f}", "y": f"{y:.3f}", "z": "0", "h": f"{math.radians(heading_deg):.5f}", "p": "0", "r": "0",
    })
    return pos


def _teleport_init(actions: Element, entity_ref: str, x: float, y: float, heading_deg: float) -> None:
    private = SubElement(actions, "Private", {"entityRef": entity_ref})
    pa = SubElement(private, "PrivateAction")
    tp = SubElement(pa, "TeleportAction")
    _world_position(tp, x, y, heading_deg)


def _follow_trajectory_maneuver_group(
    story: Element, entity_ref: str, waypoints: list[tuple[float, float, float, float]]
) -> None:
    """waypoints: list of (t_s, x, y, heading_deg)."""
    mg = SubElement(story, "ManeuverGroup", {"name": f"MG_{entity_ref}", "maximumExecutionCount": "1"})
    actors = SubElement(mg, "Actors", {"selectTriggeringEntities": "false"})
    SubElement(actors, "EntityRef", {"entityRef": entity_ref})
    maneuver = SubElement(mg, "Maneuver", {"name": f"Maneuver_{entity_ref}"})
    event = SubElement(maneuver, "Event", {"name": f"Event_{entity_ref}", "priority": "overwrite"})
    action = SubElement(event, "Action", {"name": f"Action_{entity_ref}"})
    pa = SubElement(action, "PrivateAction")
    routing = SubElement(pa, "RoutingAction")
    fta = SubElement(routing, "FollowTrajectoryAction")
    trajectory = SubElement(fta, "Trajectory", {"name": f"Trajectory_{entity_ref}", "closed": "false"})
    SubElement(trajectory, "ParameterDeclarations")
    shape = SubElement(trajectory, "Shape")
    polyline = SubElement(shape, "Polyline")
    t0 = waypoints[0][0]
    for t_s, x, y, heading_deg in waypoints:
        vertex = SubElement(polyline, "Vertex", {"time": f"{t_s - t0:.3f}"})
        _world_position(vertex, x, y, heading_deg)
    SubElement(fta, "TimeReference").append(
        _timing_element()
    )
    SubElement(fta, "TrajectoryFollowingMode", {"followingMode": "position"})

    start_trigger = SubElement(event, "StartTrigger")
    cg = SubElement(start_trigger, "ConditionGroup")
    cond = SubElement(cg, "Condition", {"name": f"Start_{entity_ref}", "delay": "0", "conditionEdge": "rising"})
    bvc = SubElement(cond, "ByValueCondition")
    SubElement(bvc, "SimulationTimeCondition", {"value": f"{max(0.0, t0):.3f}", "rule": "greaterThan"})


def _timing_element() -> Element:
    timing = Element("Timing", {"domainAbsoluteRelative": "relative", "scale": "1.0", "offset": "0.0"})
    return timing


REAL_EGO_AS_VEHICLE_NAME = "Vehicle_RealEgo"


def _ego_waypoints(poses: list, t0_us: int) -> list[tuple[float, float, float, float]]:
    """(t_rel_s, x, y, heading_deg) -- EgoPose.heading_deg is ADMA
    (CCW-from-north) convention, so needs the +90 conversion to match the
    math convention every other waypoint here (and OpenSCENARIO's own `h`
    field) uses.
    """
    return [((p.t_us - t0_us) / 1e6, p.x_m, p.y_m, (90 + p.heading_deg) % 360) for p in poses]


def _vehicle_waypoints(observations: list, t0_us: int) -> list[tuple[float, float, float, float]]:
    return [((o.t_us - t0_us) / 1e6, o.x_m, o.y_m, o.heading_deg) for o in observations]


def generate_openscenario(
    trace: Trace, xodr_filename: str, ego_time_stride: int = 20, pov_vehicle_id: int | None = None
) -> str:
    """By default, replays the recorded ego as "Ego" and every annotated
    vehicle as a FollowTrajectoryAction entity.

    `pov_vehicle_id`, when given, re-roots the scenario from that vehicle's
    point of view instead: its own recorded path becomes "Ego", the real
    ego becomes a regular vehicle entity (`REAL_EGO_AS_VEHICLE_NAME`), and
    the whole scenario is truncated to that vehicle's own observed time
    window (its first through last observation) -- the only span for which
    its trajectory is actually known. The road network (.xodr) is reused
    unchanged; only which agent is "Ego" and the exported time range
    change.
    """
    if pov_vehicle_id is not None:
        if pov_vehicle_id not in trace.annotation.vehicles or not trace.annotation.vehicles[pov_vehicle_id].observations:
            raise ValueError(f"vehicle {pov_vehicle_id} has no observations to use as a POV")
        pov_track = trace.annotation.vehicles[pov_vehicle_id]
        pov_obs = sorted(pov_track.observations, key=lambda o: o.t_us)
        t0_us, t1_us = pov_obs[0].t_us, pov_obs[-1].t_us
        ego_category = _vehicle_category(pov_track.obj_type)
        ego_dims = (pov_obs[0].length, pov_obs[0].width, pov_obs[0].height)
        ego_waypoints = _vehicle_waypoints(pov_obs, t0_us)
        other_tracks = {
            vid: track for vid, track in trace.annotation.vehicles.items() if vid != pov_vehicle_id
        }
        real_ego_poses = [p for p in trace.ego.poses if t0_us <= p.t_us <= t1_us]
    else:
        t0_us, t1_us = trace.ego.t0_us, trace.ego.t1_us
        ego_category, ego_dims = "car", (EGO_LENGTH_M, EGO_WIDTH_M, EGO_HEIGHT_M)
        ego_waypoints = _ego_waypoints(_downsample_time(trace.ego.poses, ego_time_stride), t0_us)
        other_tracks = trace.annotation.vehicles
        real_ego_poses = []

    osc = Element("OpenSCENARIO")
    now = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S")
    SubElement(osc, "FileHeader", {
        "revMajor": "1", "revMinor": "2", "date": now,
        "description": f"trace_fixer export of {trace.trace_id}", "author": "trace_fixer",
    })
    SubElement(osc, "ParameterDeclarations")
    SubElement(osc, "CatalogLocations")

    road_network = SubElement(osc, "RoadNetwork")
    SubElement(road_network, "LogicFile", {"filepath": xodr_filename})

    entities = SubElement(osc, "Entities")
    _vehicle_scenario_object(entities, ENTITY_NAME_EGO, ego_category, *ego_dims)
    entity_waypoints: dict[str, list[tuple[float, float, float, float]]] = {}
    if len(real_ego_poses) >= 2:
        _vehicle_scenario_object(entities, REAL_EGO_AS_VEHICLE_NAME, "car", EGO_LENGTH_M, EGO_WIDTH_M, EGO_HEIGHT_M)
        entity_waypoints[REAL_EGO_AS_VEHICLE_NAME] = _ego_waypoints(real_ego_poses, t0_us)

    for track in other_tracks.values():
        obs = [o for o in track.observations if t0_us <= o.t_us <= t1_us]
        if not obs:
            continue
        name = f"Vehicle_{track.obj_id}"
        obs0 = obs[0]
        _vehicle_scenario_object(entities, name, _vehicle_category(track.obj_type), obs0.length, obs0.width, obs0.height)
        entity_waypoints[name] = _vehicle_waypoints(obs, t0_us)

    storyboard = SubElement(osc, "Storyboard")
    init = SubElement(storyboard, "Init")
    init_actions = SubElement(init, "Actions")

    _teleport_init(init_actions, ENTITY_NAME_EGO, *ego_waypoints[0][1:])
    for name, waypoints in entity_waypoints.items():
        _teleport_init(init_actions, name, *waypoints[0][1:])

    story = SubElement(storyboard, "Story", {"name": "MainStory"})
    act = SubElement(story, "Act", {"name": "Act1"})

    _follow_trajectory_maneuver_group(act, ENTITY_NAME_EGO, ego_waypoints)
    for name, waypoints in entity_waypoints.items():
        _follow_trajectory_maneuver_group(act, name, waypoints)

    act_start_trigger = SubElement(act, "StartTrigger")
    act_cg = SubElement(act_start_trigger, "ConditionGroup")
    act_cond = SubElement(act_cg, "Condition", {"name": "ActStart", "delay": "0", "conditionEdge": "rising"})
    act_bvc = SubElement(act_cond, "ByValueCondition")
    SubElement(act_bvc, "SimulationTimeCondition", {"value": "0", "rule": "greaterThan"})

    duration_s = (t1_us - t0_us) / 1e6
    stop_trigger = SubElement(storyboard, "StopTrigger")
    stop_cg = SubElement(stop_trigger, "ConditionGroup")
    stop_cond = SubElement(stop_cg, "Condition", {"name": "EndCondition", "delay": "0", "conditionEdge": "rising"})
    stop_bvc = SubElement(stop_cond, "ByValueCondition")
    SubElement(stop_bvc, "SimulationTimeCondition", {"value": f"{duration_s:.3f}", "rule": "greaterThan"})

    xml_bytes = tostring(osc, encoding="utf-8")
    pretty = minidom.parseString(xml_bytes).toprettyxml(indent="  ")
    return pretty
