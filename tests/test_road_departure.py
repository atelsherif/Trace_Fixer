"""Tests for telling a real road departure apart from annotation noise, and
for the fix engine refusing to "correct" the former.

The bug these guard against: the annotated Road Edge / Guardrail polylines
only describe the ego's own road, so a vehicle turning off it is outside the
corridor by construction. The off-road clamp used to drag it back into the
ego's lane and report that as a fix, producing a repaired trace in which a
vehicle followed the ego down a road it had already left.
"""
import math
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
# A real motorway trace that still has off-road and road-departure
# issues after the corridor check learned to defer to the annotation's
# own lane labels. sample2's vehicles are all lane-labelled, so it no
# longer exercises those categories at all -- only kinematic ones.
OFFROAD_DIR = REPO_ROOT / "data" / "traces" / "LB-VS-271_20200722_split_068_MERGED_baseline_6"


def _straight_border(obj_id, y_rel, times, x_from=-60.0, x_to=120.0, step=10.0):
    """A road edge running parallel to the ego at a constant ego-relative y."""
    from trace_fixer.models import BorderLine, LaneSnapshot

    n = int((x_to - x_from) / step) + 1
    points = [(x_from + i * step, y_rel) for i in range(n)]
    return BorderLine(
        obj_id=obj_id,
        obj_type="Road Edge",
        snapshots=[LaneSnapshot(t_us=t, frame=i, points_rel=points) for i, t in enumerate(times)],
    )


def _track(y_rels, times, heading_deg=270.0, width=1.8):
    """A vehicle driving parallel to +x, whose ego-relative y follows y_rels.

    Positions are laid out in the global frame too, spaced 4 m apart, so the
    path-tangent measurements the classifier makes have something real to
    read. heading_deg 270 is +x travel (see geo.transform.heading_to_yaw_rad).
    """
    from trace_fixer.models import VehicleObs, VehicleTrack

    obs = []
    for i, (y_rel, t) in enumerate(zip(y_rels, times)):
        obs.append(
            VehicleObs(
                t_us=t,
                frame=i,
                obj_movement="moving",
                # "Other" rather than a numbered lane: the corridor check
                # defers to the annotation's own lane label, and a vehicle
                # the annotator placed in a lane is on the road whatever the
                # Road Edge polylines say (see geo.road_corridor's
                # ON_ROAD_LANE_LABELS). These fixtures are about vehicles
                # that genuinely are not in a lane.
                obj_lane="Other",
                obj_confidence="high",
                x_rel=20.0,
                y_rel=y_rel,
                z_rel=0.0,
                length=4.5,
                width=width,
                height=1.5,
                zrot=0.0,
                x_m=float(i * 4),
                y_m=y_rel,
                heading_deg=heading_deg,
            )
        )
    return VehicleTrack(obj_id=1, obj_type="Car", reflecting_parts=None, observations=obs)


TIMES = [i * 100_000 for i in range(12)]


def _scene(y_rels, border, times=None):
    """A Trace assembled the way a real one is -- including
    populate_global_coords, which is what fills the border snapshots'
    `points_m`. The corridor check reads those global points and reprojects
    them into each observation's own ego frame (see geo.road_corridor), so
    a fixture that only sets `points_rel` describes a road with no geometry
    in it.

    Returns (track, border_lines, ego_interpolator) ready for offroad_runs.
    """
    from trace_fixer.geo.populate import populate_global_coords
    from trace_fixer.geo.transform import EgoInterpolator
    from trace_fixer.models import Annotation, EgoPose, EgoTrace, Trace

    times = times or TIMES
    # Ego crawling along +x at 1 m/s so the ego frame is effectively fixed:
    # these fixtures are about the corridor, not about ego motion.
    poses = [
        EgoPose(t_us=t, lat_deg=0.0, lon_deg=0.0, heading_deg=270.0, vx_mps=1.0, vy_mps=0.0, vz_mps=0.0)
        for t in range(0, max(times) + 1_000_000, 100_000)
    ]
    track = _track(y_rels, times)
    trace = Trace(
        trace_id="t", ego=EgoTrace(poses=poses),
        annotation=Annotation(country_code=None, vehicles={1: track}, border_lines=border),
    )
    populate_global_coords(trace)
    return track, trace.annotation.border_lines, EgoInterpolator(trace.ego)


def test_a_box_parked_just_outside_the_edge_is_noise_not_a_departure():
    """The case the clamp exists for: road-parallel, sub-meter, comes back."""
    from trace_fixer.geo.road_departure import offroad_runs

    border = {1: _straight_border(1, -3.0, TIMES)}
    y = [-2.0] * 4 + [-4.2] * 3 + [-2.0] * 5  # ~0.4 m past the edge, briefly
    runs = offroad_runs(*_scene(y, border))
    assert len(runs) == 1
    assert not runs[0].is_departure
    assert runs[0].side == "right"


def test_a_vehicle_that_turns_off_the_road_is_a_departure():
    from trace_fixer.geo.road_departure import offroad_runs

    border = {1: _straight_border(1, -3.0, TIMES)}
    # steadily peels off to the right and never returns
    y = [-2.0] * 3 + [-4.0, -6.0, -8.0, -10.0, -12.0, -14.0, -16.0, -18.0, -20.0]
    runs = offroad_runs(*_scene(y[: len(TIMES)], border))
    assert len(runs) == 1
    assert runs[0].is_departure
    assert runs[0].peak_m > 4.0


def test_one_turn_off_is_one_issue_not_one_per_frame():
    """A contiguous excursion is a single event. Reporting it per observation
    buried the trace's real problems under dozens of identical rows."""
    from trace_fixer.geo.road_departure import offroad_runs

    border = {1: _straight_border(1, -3.0, TIMES)}
    y = [-2.0] * 2 + [-6.0 - i for i in range(10)]
    runs = offroad_runs(*_scene(y[: len(TIMES)], border))
    assert len(runs) == 1
    assert runs[0].t_start_us == TIMES[2]
    assert runs[0].t_end_us == TIMES[-1]


def test_the_fix_engine_leaves_a_departing_vehicle_exactly_where_it_was():
    """The headline regression: a vehicle that left the road must come back
    from Apply fixes unmoved, not re-drawn onto the ego's road."""
    from trace_fixer.scene import load_trace
    from trace_fixer.validation.checks import run_validation
    from trace_fixer.validation.fixes import apply_fixes

    trace = load_trace("offroad", OFFROAD_DIR / "adma.csv", OFFROAD_DIR / "annotation.xml")
    issues = run_validation(trace)
    departures = [i for i in issues if i.category == "road_departure"]
    assert departures, "fixture must still contain a vehicle that leaves the ego's road"

    recorded = {
        (track.obj_id, o.t_us): (o.x_m, o.y_m)
        for track in trace.annotation.vehicles.values()
        for o in track.observations
    }
    apply_fixes(trace)

    for issue in departures:
        track = trace.annotation.vehicles[issue.vehicle_id]
        during = [o for o in track.observations if issue.t_start_us <= o.t_us <= issue.t_end_us]
        assert during
        for o in during:
            x0, y0 = recorded[(track.obj_id, o.t_us)]
            assert math.hypot(o.x_m - x0, o.y_m - y0) < 1e-9


def test_a_departure_is_reported_for_review_and_never_marked_fixable():
    from trace_fixer.scene import load_trace
    from trace_fixer.validation.checks import run_validation

    trace = load_trace("offroad", OFFROAD_DIR / "adma.csv", OFFROAD_DIR / "annotation.xml")
    departures = [i for i in run_validation(trace) if i.category == "road_departure"]
    assert departures
    for issue in departures:
        assert issue.fixable is False
        assert issue.severity == "low"
        # the description has to carry the evidence, or a reviewer can't
        # judge whether the classifier got it right
        assert "m beyond the edge" in issue.description
        assert "off its earlier course" in issue.description


def test_the_clamp_declines_corrections_larger_than_it_can_justify():
    """Even for a run the classifier calls noise, a multi-meter "correction"
    means the corridor geometry is what's wrong -- so the recorded position
    stands and the flag stands with it."""
    from trace_fixer.scene import load_trace
    from trace_fixer.validation.checks import run_validation
    from trace_fixer.validation.fixes import MAX_CLAMP_M, apply_fixes

    trace = load_trace("offroad", OFFROAD_DIR / "adma.csv", OFFROAD_DIR / "annotation.xml")
    run_validation(trace)
    apply_fixes(trace)

    for track in trace.annotation.vehicles.values():
        for o in track.observations:
            assert o.moved_from_original_m <= MAX_CLAMP_M + 1e-6


def test_an_oncoming_vehicle_outside_the_edge_is_not_read_as_turned_away():
    """Attitude is measured against the road's axis, not its direction: a
    vehicle facing 180 deg from the ego is road-parallel, not maximally
    turned off. Without the fold it would be a departure on attitude alone.
    """
    from trace_fixer.geo.road_departure import _axis_angle_deg

    assert _axis_angle_deg(0.0) == pytest.approx(0.0)
    assert _axis_angle_deg(math.pi) == pytest.approx(0.0)
    assert _axis_angle_deg(math.radians(-175.0)) == pytest.approx(5.0)
    assert _axis_angle_deg(math.radians(90.0)) == pytest.approx(90.0)
