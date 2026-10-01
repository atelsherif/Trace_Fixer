"""Tests for trace_fixer.prediction.extrapolate's per-direction horizon,
in particular the longer horizon for a vehicle trailing the ego -- see
REAR_HORIZON_MULTIPLIER's docstring for why.
"""
from trace_fixer.models import Annotation, EgoPose, EgoTrace, Trace, VehicleObs, VehicleTrack


def _ego_trace(n_seconds: int = 60) -> EgoTrace:
    # heading_deg=270 -> math-convention yaw 0 (see geo.transform.heading_to_yaw_rad),
    # i.e. "pointing along +x", matching the poses' own x_m progression below.
    poses = [
        EgoPose(t_us=i * 1_000_000, lat_deg=0, lon_deg=0, heading_deg=270.0, vx_mps=10, vy_mps=0, vz_mps=0)
        for i in range(n_seconds)
    ]
    for i, p in enumerate(poses):
        p.x_m, p.y_m = i * 10.0, 0.0
    return EgoTrace(poses=poses)


def _obs(t_s: float, frame: int, x_rel: float, x_m: float) -> VehicleObs:
    return VehicleObs(
        t_us=int(t_s * 1e6),
        frame=frame,
        obj_movement="Moving",
        obj_lane="EGO lane",
        obj_confidence="High",
        x_rel=x_rel,
        y_rel=0.0,
        z_rel=0.0,
        length=4.5,
        width=1.9,
        height=1.5,
        zrot=0.0,
        x_m=x_m,
        y_m=0.0,
        heading_deg=0.0,
    )


def test_predict_forward_gives_a_trailing_vehicle_a_longer_horizon():
    """A vehicle last seen behind the ego (x_rel < 0, e.g. the ego pulled
    away from it) gets REAR_HORIZON_MULTIPLIER times the base horizon;
    one last seen ahead gets the base horizon unchanged."""
    from trace_fixer.prediction.extrapolate import (
        DEFAULT_STEP_S,
        REAR_HORIZON_MULTIPLIER,
        predict_forward,
    )

    trace = Trace(trace_id="t", ego=_ego_trace(), annotation=Annotation(country_code=None))
    horizon_s = 4.0

    ahead = VehicleTrack(obj_id=1, obj_type="Car", reflecting_parts=None, observations=[
        _obs(15.0, 100, x_rel=20.0, x_m=170.0), _obs(16.0, 101, x_rel=25.0, x_m=180.0),
    ])
    behind = VehicleTrack(obj_id=2, obj_type="Car", reflecting_parts=None, observations=[
        _obs(15.0, 100, x_rel=-20.0, x_m=130.0), _obs(16.0, 101, x_rel=-25.0, x_m=120.0),
    ])

    n_ahead = predict_forward(ahead, trace, horizon_s=horizon_s)
    n_behind = predict_forward(behind, trace, horizon_s=horizon_s)

    assert n_ahead == round(horizon_s / DEFAULT_STEP_S)
    assert n_behind == round(horizon_s * REAR_HORIZON_MULTIPLIER / DEFAULT_STEP_S)
    assert n_behind == n_ahead * REAR_HORIZON_MULTIPLIER


def test_predict_backward_gives_a_trailing_vehicle_a_longer_horizon():
    """Same rule for the pre-FOV direction: a vehicle first seen already
    behind the ego gets the longer horizon looking further into the past."""
    from trace_fixer.prediction.extrapolate import (
        DEFAULT_STEP_S,
        REAR_HORIZON_MULTIPLIER,
        predict_backward,
    )

    trace = Trace(trace_id="t", ego=_ego_trace(), annotation=Annotation(country_code=None))
    horizon_s = 4.0

    ahead = VehicleTrack(obj_id=1, obj_type="Car", reflecting_parts=None, observations=[
        _obs(15.0, 100, x_rel=20.0, x_m=170.0), _obs(16.0, 101, x_rel=25.0, x_m=180.0),
    ])
    behind = VehicleTrack(obj_id=2, obj_type="Car", reflecting_parts=None, observations=[
        _obs(15.0, 100, x_rel=-20.0, x_m=130.0), _obs(16.0, 101, x_rel=-15.0, x_m=140.0),
    ])

    n_ahead = predict_backward(ahead, trace, horizon_s=horizon_s)
    n_behind = predict_backward(behind, trace, horizon_s=horizon_s)

    assert n_ahead == round(horizon_s / DEFAULT_STEP_S)
    assert n_behind == round(horizon_s * REAR_HORIZON_MULTIPLIER / DEFAULT_STEP_S)


def test_horizon_m_caps_distance_not_just_time():
    """Whichever of horizon_s/horizon_m is reached first stops the
    prediction -- a fast-moving vehicle capped by distance should predict
    fewer steps than the same call with no distance cap at all."""
    from trace_fixer.prediction.extrapolate import DEFAULT_STEP_S, predict_forward

    trace = Trace(trace_id="t", ego=_ego_trace(), annotation=Annotation(country_code=None))
    track = VehicleTrack(obj_id=1, obj_type="Car", reflecting_parts=None, observations=[
        _obs(15.0, 100, x_rel=20.0, x_m=170.0), _obs(16.0, 101, x_rel=25.0, x_m=180.0),  # 10 m/s
    ])
    n_unbounded = predict_forward(track, trace, horizon_s=10.0, avoid_collisions=False)
    assert n_unbounded == round(10.0 / DEFAULT_STEP_S)

    track2 = VehicleTrack(obj_id=1, obj_type="Car", reflecting_parts=None, observations=[
        _obs(15.0, 100, x_rel=20.0, x_m=170.0), _obs(16.0, 101, x_rel=25.0, x_m=180.0),
    ])
    # at 10 m/s, 30m is reached in 3s -- well inside the 10s time cap.
    n_capped = predict_forward(track2, trace, horizon_s=10.0, horizon_m=30.0, avoid_collisions=False)
    assert n_capped < n_unbounded
    assert n_capped == round(3.0 / DEFAULT_STEP_S)


def test_avoid_collisions_brakes_instead_of_driving_through_a_stationary_vehicle():
    """A vehicle predicted straight into another (stationary) vehicle's
    box must brake -- not drive through it -- when avoid_collisions is on,
    and must reach further without it, per extrapolate.py's own
    following-distance-governor design (speed only, no steering)."""
    from trace_fixer.prediction.extrapolate import DEFAULT_STEP_S, predict_forward

    ego = _ego_trace()
    # Vehicle 2: parked directly ahead of vehicle 1's straight-line path,
    # for the whole span vehicle 1's prediction could possibly reach.
    obstacle_obs = [
        _obs(t, 200 + i, x_rel=50.0, x_m=200.0) for i, t in enumerate([15.0 + k * 0.5 for k in range(40)])
    ]
    obstacle = VehicleTrack(obj_id=2, obj_type="Car", reflecting_parts=None, observations=obstacle_obs)

    def make_moving_track():
        return VehicleTrack(obj_id=1, obj_type="Car", reflecting_parts=None, observations=[
            _obs(15.0, 100, x_rel=20.0, x_m=170.0), _obs(16.0, 101, x_rel=25.0, x_m=180.0),  # 10 m/s toward x=200
        ])

    trace_guarded = Trace(
        trace_id="t", ego=ego, annotation=Annotation(country_code=None, vehicles={2: obstacle})
    )
    guarded_track = make_moving_track()
    trace_guarded.annotation.vehicles[1] = guarded_track
    predict_forward(guarded_track, trace_guarded, horizon_s=10.0, step_s=DEFAULT_STEP_S, avoid_collisions=True)

    trace_unguarded = Trace(
        trace_id="t", ego=_ego_trace(), annotation=Annotation(country_code=None, vehicles={2: obstacle})
    )
    unguarded_track = make_moving_track()
    trace_unguarded.annotation.vehicles[1] = unguarded_track
    predict_forward(unguarded_track, trace_unguarded, horizon_s=10.0, step_s=DEFAULT_STEP_S, avoid_collisions=False)

    guarded_synth = [o for o in guarded_track.observations if o.synthetic]
    unguarded_synth = [o for o in unguarded_track.observations if o.synthetic]
    guarded_max_x = max(o.x_m for o in guarded_synth)
    unguarded_max_x = max(o.x_m for o in unguarded_synth)

    # Unguarded drives straight through the obstacle parked at x=200.
    assert unguarded_max_x > 220.0
    # Guarded brakes and stops short of it...
    assert guarded_max_x < 200.0
    # ...and truncates rather than emitting a stationary ghost sitting in
    # the obstacle for every remaining step: braking is rate-limited, so
    # once even a full stop can't open a gap there is no plausible
    # continuation left to predict.
    assert len(guarded_synth) < len(unguarded_synth)
    step_distances = [
        abs(b.x_m - a.x_m) for a, b in zip(guarded_synth, guarded_synth[1:])
    ]
    assert all(d > 0.01 for d in step_distances), "should truncate, not park in place"


# --- Track continuation detection -------------------------------------

def _track(obj_id, obj_type, obs):
    return VehicleTrack(obj_id=obj_id, obj_type=obj_type, reflecting_parts=None, observations=obs)


def _moving_obs(t_s, frame, x_m, *, y_rel=-3.8, width=2.8, lane="1st Right", length=14.0):
    o = _obs(t_s, frame, x_rel=100.0, x_m=x_m)
    o.y_rel, o.width, o.obj_lane, o.length = y_rel, width, lane, length
    return o


def _continuation_trace(**second_track_overrides):
    """One truck observed 0-8s, then re-acquired as a new id 9.3-12s after
    a 1.3s tracking gap -- the shape of the real case on sample1."""
    ego = _ego_trace()
    first = _track(3, "Truck", [_moving_obs(t, 100 + i, 200.0 + 30.0 * t) for i, t in enumerate([7.0, 8.0])])
    second = _track(
        4,
        second_track_overrides.pop("obj_type", "Truck"),
        [_moving_obs(t, 200 + i, 200.0 + 30.0 * t, **second_track_overrides) for i, t in enumerate([9.3, 10.3])],
    )
    trace = Trace(
        trace_id="t", ego=ego,
        annotation=Annotation(country_code=None, vehicles={3: first, 4: second}),
    )
    return trace


def test_continuation_is_detected_across_a_tracking_gap():
    """The real sample1 case: one truck, two ids, a 1.3s gap. Same lane,
    same lateral offset, same width, and a gap bridged at exactly the
    speed both ends were observed travelling."""
    from trace_fixer.prediction.extrapolate import find_track_continuations

    assert find_track_continuations(_continuation_trace()) == {3: 4}


def test_a_different_vehicle_is_not_absorbed_as_a_continuation():
    """Each criterion on its own must be able to veto the match --
    otherwise a genuinely separate vehicle's prediction gets suppressed."""
    from trace_fixer.prediction.extrapolate import find_track_continuations

    assert find_track_continuations(_continuation_trace(lane="EGO lane")) == {}, "different lane"
    assert find_track_continuations(_continuation_trace(y_rel=3.9)) == {}, "different lateral offset"
    assert find_track_continuations(_continuation_trace(width=1.8)) == {}, "different width"
    assert find_track_continuations(_continuation_trace(obj_type="Car")) == {}, "different object type"


def test_a_gap_no_speed_could_bridge_is_not_a_continuation():
    """The dominant real-world discriminator: a coincidental neighbour
    would have to travel at hundreds of km/h to be the same vehicle."""
    from trace_fixer.prediction.extrapolate import find_track_continuations

    trace = _continuation_trace()
    # move the successor 200m further along -- same lane, same size, but it
    # would need ~150 m/s to have got there
    for o in trace.annotation.vehicles[4].observations:
        o.x_m += 200.0
    assert find_track_continuations(trace) == {}


def test_only_the_earlier_track_predicts_across_a_continuation_gap():
    """The point of detecting continuations: two ghosts of one vehicle in
    the same gap is what validation reports as a collision."""
    from trace_fixer.prediction.extrapolate import predict_all

    trace = _continuation_trace()
    added = predict_all(trace, horizon_s=4.0)

    # the successor must not predict backward into the gap its predecessor
    # already explains
    assert "backward" not in added.get(4, {})
    # ...and the predecessor's forward prediction stops where the
    # successor's own real observations begin
    predecessor_synthetic = [o for o in trace.annotation.vehicles[3].observations if o.synthetic]
    successor_first = trace.annotation.vehicles[4].observations[0]
    assert predecessor_synthetic
    assert max(o.t_us for o in predecessor_synthetic) <= successor_first.t_us


def test_link_continuations_can_be_turned_off():
    from trace_fixer.prediction.extrapolate import predict_all

    trace = _continuation_trace()
    added = predict_all(trace, horizon_s=4.0, link_continuations=False)
    assert "backward" in added.get(4, {})


# --- The governor only reacts to what's in front -----------------------

def test_a_predicted_vehicle_does_not_brake_for_a_tailgater():
    """Braking can only open a gap to something ahead; braking for a
    vehicle *behind* closes the gap to it. Two long trucks nose-to-tail had
    the leader brake for its own follower and then stop predicting
    altogether, because no amount of braking could ever clear it -- which
    is what left the follower with nothing to follow and produced the
    overlap this whole governor exists to prevent.
    """
    from trace_fixer.prediction.extrapolate import DEFAULT_STEP_S, predict_forward

    ego = _ego_trace()
    # A 20 m truck sitting 21 m behind the leader, i.e. close enough that
    # the two boxes (plus the safety margin) already overlap.
    follower_obs = [
        _obs(t, 200 + i, x_rel=-30.0, x_m=179.0 + 10.0 * (t - 15.0))
        for i, t in enumerate([15.0 + k * 0.5 for k in range(40)])
    ]
    for o in follower_obs:
        o.length, o.width = 20.0, 2.8
    follower = VehicleTrack(obj_id=2, obj_type="Truck", reflecting_parts=None, observations=follower_obs)

    leader = VehicleTrack(obj_id=1, obj_type="Truck", reflecting_parts=None, observations=[
        _obs(15.0, 100, x_rel=-10.0, x_m=200.0), _obs(16.0, 101, x_rel=-10.0, x_m=210.0),
    ])
    for o in leader.observations:
        o.length, o.width = 20.0, 2.8

    def predicted(avoid_collisions):
        track = VehicleTrack(obj_id=1, obj_type="Truck", reflecting_parts=None, observations=[
            _obs(15.0, 100, x_rel=-10.0, x_m=200.0), _obs(16.0, 101, x_rel=-10.0, x_m=210.0),
        ])
        for o in track.observations:
            o.length, o.width = 20.0, 2.8
        trace = Trace(
            trace_id="t", ego=_ego_trace(),
            annotation=Annotation(country_code=None, vehicles={1: track, 2: follower}),
        )
        predict_forward(track, trace, horizon_s=10.0, step_s=DEFAULT_STEP_S, avoid_collisions=avoid_collisions)
        return [o for o in track.observations if o.synthetic]

    guarded, unguarded = predicted(True), predicted(False)
    # Compared against the ungoverned run rather than a step count computed
    # here, since the horizon this vehicle actually gets depends on
    # REAR_HORIZON_MULTIPLIER (it is itself behind the ego).
    assert len(guarded) == len(unguarded), "a tailgater must not truncate the leader"
    # ...and it never slowed for it either: every step covers the same
    # ground as the ungoverned one.
    for a, b in zip(guarded, unguarded):
        assert abs(a.x_m - b.x_m) < 1e-9


def test_predicted_vehicles_are_stepped_against_each_others_live_positions():
    """predict_all steps every vehicle in one chronological sweep. The
    regression: predicting each vehicle's whole path in turn let a vehicle
    governed early pick its speed against a path that a vehicle governed
    later then replaced, so the two ended up overlapping even though both
    were governed. This is the sample1 vehicle-1-vs-vehicle-4 case,
    reduced: a long truck with an early anchor closing on a long truck
    whose own prediction starts later and stops sooner.
    """
    import math

    from trace_fixer.geo.collision import obb_overlap
    from trace_fixer.prediction.extrapolate import predict_all

    ego = _ego_trace()

    def truck(obj_id, times, x0, speed, length):
        obs = [_obs(t, 100 + i, x_rel=-20.0, x_m=x0 + speed * (t - times[0])) for i, t in enumerate(times)]
        for o in obs:
            o.length, o.width = length, 2.8
        return VehicleTrack(obj_id=obj_id, obj_type="Truck", reflecting_parts=None, observations=obs)

    # Chaser is faster and starts predicting ~9 s earlier than the leader,
    # so a per-vehicle pass would decide its whole path before the leader
    # had settled on its own.
    chaser = truck(1, [24.0, 25.0, 26.0], 1000.0, 30.0, 19.8)
    leader = truck(4, [33.0, 34.0, 35.0], 1180.0, 24.0, 21.5)
    trace = Trace(
        trace_id="t", ego=ego,
        annotation=Annotation(country_code=None, vehicles={1: chaser, 4: leader}),
    )
    predict_all(trace, horizon_s=6.0)

    a = trace.annotation.vehicles[1].observations
    b = {o.t_us: o for o in trace.annotation.vehicles[4].observations}
    b_times = sorted(b)
    tolerance_us = 300_000  # same window validation/checks.py compares over
    for oa in a:
        near = [t for t in b_times if abs(t - oa.t_us) <= tolerance_us]
        for t in near:
            ob = b[t]
            assert not obb_overlap(
                oa.x_m, oa.y_m, oa.heading_deg, oa.length, oa.width,
                ob.x_m, ob.y_m, ob.heading_deg, ob.length, ob.width,
            ), f"predicted boxes overlap at t={oa.t_us / 1e6:.2f}s"


def test_sample1_prediction_introduces_no_collisions_at_any_horizon():
    """The reported bug, end to end: on sample1, vehicles 1 and 4 collided
    in their predictions at horizons of 6 s and above."""
    from pathlib import Path

    from trace_fixer.prediction.extrapolate import predict_all
    from trace_fixer.scene import load_trace
    from trace_fixer.validation.checks import run_validation

    sample1 = Path(__file__).resolve().parents[1] / "data" / "traces" / "sample1_baseline_1"
    for horizon_s in (4.0, 6.0, 10.0, 15.0):
        trace = load_trace("sample1", sample1 / "adma.csv", sample1 / "annotation.xml")
        assert run_validation(trace) == [], "sample1 is clean before prediction"
        predict_all(trace, horizon_s=horizon_s)
        collisions = [i for i in run_validation(trace) if i.category == "collision"]
        assert collisions == [], f"horizon {horizon_s}s: {[i.description for i in collisions]}"


def test_the_governor_removes_collisions_without_adding_kinematic_issues():
    """sample2 is the busy one -- 29+ predicted collisions ungoverned. The
    governor has to clear them by braking plausibly, not by braking so hard
    it trades one category of issue for another."""
    from collections import Counter
    from pathlib import Path

    from trace_fixer.prediction.extrapolate import predict_all
    from trace_fixer.scene import load_trace
    from trace_fixer.validation.checks import run_validation

    sample2 = Path(__file__).resolve().parents[1] / "data" / "traces" / "sample2_baseline_2"

    def counts(avoid_collisions):
        trace = load_trace("sample2", sample2 / "adma.csv", sample2 / "annotation.xml")
        run_validation(trace)
        predict_all(trace, horizon_s=10.0, avoid_collisions=avoid_collisions)
        return Counter(i.category for i in run_validation(trace))

    ungoverned = counts(False)
    governed = counts(True)
    assert ungoverned["collision"] > 0, "fixture must still have collisions to clear"
    assert governed["collision"] == 0
    # braking stays under the validator's own "unrealistic acceleration"
    # threshold, so clearing the collisions costs nothing elsewhere
    assert governed["kinematic"] <= ungoverned["kinematic"]
