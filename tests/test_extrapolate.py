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
