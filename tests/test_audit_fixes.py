"""Regression tests for defects found in a review pass over the whole tool,
rather than while building any one feature. Each names the wrong behaviour
it replaced, since none of them announced themselves -- every one produced
a plausible-looking answer that happened to be wrong.
"""
import tempfile
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
SAMPLE2_DIR = REPO_ROOT / "data" / "traces" / "sample2"


def _ego(seconds: int = 60, speed: float = 20.0):
    from trace_fixer.models import EgoPose, EgoTrace

    poses = []
    for i in range(seconds * 10):
        p = EgoPose(
            t_us=i * 100_000, lat_deg=0, lon_deg=0, heading_deg=270.0,
            vx_mps=speed, vy_mps=0, vz_mps=0,
        )
        p.x_m, p.y_m = i * speed / 10.0, 0.0
        poses.append(p)
    return EgoTrace(poses=poses)


def _obs(t_s: float, x_rel: float):
    from trace_fixer.models import VehicleObs

    return VehicleObs(
        t_us=int(t_s * 1e6), frame=int(t_s * 10), obj_movement="Moving", obj_lane="EGO lane",
        obj_confidence="High", x_rel=x_rel, y_rel=0.0, z_rel=0.0,
        length=4.5, width=1.9, height=1.5, zrot=0.0,
    )


def _trace_with(observations, sync_offset_us: int = 0):
    from trace_fixer.models import Annotation, Trace, VehicleTrack

    track = VehicleTrack(obj_id=1, obj_type="Car", reflecting_parts=None, observations=observations)
    return Trace(
        trace_id="t", ego=_ego(),
        annotation=Annotation(country_code=None, vehicles={1: track}),
        sync_offset_us=sync_offset_us,
    )


# --- analysis: short-headway episodes -----------------------------------

def test_two_separate_tailgates_are_two_events():
    """The merge loop had no "is this far enough apart to be a new episode"
    test, so it only ever extended the first group: every approach a vehicle
    made collapsed into one event spanning the first flagged observation to
    the last. Two one-second tailgates forty seconds apart came back as a
    single forty-one-second "short headway".
    """
    from trace_fixer.analysis import detect_short_headway_events

    # 10 m ahead at 20 m/s is a 0.5 s headway (flagged); 200 m ahead is 10 s.
    trace = _trace_with([
        _obs(5.0, 10.0), _obs(5.5, 10.0), _obs(6.0, 10.0),
        _obs(20.0, 200.0), _obs(30.0, 200.0),
        _obs(45.0, 12.0), _obs(45.5, 12.0), _obs(46.0, 12.0),
    ])
    events = detect_short_headway_events(trace)
    assert len(events) == 2
    assert [(e.t_start_us / 1e6, e.t_end_us / 1e6) for e in events] == [(5.0, 6.0), (45.0, 46.0)]
    # and neither swallows the other's span
    assert all(e.t_end_us - e.t_start_us <= 2_000_000 for e in events)


def test_a_single_episode_with_sparse_keyframes_still_merges():
    """The guard must not split one approach into one event per keyframe:
    annotation spacing runs to ~1.2 s on the bundled samples."""
    from trace_fixer.analysis import detect_short_headway_events

    trace = _trace_with([_obs(5.0, 10.0), _obs(6.2, 10.0), _obs(7.4, 10.0)])
    assert len(detect_short_headway_events(trace)) == 1


def test_headway_reads_the_ego_speed_through_the_sync_offset():
    """Headway is x_rel / ego speed, and the ego has to be sampled on the
    ego clock. Every other consumer of an annotation timestamp applies the
    sync offset; this one sampled the raw annotation time, so a trace with
    an offset set measured against the speed from the wrong instant."""
    from trace_fixer.geo.sync import apply_offset
    from trace_fixer.geo.transform import EgoInterpolator
    from trace_fixer.models import Annotation, EgoPose, EgoTrace, Trace, VehicleTrack

    # Ego accelerates hard, so "which instant" is measurable.
    poses = []
    for i in range(600):
        speed = 5.0 + i * 0.05
        p = EgoPose(t_us=i * 100_000, lat_deg=0, lon_deg=0, heading_deg=270.0,
                    vx_mps=speed, vy_mps=0, vz_mps=0)
        p.x_m, p.y_m = i * 2.0, 0.0
        poses.append(p)
    ego = EgoTrace(poses=poses)
    interp = EgoInterpolator(ego)

    offset = 3_000_000
    track = VehicleTrack(obj_id=1, obj_type="Car", reflecting_parts=None,
                         observations=[_obs(20.0, 15.0), _obs(20.5, 15.0), _obs(21.0, 15.0)])
    trace = Trace(trace_id="t", ego=ego,
                  annotation=Annotation(country_code=None, vehicles={1: track}),
                  sync_offset_us=offset)

    from trace_fixer.analysis import detect_short_headway_events

    events = detect_short_headway_events(trace)
    assert events, "this vehicle is close enough ahead to flag at either speed"

    def min_headway(apply_the_offset: bool) -> float:
        return min(
            15.0 / interp.at(apply_offset(o.t_us, offset) if apply_the_offset else o.t_us)[3]
            for o in track.observations
        )

    on_ego_clock, on_raw_clock = min_headway(True), min_headway(False)
    assert abs(on_ego_clock - on_raw_clock) > 0.01, "fixture must make the two distinguishable"
    assert events[0].min_headway_s == pytest.approx(on_ego_clock, rel=1e-6)


# --- scan: which annotation a trace gets paired with --------------------

def test_an_exact_annotation_beats_an_approximate_one():
    """Annotations were resolved in filesystem-walk order, so a file that
    only matched by prefix could claim a trace name and leave the trace's
    *actual* annotation to be dropped as a duplicate -- and since scandir
    order is undefined, which one won could differ between two scans of the
    same corpus."""
    from trace_fixer.scan import scan_for_trace_pairs

    root = Path(tempfile.mkdtemp())
    (root / "adma" / "ADMA" / "TraceA").mkdir(parents=True)
    (root / "adma" / "ADMA" / "TraceA" / "adma.csv").write_text("x")
    ann = root / "annotations" / "Annotations"
    ann.mkdir(parents=True)
    (ann / "TraceA_reprocessed_v2.xml").write_text("<x/>")
    (ann / "TraceA__ref-QC_IND.xml").write_text("<x/>")

    result = scan_for_trace_pairs(root)
    assert result.matched["TraceA"][1].name == "TraceA__ref-QC_IND.xml"
    assert result.xml_duplicate_count == 1


def test_scanning_the_same_corpus_twice_pairs_it_the_same_way():
    from trace_fixer.scan import scan_for_trace_pairs

    root = Path(tempfile.mkdtemp())
    for name in ("TraceA", "TraceB"):
        d = root / "adma" / "ADMA" / name
        d.mkdir(parents=True)
        (d / "adma.csv").write_text("x")
    ann = root / "annotations" / "Annotations"
    ann.mkdir(parents=True)
    for stem in ("TraceA__ref-QC_IND", "TraceA_copy", "TraceB__refQC_IND", "TraceB_v2"):
        (ann / f"{stem}.xml").write_text("<x/>")

    first = scan_for_trace_pairs(root)
    second = scan_for_trace_pairs(root)
    assert {k: v[1].name for k, v in first.matched.items()} == {
        k: v[1].name for k, v in second.matched.items()
    }


# --- validation: acceleration between unevenly spaced keyframes ---------

def test_acceleration_uses_the_centred_interval():
    """speeds[i] spans [i-1, i] and speeds[i+1] spans [i, i+1], so they sit
    half an interval either side of obs[i] and the time between them is
    (dt0 + dt1) / 2. Dividing by dt1 alone doubled the reported
    acceleration across a 0.4 s / 0.2 s keyframe pairing -- exactly what the
    seam between real annotation and 0.2 s prediction steps looks like.
    """
    from trace_fixer.models import Annotation, Trace, VehicleTrack
    from trace_fixer.validation.checks import run_validation

    # Constant 20 m/s then 22 m/s: a real +2 m/s over 0.3 s centred = 6.7 m/s^2.
    # Divided by dt1 = 0.2 s it reads 10 m/s^2, which crosses a severity band.
    obs = []
    for t_s, x in ((0.0, 0.0), (0.4, 8.0), (0.6, 12.4)):
        o = _obs(t_s, 20.0)
        o.x_m, o.y_m = x, 0.0
        obs.append(o)
    track = VehicleTrack(obj_id=1, obj_type="Car", reflecting_parts=None, observations=obs)
    trace = Trace(trace_id="t", ego=_ego(), annotation=Annotation(country_code=None, vehicles={1: track}))

    kinematic = [i for i in run_validation(trace) if i.category == "kinematic" and "acceleration" in i.description]
    reported = float(kinematic[0].description.split("acceleration")[1].split("m/s^2")[0])
    assert reported == pytest.approx((22.0 - 20.0) / 0.3, abs=0.05)


# --- prediction: the real/predicted seam --------------------------------

def test_prediction_does_not_snap_the_box_onto_its_direction_of_travel():
    """A predicted box used to be emitted pointing along the direction of
    travel from step one, while the real observation beside it carried the
    annotated box angle -- so the whole of the annotation's heading error
    (19 deg on sample2) landed in a single 0.2 s step, a ~95 deg/s yaw rate
    the validator then flagged at every seam. The box now starts where the
    annotation left it and converges, never faster than
    BOX_HEADING_RATE_DEG_S."""
    from trace_fixer.prediction.extrapolate import BOX_HEADING_RATE_DEG_S, predict_all
    from trace_fixer.scene import load_trace

    trace = load_trace("sample2", SAMPLE2_DIR / "adma.csv", SAMPLE2_DIR / "annotation.xml")
    predict_all(trace, horizon_s=10.0)

    def wrap(a):
        return (a + 180) % 360 - 180

    worst = 0.0
    for track in trace.annotation.vehicles.values():
        obs = sorted(track.observations, key=lambda o: o.t_us)
        for a, b in zip(obs, obs[1:]):
            if not (a.synthetic or b.synthetic):
                continue
            dt = (b.t_us - a.t_us) / 1e6
            if dt <= 0:
                continue
            worst = max(worst, abs(wrap(b.heading_deg - a.heading_deg)) / dt)
    assert worst <= BOX_HEADING_RATE_DEG_S + 1e-6


def test_prediction_adds_no_kinematic_issues_to_sample2():
    from trace_fixer.prediction.extrapolate import predict_all
    from trace_fixer.scene import load_trace
    from trace_fixer.validation.checks import run_validation

    trace = load_trace("sample2", SAMPLE2_DIR / "adma.csv", SAMPLE2_DIR / "annotation.xml")
    before = sum(1 for i in run_validation(trace) if i.category == "kinematic")
    predict_all(trace, horizon_s=10.0)
    after = sum(1 for i in run_validation(trace) if i.category == "kinematic")
    assert after == before


# --- batch runs must not touch the interactive session ------------------

def test_a_batch_run_ignores_what_the_session_did_to_the_trace():
    """The store caches one mutable Trace per id, and batch runs called
    store.get -- so an ad-hoc sync offset set in the GUI was silently baked
    into the exported files, and the whole-corpus run (a background thread)
    mutated and then evicted the very trace the viewport was drawing."""
    from fastapi.testclient import TestClient

    import trace_fixer.api as api
    from trace_fixer.store import TraceStore

    tmp = Path(tempfile.mkdtemp())
    api.store = TraceStore(traces_dir=REPO_ROOT / "data" / "traces")
    api.OUTPUT_DIR = tmp / "out"
    api.OUTPUT_DIR.mkdir(parents=True)
    client = TestClient(api.app)

    client.post("/api/traces/sample2/sync_offset", json={"offset_us": 500_000})
    session_trace = api.store.get("sample2")
    assert session_trace.sync_offset_us == 500_000

    r = client.post("/api/traces/sample2/batch_fix_predict")
    assert r.status_code == 200

    # the session's copy is exactly as the person left it...
    assert api.store.get("sample2") is session_trace
    assert session_trace.sync_offset_us == 500_000
    assert not any(
        o.synthetic for t in session_trace.annotation.vehicles.values() for o in t.observations
    ), "the batch must not have written its predictions into the open trace"


def test_two_batch_runs_of_the_same_trace_agree():
    """The point of starting from the files: a batch is reproducible."""
    from fastapi.testclient import TestClient

    import trace_fixer.api as api
    from trace_fixer.store import TraceStore

    tmp = Path(tempfile.mkdtemp())
    api.store = TraceStore(traces_dir=REPO_ROOT / "data" / "traces")
    api.OUTPUT_DIR = tmp / "out"
    api.OUTPUT_DIR.mkdir(parents=True)
    client = TestClient(api.app)

    first = client.post("/api/traces/sample2/batch_fix_predict").json()
    second = client.post("/api/traces/sample2/batch_fix_predict").json()
    for key in ("before_issue_count", "after_issue_count", "provenance", "predicted"):
        assert first[key] == second[key], key
