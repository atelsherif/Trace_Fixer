"""Tests for trace_fixer.analysis: object counts, ego braking detection,
and overtake detection."""
import copy
import math
from pathlib import Path

import pytest

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


def test_count_objects_by_type(sample1):
    from trace_fixer.analysis import count_objects_by_type

    assert count_objects_by_type(sample1) == {"Truck": 4, "Car": 1}


def test_neither_sample_has_real_braking_events(sample1, sample2):
    """Both bundled samples are steady-state highway driving (confirmed by
    inspecting their speed profiles): this is a negative-case check that the
    detector doesn't fire on ordinary cruising."""
    from trace_fixer.analysis import detect_braking_events

    assert detect_braking_events(sample1) == []
    assert detect_braking_events(sample2) == []


def test_detects_injected_hard_braking_event(sample1):
    """Synthetic positive case: since neither sample has a real braking
    event, inject one into a copy of the ego trace and confirm it's found
    with the right severity and a plausible time window.
    """
    from trace_fixer.analysis import HARD_DECEL_MPS2, detect_braking_events

    trace = copy.deepcopy(sample1)
    poses = trace.ego.poses
    # Ramp speed down hard over ~1s starting a third of the way through,
    # then hold -- a clean, unambiguous hard-braking profile.
    start_idx = len(poses) // 3
    start_speed = 33.0
    decel = 8.0  # m/s^2, clearly above HARD_DECEL_MPS2
    for i in range(start_idx, len(poses)):
        dt_s = (poses[i].t_us - poses[start_idx].t_us) / 1e6
        speed = max(5.0, start_speed - decel * dt_s)
        heading_rad = math.radians((90 + poses[i].heading_deg) % 360)
        poses[i].vx_mps = speed * math.cos(heading_rad)
        poses[i].vy_mps = speed * math.sin(heading_rad)

    events = detect_braking_events(trace)
    assert len(events) >= 1
    assert any(e.severity == "hard" for e in events)
    assert any(abs(e.peak_decel_mps2) >= HARD_DECEL_MPS2 for e in events)


def test_sample1_overtake_events(sample1):
    from trace_fixer.analysis import detect_overtake_events

    events = detect_overtake_events(sample1)
    assert len(events) == 3
    assert {e.vehicle_id for e in events} == {1, 4, 5}
    assert all(e.direction == "ego_overtakes_vehicle" for e in events)
    # chronological
    assert [e.t_us for e in events] == sorted(e.t_us for e in events)


def test_overtake_direction_labels_are_meaningful(sample1):
    from trace_fixer.analysis import detect_overtake_events

    for ev in detect_overtake_events(sample1):
        assert ev.direction in ("vehicle_overtakes_ego", "ego_overtakes_vehicle")
        assert abs(ev.lateral_offset_m) <= 8.0


def test_build_trace_summary_combines_all_three(sample1):
    from trace_fixer.analysis import build_trace_summary

    summary = build_trace_summary(sample1)
    assert summary.object_counts == {"Truck": 4, "Car": 1}
    assert summary.braking_events == []
    assert len(summary.overtake_events) == 3
