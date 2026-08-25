"""Generates alternative ("ODD variant") scenarios from a trace by
perturbing one selected surrounding vehicle's already-recorded relative
trajectory at a time -- the ego's own path and the road never change,
only what nearby traffic does. The goal is a small menu of harder/
different scenarios to export from a single real recording, not a full
traffic simulator, so every perturbation reuses the vehicle's own
recorded motion (rescaled, time-warped, or stepped) rather than
synthesizing new kinematics from scratch -- results stay close to
something that was actually driven.

Two generation modes:

  - Presets (`generate_preset_variants`): five fixed, named, explainable
    variants, each targeting a specific real behavior already detected by
    `analysis.py` (a cut-in, a lead vehicle, an overtake) and making it
    measurably harder or different. A preset that finds no matching
    vehicle in this particular trace is simply skipped -- not every
    preset applies to every recording.
  - Randomized (`generate_randomized_variants`): a random vehicle +
    perturbation kind + magnitude (within safe bounds) per slot, seeded
    for reproducibility. Also used to backfill preset slots that found no
    matching vehicle, so a `count`-sized request still returns `count`
    variants where the trace has enough vehicles to support it.

Each variant is a full, independent, deep-copied `Trace` with
`populate_global_coords` and `run_validation` re-run on it, so its scene
JSON, issue list, and every existing export path (OpenDRIVE, OpenSCENARIO,
ADP yaml, fixed-trace, report) all work on it completely unchanged --
see api.py's `/variants/*` endpoints, which just swap in the variant's
`Trace` object wherever a normal trace's would go.

Known simplification: only `x_rel`/`y_rel` (the vehicle's ego-relative
position) are perturbed. `zrot` (the vehicle's own recorded heading) is
left as originally observed, so a sharply time-compressed lateral move
won't show a correspondingly steeper heading -- acceptable for generating
a harder/different *positional* scenario, but not a substitute for a real
vehicle dynamics model.
"""
from __future__ import annotations

import copy
import random
import uuid
from dataclasses import dataclass
from typing import Callable

from trace_fixer.analysis import LANE_HALF_WIDTH_M, detect_cut_in_events, detect_overtake_events
from trace_fixer.geo.populate import populate_global_coords
from trace_fixer.models import Trace, VehicleObs
from trace_fixer.validation.checks import run_validation

MIN_OBS_FOR_VARIANT = 5


@dataclass
class Variant:
    variant_id: str
    kind: str
    name: str
    description: str
    vehicle_id: int
    source: str  # "preset" | "randomized"
    trace: Trace
    issue_count: int
    base_issue_count: int


def _median(values: list[float]) -> float:
    s = sorted(values)
    n = len(s)
    if n == 0:
        return 0.0
    mid = n // 2
    return s[mid] if n % 2 else (s[mid - 1] + s[mid]) / 2.0


def _interp(xs: list[int], ys: list[float], x: float) -> float:
    n = len(xs)
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    lo, hi = 0, n - 1
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        if xs[mid] <= x:
            lo = mid
        else:
            hi = mid
    x0, x1 = xs[lo], xs[hi]
    if x1 == x0:
        return ys[lo]
    frac = (x - x0) / (x1 - x0)
    return ys[lo] + frac * (ys[hi] - ys[lo])


# ---------- perturbation primitives (mutate a sorted VehicleObs list in place) ----------

def _time_warp_lateral(obs: list[VehicleObs], compress: float, tighten: float) -> None:
    """compress<1 front-loads the lateral (y_rel) motion in time (an
    earlier, tighter cut-in/merge); compress>1 stretches it out (a later,
    looser one). `tighten` scales the resulting y_rel afterward.
    """
    t0 = obs[0].t_us
    span = max(1, obs[-1].t_us - t0)
    orig_t = [o.t_us for o in obs]
    orig_y = [o.y_rel for o in obs]
    for o in obs:
        frac = (o.t_us - t0) / span
        warped_frac = min(1.0, frac / compress)
        target_t = t0 + warped_frac * span
        o.y_rel = _interp(orig_t, orig_y, target_t) * tighten


def _scale_gap(obs: list[VehicleObs], factor: float, floor_m: float = 1.0) -> None:
    for o in obs:
        if o.x_rel > 0:
            o.x_rel = max(floor_m, o.x_rel * factor)


def _windowed_gap_step(obs: list[VehicleObs], step_factor: float, window_frac: tuple[float, float], floor_m: float = 1.0) -> None:
    """Gap stays as-recorded before the window, ramps down linearly across
    it, and stays at `step_factor` after -- a lead vehicle that brakes hard
    partway through and doesn't fully recover the gap.
    """
    t0 = obs[0].t_us
    span = max(1, obs[-1].t_us - t0)
    w0, w1 = window_frac
    for o in obs:
        if o.x_rel <= 0:
            continue
        frac = (o.t_us - t0) / span
        if frac < w0:
            scale = 1.0
        elif frac < w1:
            local = (frac - w0) / max(1e-9, w1 - w0)
            scale = 1.0 - local * (1.0 - step_factor)
        else:
            scale = step_factor
        o.x_rel = max(floor_m, o.x_rel * scale)


def _scale_lateral(obs: list[VehicleObs], factor: float) -> None:
    for o in obs:
        o.y_rel *= factor


# ---------- vehicle selection (reuses analysis.py's own event detectors) ----------

def _pick_cutin_vehicle(trace: Trace) -> int | None:
    events = detect_cut_in_events(trace)
    if not events:
        return None
    return min(events, key=lambda e: e.range_m).vehicle_id


def _pick_lead_vehicle(trace: Trace) -> int | None:
    """The vehicle most consistently ahead of the ego in its own lane
    (median x_rel > 0, median |y_rel| within a lane width), closest first.
    """
    candidates = []
    for track in trace.annotation.vehicles.values():
        if len(track.observations) < MIN_OBS_FOR_VARIANT:
            continue
        med_x = _median([o.x_rel for o in track.observations])
        med_y = _median([o.y_rel for o in track.observations])
        if med_x > 0 and abs(med_y) <= LANE_HALF_WIDTH_M:
            candidates.append((med_x, track.obj_id))
    if not candidates:
        return None
    candidates.sort()
    return candidates[0][1]


def _pick_overtake_vehicle(trace: Trace) -> int | None:
    events = detect_overtake_events(trace)
    if not events:
        return None
    return max(events, key=lambda e: abs(e.lateral_offset_m)).vehicle_id


@dataclass(frozen=True)
class _PresetSpec:
    kind: str
    name: str
    description: str
    pick: Callable[[Trace], "int | None"]
    apply: Callable[[list[VehicleObs]], None]


PRESET_SPECS: list[_PresetSpec] = [
    _PresetSpec(
        "harder_cutin", "Harder cut-in",
        "The nearest cut-in vehicle merges into the ego's lane sooner and settles closer to lane center.",
        _pick_cutin_vehicle, lambda obs: _time_warp_lateral(obs, compress=0.7, tighten=0.8),
    ),
    _PresetSpec(
        "delayed_merge", "Delayed merge",
        "The same cut-in vehicle merges later and stays closer to the lane boundary longer.",
        _pick_cutin_vehicle, lambda obs: _time_warp_lateral(obs, compress=1.4, tighten=1.2),
    ),
    _PresetSpec(
        "closer_following", "Closer following",
        "The lead vehicle in the ego's lane keeps a tighter gap for its whole observed window.",
        _pick_lead_vehicle, lambda obs: _scale_gap(obs, factor=0.65),
    ),
    _PresetSpec(
        "hard_brake_lead", "Hard brake ahead",
        "The lead vehicle brakes hard partway through, closing the gap sharply and not fully recovering it.",
        _pick_lead_vehicle, lambda obs: _windowed_gap_step(obs, step_factor=0.5, window_frac=(0.3, 0.55)),
    ),
    _PresetSpec(
        "tighter_clearance", "Tighter passing clearance",
        "The passing/overtaking vehicle stays laterally closer to the ego while going around.",
        _pick_overtake_vehicle, lambda obs: _scale_lateral(obs, factor=0.6),
    ),
]

_RandomApplier = Callable[[list[VehicleObs], random.Random], None]
_RANDOM_KINDS: list[tuple[str, _RandomApplier]] = [
    ("gap_scale", lambda obs, rng: _scale_gap(obs, factor=rng.uniform(0.5, 0.85))),
    ("lateral_scale", lambda obs, rng: _scale_lateral(obs, factor=rng.uniform(0.5, 0.85))),
    ("time_compress", lambda obs, rng: _time_warp_lateral(obs, compress=rng.uniform(0.55, 0.85), tighten=rng.uniform(0.7, 0.95))),
    ("time_expand", lambda obs, rng: _time_warp_lateral(obs, compress=rng.uniform(1.2, 1.6), tighten=rng.uniform(1.05, 1.3))),
    ("windowed_brake", lambda obs, rng: _windowed_gap_step(
        obs, step_factor=rng.uniform(0.4, 0.7), window_frac=_random_window(rng)
    )),
]


def _random_window(rng: random.Random) -> tuple[float, float]:
    start = rng.uniform(0.15, 0.55)
    width = rng.uniform(0.15, 0.35)
    return (start, min(0.95, start + width))


def _eligible_vehicle_ids(trace: Trace) -> list[int]:
    return [t.obj_id for t in trace.annotation.vehicles.values() if len(t.observations) >= MIN_OBS_FOR_VARIANT]


def _build_variant(
    trace: Trace, vehicle_id: int, apply_fn: Callable[[list[VehicleObs]], None],
    variant_id: str, kind: str, name: str, description: str, source: str,
) -> Variant:
    new_trace = copy.deepcopy(trace)
    track = new_trace.annotation.vehicles[vehicle_id]
    obs_sorted = sorted(track.observations, key=lambda o: o.t_us)
    apply_fn(obs_sorted)
    # Marks the perturbed observations as "fixed" so export.annotation_writer's
    # surgical XML patching (gated on this flag) actually writes the new
    # x_rel/y_rel out -- otherwise a variant's Fixed Trace export would
    # silently reproduce the original, unperturbed annotation.
    for o in obs_sorted:
        o.fixed = True
    populate_global_coords(new_trace)
    run_validation(new_trace)
    new_trace.trace_id = f"{trace.trace_id}__{variant_id}"
    return Variant(
        variant_id=variant_id, kind=kind, name=name, description=description, vehicle_id=vehicle_id,
        source=source, trace=new_trace, issue_count=len(new_trace.issues), base_issue_count=len(trace.issues),
    )


def generate_preset_variants(trace: Trace, count: int = 5) -> list[Variant]:
    variants: list[Variant] = []
    for spec in PRESET_SPECS[:count]:
        vehicle_id = spec.pick(trace)
        if vehicle_id is None:
            continue
        variant_id = f"{spec.kind}-{uuid.uuid4().hex[:6]}"
        variants.append(_build_variant(
            trace, vehicle_id, spec.apply, variant_id, spec.kind, spec.name, spec.description, "preset",
        ))
    if len(variants) < count:
        variants.extend(generate_randomized_variants(trace, count - len(variants), seed=hash(trace.trace_id) & 0xFFFFFFFF))
    return variants


def generate_randomized_variants(trace: Trace, count: int, seed: int | None = None) -> list[Variant]:
    vehicle_ids = _eligible_vehicle_ids(trace)
    if not vehicle_ids or count <= 0:
        return []
    rng = random.Random(seed)
    variants = []
    for _ in range(count):
        vehicle_id = rng.choice(vehicle_ids)
        kind_name, apply_fn = rng.choice(_RANDOM_KINDS)
        variant_id = f"random-{kind_name}-{uuid.uuid4().hex[:6]}"
        label = kind_name.replace("_", " ")
        variants.append(_build_variant(
            trace, vehicle_id, lambda obs, f=apply_fn, r=rng: f(obs, r),
            variant_id, f"random_{kind_name}", f"Randomized: {label}",
            f"Randomized {label} perturbation applied to vehicle {vehicle_id}'s recorded trajectory.",
            "randomized",
        ))
    return variants
