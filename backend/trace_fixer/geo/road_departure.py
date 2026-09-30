"""Tells a genuine road departure apart from annotation noise.

The annotation's Road Edge / Guardrail polylines describe *the ego's* road and
nothing else. So a vehicle that takes an exit ramp, or turns right into a side
street, is outside that corridor by design -- and the off-road clamp used to
drag it straight back into the ego's lane, then report the result as a fix. The
repaired trace showed a vehicle dutifully following the ego down a road it had
already left, which is worse than the flag it replaced.

The discriminator here is deliberately geometric and explainable, in the same
spirit as validation/checks.py: a vehicle that is *leaving* has turned off the
road's axis and/or is far enough out that no plausible annotation error would
put it there, whereas annotation noise puts a box slightly outside the edge
while it stays road-parallel and comes back.

Departures are reported for review (never auto-corrected); noise is what the
clamp is allowed to touch.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from trace_fixer.geo.road_corridor import corridor_excursion
from trace_fixer.models import BorderLine, VehicleTrack

# Beyond this far outside the edge, a lateral annotation error is no longer a
# plausible explanation -- lane widths are ~3.5 m, so 2 m past a guardrail is
# already most of a lane off the road.
DEPARTURE_EXCURSION_M = 2.0
# Past this, nothing else needs to corroborate it.
DEPARTURE_FAR_M = 4.0
# How far off the road's axis the vehicle must have turned. Deliberately
# small: a real turn-off reaches tens of degrees, while a road-parallel
# vehicle sitting outside the edge because of a lateral bias stays near 0.
DEPARTURE_TURN_DEG = 10.0
# Window used to establish what course the vehicle held *before* the
# excursion, so the turn is measured against its own prior heading rather
# than an assumed road direction.
COURSE_WINDOW_S = 1.0
# A tangent over a shorter baseline than this is dominated by position noise.
MIN_COURSE_BASELINE_M = 1.5

NOISE = "noise"
DEPARTURE = "departure"


@dataclass(frozen=True)
class OffroadRun:
    """One unbroken stretch of a track spent outside the annotated corridor.

    The run, not the individual observation, is the unit that gets classified
    and reported: a single turn-off is one thing that happened, and splitting
    it into an issue per frame buries the trace's real problems under dozens
    of rows saying the same thing.
    """

    t_start_us: int
    t_end_us: int
    side: str  # "left" | "right" -- the side of the ego's road it left by
    label: str  # NOISE | DEPARTURE
    peak_m: float  # furthest the box reached beyond the edge
    turn_deg: float  # how far it turned off its earlier course

    @property
    def is_departure(self) -> bool:
        return self.label == DEPARTURE


def _wrap180(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0


def _axis_angle_deg(zrot_rad: float) -> float:
    """Angle between the vehicle's heading and the ego's, folded into
    [0, 90]. Folding matters so an oncoming vehicle (zrot ~ 180 deg) reads as
    road-parallel rather than maximally turned away."""
    a = abs(_wrap180(math.degrees(zrot_rad)))
    return min(a, 180.0 - a)


def _course_deg(track: VehicleTrack, i0: int, i1: int) -> float | None:
    """Bearing of the straight line from observation i0 to i1, or None when
    the two are too close together for the direction to mean anything."""
    a, b = track.observations[i0], track.observations[i1]
    dx, dy = b.x_m - a.x_m, b.y_m - a.y_m
    if math.hypot(dx, dy) < MIN_COURSE_BASELINE_M:
        return None
    return math.degrees(math.atan2(dy, dx))


def _turn_deg(track: VehicleTrack, start: int, end: int) -> float:
    """How far the vehicle turned away from the course it was holding before
    this run. Measured from multi-sample path tangents, which survive the
    box-angle jitter that makes per-observation headings unreliable; falls
    back to the annotated ego-relative heading when there is no usable
    stretch of track before the run (it began at the first observation, or
    the vehicle was barely moving).
    """
    obs = track.observations
    window_us = int(COURSE_WINDOW_S * 1e6)
    baseline_start = start
    while baseline_start > 0 and obs[start].t_us - obs[baseline_start].t_us < window_us:
        baseline_start -= 1

    before = _course_deg(track, baseline_start, start) if baseline_start < start else None
    during = _course_deg(track, start, end) if end > start else None
    if before is not None and during is not None:
        return abs(_wrap180(during - before))

    # No usable baseline: how far off the ego's axis it got is the only
    # attitude evidence left.
    return max(_axis_angle_deg(o.zrot) for o in obs[start : end + 1])


def offroad_runs(track: VehicleTrack, border_lines: dict[int, BorderLine]) -> list[OffroadRun]:
    """Every stretch of `track` spent outside the annotated corridor, each
    classified as a whole so a single turn-off can't come back as a mix of
    both labels."""
    obs = track.observations
    measured = [corridor_excursion(border_lines, o) for o in obs]
    excursions = [m[0] for m in measured]

    runs: list[OffroadRun] = []
    i = 0
    while i < len(obs):
        if excursions[i] <= 0.0:
            i += 1
            continue
        start = i
        while i + 1 < len(obs) and excursions[i + 1] > 0.0:
            i += 1
        end = i

        peak_idx = max(range(start, end + 1), key=lambda j: excursions[j])
        peak = excursions[peak_idx]
        turn = _turn_deg(track, start, end)
        ran_to_end_of_track = end == len(obs) - 1
        is_departure = peak >= DEPARTURE_FAR_M or (
            peak >= DEPARTURE_EXCURSION_M and (ran_to_end_of_track or turn >= DEPARTURE_TURN_DEG)
        )
        runs.append(
            OffroadRun(
                t_start_us=obs[start].t_us,
                t_end_us=obs[end].t_us,
                side=measured[peak_idx][1] or "right",
                label=DEPARTURE if is_departure else NOISE,
                peak_m=peak,
                turn_deg=turn,
            )
        )
        i += 1

    return runs


def classify_offroad(track: VehicleTrack, border_lines: dict[int, BorderLine]) -> dict[int, str]:
    """offroad_runs() flattened to {observation t_us: NOISE | DEPARTURE}, for
    callers that walk observations rather than runs (the fix engine's
    corridor clamp). Observations inside the corridor are absent."""
    labels: dict[int, str] = {}
    for run in offroad_runs(track, border_lines):
        for o in track.observations:
            if run.t_start_us <= o.t_us <= run.t_end_us:
                labels[o.t_us] = run.label
    return labels
