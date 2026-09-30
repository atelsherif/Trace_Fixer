"""Shared helper: derive local left/right drivable-corridor bounds (ego-relative
y, at a given ego-relative x and time) from the annotated Road Edge / Guardrail
polylines. Used by both the validation off-road check and the fix engine's
off-road clamp so the two agree on what "on-road" means.

KNOWN LIMITATION -- the corridor is compared across ego frames from
different times, and off-road verdicts should be read with that in mind.

A snapshot's `points_rel` are in the ego frame *at that snapshot's own
time*, but they are tested against a vehicle's `x_rel`/`y_rel` from the
observation's time, and border snapshots are sparse: the bundled samples
carry 10 and 11 distinct snapshot times across a 60 s clip, so the median
observation is judged against geometry 1.7-3.5 s away and the worst against
geometry 12.6 s away. At 25 m/s that is several hundred metres of ego
travel, plus whatever the ego rotated through. Where the road is straight
and the ego is not turning the two frames nearly coincide and the verdict
is sound; on a curve it is not.

The principled fix is to work from the global points (`points_m`, filled by
geo.populate) and reproject them into the ego frame of the observation being
judged -- a road edge does not move, so once in world coordinates every
snapshot's geometry stays valid for the whole clip. That was tried and
backed out: it changes which edge is "innermost" often enough to flip
verdicts (on sample2 it moved one vehicle's departure from the left side to
the right), and with two bundled traces there is no way to tell which
answer is the correct one. It needs a corpus with known-good off-road
ground truth to validate against before it goes in. Until then the sparse
comparison stays, documented, rather than being replaced by something
equally unverified.
"""
from __future__ import annotations

from trace_fixer.models import BorderLine, VehicleObs

CORRIDOR_X_TOL_M = 40.0

# How far past an edge/guardrail a box may reach before it counts as outside
# the corridor. Lives here rather than in validation.checks so the departure
# classifier can use it without importing the validator.
OFFROAD_MARGIN_M = 0.3

# Below this, an "excursion" is a rounding artifact of where the margin falls
# rather than something a reviewer could act on -- annotated box edges aren't
# accurate to the centimeter. Treated as inside the corridor.
MIN_EXCURSION_M = 0.05


def corridor_bounds(
    border_lines: dict[int, BorderLine], t_us: int, x_rel: float, x_tol: float = CORRIDOR_X_TOL_M
) -> tuple[float | None, float | None]:
    """Returns (left_bound, right_bound) in ego-relative y meters -- the
    innermost (most restrictive) edge/guardrail on each side near x_rel, at
    the annotation keyframe closest to t_us. Either bound is None if no
    border geometry was found nearby.
    """
    left_candidates: list[float] = []
    right_candidates: list[float] = []
    for line in border_lines.values():
        if not line.snapshots:
            continue
        snap = min(line.snapshots, key=lambda s: abs(s.t_us - t_us))
        nearby = [p for p in snap.points_rel if abs(p[0] - x_rel) <= x_tol]
        if not nearby:
            continue
        px, py = min(nearby, key=lambda p: abs(p[0] - x_rel))
        if py > 0:
            left_candidates.append(py)
        else:
            right_candidates.append(py)
    left_bound = min(left_candidates) if left_candidates else None
    right_bound = max(right_candidates) if right_candidates else None
    return left_bound, right_bound


def corridor_excursion(
    border_lines: dict[int, BorderLine], obs: VehicleObs
) -> tuple[float, str | None]:
    """How far outside the corridor this observation's box reaches, and on
    which side ("left"/"right").

    Returns (0.0, None) when the box is inside the corridor (or within
    MIN_EXCURSION_M of the edge), or when there is no border geometry near it
    to judge against. This *is* the off-road test, factored out so the fix
    engine and the departure classifier can ask "how far out?" and not just
    "out or not?".
    """
    left, right = corridor_bounds(border_lines, obs.t_us, obs.x_rel)
    half_w = obs.width / 2.0
    out, side = 0.0, None
    if left is not None and obs.y_rel - half_w > left - OFFROAD_MARGIN_M:
        out, side = (obs.y_rel - half_w) - (left - OFFROAD_MARGIN_M), "left"
    elif right is not None and obs.y_rel + half_w < right + OFFROAD_MARGIN_M:
        out, side = (right + OFFROAD_MARGIN_M) - (obs.y_rel + half_w), "right"
    return (out, side) if out >= MIN_EXCURSION_M else (0.0, None)
