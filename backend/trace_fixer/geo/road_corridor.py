"""Shared helper: derive local left/right drivable-corridor bounds (ego-relative
y, at a given ego-relative x and time) from the annotated Road Edge / Guardrail
polylines. Used by both the validation off-road check and the fix engine's
off-road clamp so the two agree on what "on-road" means.

The geometry is reprojected, not taken as stored. A snapshot's
`points_rel` are in the ego frame *at that snapshot's own time*, and
border snapshots are sparse -- across a 25-trace corpus the median
observation's nearest snapshot is 1.6 s away, the 90th percentile 7.0 s,
the worst 20.6 s. Testing a vehicle's `y_rel` from time T against a
polyline captured at time S treats two different ego frames as one, and
the error that introduces is roughly `x_rel * sin(delta_yaw)`: with
`x_rel` reaching 40 m, five degrees of ego rotation is three and a half
metres of phantom lateral offset, which is most of a lane.

So each snapshot's *global* points (`points_m`, filled by geo.populate)
are reprojected into the ego frame of the observation being judged. A
road edge does not move, so the world-frame geometry is the thing that is
actually true, and the ego pose at the observation's own time is the only
frame it should be compared in.

Measured against the corpus, bucketed by how far the ego rotated between
the snapshot and the observation, this changes verdicts exactly where it
should and nowhere else:

    ego rotation     observations    verdicts changed
    0-1 deg                  2652                3.1%
    1-3 deg                  1417               17.8%
    3-10 deg                 1369               13.4%
    >10 deg                   763               12.6%

Where the frames coincide the two agree 97% of the time; the moment there
is real rotation, disagreement jumps five- to six-fold. (It stops climbing
past 3 deg because at large rotations the geometry often falls outside the
+/-40 m window in both variants, so both abstain and agree that way.)

Note what is deliberately *not* changed: only the nearest-in-time snapshot
per border line is consulted, exactly as before. Reprojection makes every
snapshot's geometry valid for the whole clip, so it is tempting to pool all
of them -- but that changes which edge counts as "innermost" and there is
no evidence it is an improvement, so it stays out.
"""
from __future__ import annotations

import math

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
    border_lines: dict[int, BorderLine],
    t_us: int,
    ego_x: float,
    ego_y: float,
    ego_yaw_rad: float,
    x_rel: float,
    x_tol: float = CORRIDOR_X_TOL_M,
) -> tuple[float | None, float | None]:
    """Returns (left_bound, right_bound) in ego-relative y meters -- the
    innermost (most restrictive) edge/guardrail on each side near x_rel.

    Per border line, the snapshot closest to `t_us` is chosen, and its
    global points are reprojected into the ego frame given by
    (ego_x, ego_y, ego_yaw_rad) -- which must be the ego pose at the
    observation being judged, not at the snapshot. See the module
    docstring. Either bound is None if no border geometry was found nearby.
    """
    left_bound: float | None = None
    right_bound: float | None = None
    cos_y, sin_y = math.cos(ego_yaw_rad), math.sin(ego_yaw_rad)
    for line in border_lines.values():
        if not line.snapshots:
            continue
        snap = min(line.snapshots, key=lambda s: abs(s.t_us - t_us))
        nearest: tuple[float, float] | None = None  # (|dx| from x_rel, y_rel)
        for gx, gy in snap.points_m:
            dx, dy = gx - ego_x, gy - ego_y
            px = cos_y * dx + sin_y * dy
            offset = abs(px - x_rel)
            if offset > x_tol:
                continue
            if nearest is None or offset < nearest[0]:
                nearest = (offset, -sin_y * dx + cos_y * dy)
        if nearest is None:
            continue
        py = nearest[1]
        if py > 0:
            left_bound = py if left_bound is None else min(left_bound, py)
        else:
            right_bound = py if right_bound is None else max(right_bound, py)
    return left_bound, right_bound


def corridor_excursion(
    border_lines: dict[int, BorderLine],
    obs: VehicleObs,
    ego_x: float,
    ego_y: float,
    ego_yaw_rad: float,
) -> tuple[float, str | None]:
    """How far outside the corridor this observation's box reaches, and on
    which side ("left"/"right"), judged from the ego pose at the
    observation's own time.

    Returns (0.0, None) when the box is inside the corridor (or within
    MIN_EXCURSION_M of the edge), or when there is no border geometry near it
    to judge against. This *is* the off-road test, factored out so the fix
    engine and the departure classifier can ask "how far out?" and not just
    "out or not?".
    """
    left, right = corridor_bounds(border_lines, obs.t_us, ego_x, ego_y, ego_yaw_rad, obs.x_rel)
    half_w = obs.width / 2.0
    out, side = 0.0, None
    if left is not None and obs.y_rel - half_w > left - OFFROAD_MARGIN_M:
        out, side = (obs.y_rel - half_w) - (left - OFFROAD_MARGIN_M), "left"
    elif right is not None and obs.y_rel + half_w < right + OFFROAD_MARGIN_M:
        out, side = (right + OFFROAD_MARGIN_M) - (obs.y_rel + half_w), "right"
    return (out, side) if out >= MIN_EXCURSION_M else (0.0, None)
