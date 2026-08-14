"""Shared helper: derive local left/right drivable-corridor bounds (ego-relative
y, at a given ego-relative x and time) from the annotated Road Edge / Guardrail
polylines. Used by both the validation off-road check and the fix engine's
off-road clamp so the two agree on what "on-road" means.
"""
from __future__ import annotations

from trace_fixer.models import BorderLine

CORRIDOR_X_TOL_M = 40.0


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
