"""Coordinate transforms: lat/lon -> local ENU-ish meters, ego-pose interpolation,
and ego-relative annotation coordinates -> global.

Conventions used throughout this codebase (derived empirically from the sample
trace by cross-checking INS_ANGLE_TRUE_HEADING / INS_Vel_Frame_X,Y against GPS
displacement -- see docs/coordinate_conventions.md):

  - Local ground plane: x = East (meters), y = North (meters), anchored at the
    first ADMA sample of the trace (equirectangular projection, valid for the
    short distances of a single log).
  - yaw (`heading_rad`): standard right-handed math angle, counterclockwise
    from +x (East). Related to the raw ADMA channel by
        yaw_deg = (90 + INS_ANGLE_TRUE_HEADING) % 360
    i.e. INS_ANGLE_TRUE_HEADING is a counterclockwise-from-north angle, the
    opposite rotational sense of a normal compass bearing.
  - INS_Vel_Frame_X / _Y are North / East velocity components respectively
    (a local-level nav frame), *not* vehicle-frame forward/lateral velocity.
  - Annotation bounding boxes (xp forward, yp left, zrot ego-relative heading)
    are in the vehicle body frame, so global = ego_xy + R(yaw) @ (xp, yp) with
    the standard CCW rotation matrix R(yaw) = [[cos, -sin], [sin, cos]].
"""
from __future__ import annotations

import bisect
import math

from trace_fixer.models import EgoPose, EgoTrace

EARTH_RADIUS_M = 6378137.0


def latlon_to_local(lat_deg: float, lon_deg: float, lat0_deg: float, lon0_deg: float) -> tuple[float, float]:
    """Equirectangular projection relative to (lat0, lon0). Returns (x=East, y=North) meters."""
    lat0_rad = math.radians(lat0_deg)
    dlat = math.radians(lat_deg - lat0_deg)
    dlon = math.radians(lon_deg - lon0_deg)
    north = dlat * EARTH_RADIUS_M
    east = dlon * EARTH_RADIUS_M * math.cos(lat0_rad)
    return east, north


def heading_to_yaw_rad(heading_deg: float) -> float:
    return math.radians((90.0 + heading_deg) % 360.0)


def project_ego_trace(ego: EgoTrace) -> None:
    """Fill in x_m/y_m (local ENU) on every pose, in place, anchored at the first pose."""
    if not ego.poses:
        return
    lat0, lon0 = ego.poses[0].lat_deg, ego.poses[0].lon_deg
    for pose in ego.poses:
        pose.x_m, pose.y_m = latlon_to_local(pose.lat_deg, pose.lon_deg, lat0, lon0)


def _wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


class EgoInterpolator:
    """Interpolates ego pose (position + yaw) at arbitrary times along an EgoTrace.

    Times outside the trace range are clamped to the nearest endpoint pose
    (no extrapolation here -- pre/post-FOV prediction of *other vehicles* is
    a separate, explicit feature in trace_fixer.prediction).
    """

    def __init__(self, ego: EgoTrace):
        if not ego.poses:
            raise ValueError("EgoTrace has no poses")
        self.poses = ego.poses
        self._times = [p.t_us for p in self.poses]

    def at(self, t_us: int) -> tuple[float, float, float, float]:
        """Returns (x_m, y_m, yaw_rad, speed_mps) at time t_us."""
        times = self._times
        if t_us <= times[0]:
            p = self.poses[0]
            return p.x_m, p.y_m, heading_to_yaw_rad(p.heading_deg), math.hypot(p.vx_mps, p.vy_mps)
        if t_us >= times[-1]:
            p = self.poses[-1]
            return p.x_m, p.y_m, heading_to_yaw_rad(p.heading_deg), math.hypot(p.vx_mps, p.vy_mps)

        i = bisect.bisect_right(times, t_us) - 1
        i = max(0, min(i, len(self.poses) - 2))
        p0, p1 = self.poses[i], self.poses[i + 1]
        span = p1.t_us - p0.t_us
        frac = 0.0 if span <= 0 else (t_us - p0.t_us) / span

        x = p0.x_m + frac * (p1.x_m - p0.x_m)
        y = p0.y_m + frac * (p1.y_m - p0.y_m)
        h0, h1 = heading_to_yaw_rad(p0.heading_deg), heading_to_yaw_rad(p1.heading_deg)
        dh = math.atan2(math.sin(h1 - h0), math.cos(h1 - h0))
        yaw = h0 + frac * dh
        speed = math.hypot(p0.vx_mps, p0.vy_mps) + frac * (
            math.hypot(p1.vx_mps, p1.vy_mps) - math.hypot(p0.vx_mps, p0.vy_mps)
        )
        return x, y, yaw, speed


def ego_relative_to_global(
    x_rel: float, y_rel: float, ego_x: float, ego_y: float, ego_yaw_rad: float
) -> tuple[float, float]:
    cos_y, sin_y = math.cos(ego_yaw_rad), math.sin(ego_yaw_rad)
    gx = ego_x + cos_y * x_rel - sin_y * y_rel
    gy = ego_y + sin_y * x_rel + cos_y * y_rel
    return gx, gy


def global_heading_rad(ego_yaw_rad: float, zrot_rad: float) -> float:
    return ego_yaw_rad + zrot_rad


def global_to_ego_relative(
    gx: float, gy: float, ego_x: float, ego_y: float, ego_yaw_rad: float
) -> tuple[float, float]:
    """Inverse of ego_relative_to_global: world (gx, gy) -> ego-frame (x_rel, y_rel)."""
    dx, dy = gx - ego_x, gy - ego_y
    cos_y, sin_y = math.cos(ego_yaw_rad), math.sin(ego_yaw_rad)
    x_rel = cos_y * dx + sin_y * dy
    y_rel = -sin_y * dx + cos_y * dy
    return x_rel, y_rel


def global_heading_to_zrot(global_heading_rad_: float, ego_yaw_rad: float) -> float:
    return math.atan2(
        math.sin(global_heading_rad_ - ego_yaw_rad), math.cos(global_heading_rad_ - ego_yaw_rad)
    )
