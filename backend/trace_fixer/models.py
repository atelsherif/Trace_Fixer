"""Shared data model for a parsed trace pair (ADMA ego trace + annotation file)."""
from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class EgoPose:
    t_us: int  # ADTF chunk time, microseconds, ADMA clock
    lat_deg: float
    lon_deg: float
    heading_deg: float  # 0..360, clockwise from north (ADMA convention)
    vx_mps: float  # forward velocity, vehicle frame
    vy_mps: float  # lateral velocity, vehicle frame
    vz_mps: float
    x_m: float = 0.0  # local ENU, filled in by geo.transform
    y_m: float = 0.0


@dataclass
class EgoTrace:
    poses: list[EgoPose] = field(default_factory=list)

    @property
    def t0_us(self) -> int:
        return self.poses[0].t_us

    @property
    def t1_us(self) -> int:
        return self.poses[-1].t_us


@dataclass
class VehicleObs:
    t_us: int  # annotation clock (chunktime), microseconds
    frame: int
    obj_movement: str
    obj_lane: str
    obj_confidence: str
    x_rel: float  # ego frame, forward +x
    y_rel: float  # ego frame, left +y
    z_rel: float
    length: float
    width: float
    height: float
    zrot: float  # heading relative to ego, radians
    interpolation_state: str | None = None
    # filled in by geo.transform:
    x_m: float = 0.0
    y_m: float = 0.0
    heading_deg: float = 0.0
    synthetic: bool = False  # True if produced by backward prediction, not original annotation
    fixed: bool = False  # True if position/heading were adjusted by the fix engine


@dataclass
class VehicleTrack:
    obj_id: int
    obj_type: str
    reflecting_parts: str | None
    observations: list[VehicleObs] = field(default_factory=list)


@dataclass
class LaneSnapshot:
    t_us: int
    frame: int
    points_rel: list[tuple[float, float]]  # ego frame
    points_m: list[tuple[float, float]] = field(default_factory=list)  # global, filled by transform


@dataclass
class LaneMarking:
    obj_id: int
    lane_type: str
    width: float
    group: int
    snapshots: list[LaneSnapshot] = field(default_factory=list)


@dataclass
class BorderLine:
    obj_id: int
    obj_type: str
    snapshots: list[LaneSnapshot] = field(default_factory=list)


@dataclass
class StaticObs:
    t_us: int
    frame: int
    x_rel: float
    y_rel: float
    z_rel: float
    length: float
    width: float
    height: float
    zrot: float
    x_m: float = 0.0
    y_m: float = 0.0


@dataclass
class StaticObject:
    obj_id: int
    obj_type: str
    observations: list[StaticObs] = field(default_factory=list)


@dataclass
class FrameMeta:
    t_us: int
    frame: int
    road_type: str | None
    num_lanes: int | None
    ego_lane: int | None
    weather: str | None
    light_conditions: str | None


@dataclass
class Annotation:
    country_code: str | None
    frame_meta: list[FrameMeta] = field(default_factory=list)
    vehicles: dict[int, VehicleTrack] = field(default_factory=dict)
    lane_markings: dict[int, LaneMarking] = field(default_factory=dict)
    border_lines: dict[int, BorderLine] = field(default_factory=dict)
    static_objects: dict[int, StaticObject] = field(default_factory=dict)


@dataclass
class Issue:
    issue_id: str
    category: str  # "kinematic" | "collision" | "off_road" | "sync"
    severity: str  # "low" | "medium" | "high"
    vehicle_id: int | None
    t_start_us: int
    t_end_us: int
    description: str
    fixable: bool = True
    fixed: bool = False


@dataclass
class Trace:
    trace_id: str
    ego: EgoTrace
    annotation: Annotation
    sync_offset_us: int = 0
    issues: list[Issue] = field(default_factory=list)
