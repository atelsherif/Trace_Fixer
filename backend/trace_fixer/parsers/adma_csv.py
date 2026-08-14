"""Parser for ADMA reference-system CSV exports.

Column scaling follows the ADMA 3.0 "Data Packets" manual (GeneSys Elektronik,
Doc Revision 1.9):
  - INS_Pos_Abs_Latitude / Longitude: LSB 1e-7 deg, signed long, range +-90/+-180
  - INS_Vel_Frame_X/Y/Z:              LSB 0.005 m/s, signed int, range +-160
  - INS_ANGLE_TRUE_HEADING:           LSB 0.01 deg, range 0..359.99 (same LSB
    convention as the other ADMA heading/course channels, e.g. Tilt_Yaw /
    GPS_Course_Over_Ground)
  - ADTF_CHUNK_TIME: microseconds, ADMA/ADTF recording clock
"""
from __future__ import annotations

import csv
from pathlib import Path

from trace_fixer.models import EgoPose, EgoTrace

LAT_LON_LSB = 1e-7
HEADING_LSB = 0.01
VEL_LSB = 0.005

REQUIRED_COLUMNS = (
    "ADTF_CHUNK_TIME",
    "INS_ANGLE_TRUE_HEADING",
    "INS_Pos_Abs_Latitude",
    "INS_Pos_Abs_Longitude",
    "INS_Vel_Frame_X",
    "INS_Vel_Frame_Y",
    "INS_Vel_Frame_Z",
)


def parse_adma_csv(path: str | Path) -> EgoTrace:
    path = Path(path)
    poses: list[EgoPose] = []
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        missing = [c for c in REQUIRED_COLUMNS if c not in (reader.fieldnames or [])]
        if missing:
            raise ValueError(f"ADMA CSV missing required columns: {missing}")
        for row in reader:
            poses.append(
                EgoPose(
                    t_us=int(row["ADTF_CHUNK_TIME"]),
                    lat_deg=int(row["INS_Pos_Abs_Latitude"]) * LAT_LON_LSB,
                    lon_deg=int(row["INS_Pos_Abs_Longitude"]) * LAT_LON_LSB,
                    heading_deg=int(row["INS_ANGLE_TRUE_HEADING"]) * HEADING_LSB,
                    vx_mps=int(row["INS_Vel_Frame_X"]) * VEL_LSB,
                    vy_mps=int(row["INS_Vel_Frame_Y"]) * VEL_LSB,
                    vz_mps=int(row["INS_Vel_Frame_Z"]) * VEL_LSB,
                )
            )
    poses.sort(key=lambda p: p.t_us)
    return EgoTrace(poses=poses)
