"""Writes an EgoTrace back out in the original ADMA CSV column layout/scaling."""
from __future__ import annotations

import csv
from pathlib import Path

from trace_fixer.models import EgoTrace
from trace_fixer.parsers.adma_csv import HEADING_LSB, LAT_LON_LSB, REQUIRED_COLUMNS, VEL_LSB


def write_adma_csv(ego: EgoTrace, output_path: str | Path) -> None:
    output_path = Path(output_path)
    with output_path.open("w", newline="") as f:
        writer = csv.writer(f, lineterminator="\n")
        writer.writerow(REQUIRED_COLUMNS)
        for p in ego.poses:
            writer.writerow(
                [
                    p.t_us,
                    round(p.heading_deg / HEADING_LSB),
                    round(p.lat_deg / LAT_LON_LSB),
                    round(p.lon_deg / LAT_LON_LSB),
                    round(p.vx_mps / VEL_LSB),
                    round(p.vy_mps / VEL_LSB),
                    round(p.vz_mps / VEL_LSB),
                ]
            )
