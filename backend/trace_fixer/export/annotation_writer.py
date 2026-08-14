"""Writes a fixed/predicted annotation XML by surgically patching the
*original* file's tree: only vehicle bounding-box coordinates that were
actually changed are overwritten, dropped (trailing ego-collision trim)
observations are removed, and predicted pre/post-FOV observations are
inserted as new timestamp elements. Everything else in the file (lane markings,
border polygons, static objects, scene metadata) is left byte-for-byte as
authored by the annotation team, since only vehicle tracks are within this
tool's fix/predict scope.

Predicted entries are marked so downstream consumers can filter them out:
  - interpolationState="predicted" (a value not used by the original tool)
  - obj_confidence = "Predicted"
  - frame = a negative integer (real frames are always >= 1)
"""
from __future__ import annotations

import copy
import xml.etree.ElementTree as ET
from pathlib import Path

from trace_fixer.models import Trace


def _fmt(v: float) -> str:
    return repr(float(v))


def write_annotation_xml(
    trace: Trace,
    original_xml_path: str | Path,
    output_path: str | Path,
    include_predictions: bool = True,
) -> None:
    tree = ET.parse(original_xml_path)
    root = tree.getroot()
    vehicles_el = root.find("vehicles")
    if vehicles_el is None:
        raise ValueError("Source annotation XML has no <vehicles> section")

    for rv in vehicles_el.findall("rect_vehicle"):
        obj_id_text = rv.findtext("id")
        if obj_id_text is None:
            continue
        obj_id = int(obj_id_text)
        track = trace.annotation.vehicles.get(obj_id)
        if track is None:
            continue

        timestamps_el = rv.find("timestamps")
        if timestamps_el is None:
            continue

        existing_by_frame = {
            int(ts.attrib["frame"]): ts for ts in timestamps_el.findall("rect_vehicle_timestamp")
        }
        real_obs = [o for o in track.observations if not o.synthetic]
        kept_frames = {o.frame for o in real_obs}

        # Drop timestamps for observations the fix engine removed (e.g. a
        # trailing ego-overlap trim).
        for frame, ts_el in list(existing_by_frame.items()):
            if frame not in kept_frames:
                timestamps_el.remove(ts_el)

        # Patch coordinates for observations the fix engine adjusted.
        for obs in real_obs:
            if not obs.fixed:
                continue
            ts_el = existing_by_frame.get(obs.frame)
            if ts_el is None:
                continue
            coords = ts_el.find("bounding_box/vehicle_bb/coordinates")
            if coords is None:
                continue
            coords.find("xp").text = _fmt(obs.x_rel)
            coords.find("yp").text = _fmt(obs.y_rel)
            coords.find("zrot").text = _fmt(obs.zrot)

        if include_predictions:
            synthetic = [o for o in track.observations if o.synthetic]
            if synthetic:
                template = timestamps_el.find("rect_vehicle_timestamp")
                if template is not None:
                    new_elements = []
                    for i, obs in enumerate(synthetic):
                        el = copy.deepcopy(template)
                        el.set("time", str(obs.t_us))
                        el.set("chunktime", str(obs.t_us))
                        el.set("frame", str(-(len(synthetic) - i)))
                        el.set("interpolationState", "predicted")
                        movement = el.find("obj_movement")
                        if movement is not None:
                            movement.text = obs.obj_movement
                        lane = el.find("obj_lane")
                        if lane is not None:
                            lane.text = obs.obj_lane
                        confidence = el.find("obj_confidence")
                        if confidence is not None:
                            confidence.text = "Predicted"
                        coords = el.find("bounding_box/vehicle_bb/coordinates")
                        if coords is not None:
                            coords.find("xp").text = _fmt(obs.x_rel)
                            coords.find("yp").text = _fmt(obs.y_rel)
                            coords.find("zp").text = _fmt(obs.z_rel)
                            coords.find("xs").text = _fmt(obs.length)
                            coords.find("ys").text = _fmt(obs.width)
                            coords.find("zs").text = _fmt(obs.height)
                            coords.find("zrot").text = _fmt(obs.zrot)
                        new_elements.append(el)
                    timestamps_el.extend(new_elements)

        # Keep timestamp elements in chronological order.
        children = list(timestamps_el)
        children.sort(key=lambda el: int(el.attrib.get("chunktime", 0)))
        timestamps_el[:] = children

    tree.write(output_path, encoding="iso-8859-1", xml_declaration=True)
