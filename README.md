# Trace Fixer

A tool for reviewing, validating, and repairing recorded vehicle-trajectory
logs before they're used to build simulation scenarios. Each recorded log is
a pair of files:

- **ADMA file** (`.csv`) — the GeneSys ADMA reference INS/GPS system's ego
  trajectory (position, heading, velocity).
- **Annotation file** (`.xml`) — a manually-labeled scene export (Lidar +
  vision) with per-frame vehicle bounding boxes, lane markings, road edges,
  and static objects, all in the ego vehicle's own reference frame.

Trace Fixer parses both, reconstructs a single consistent global-coordinate
scene, replays it in a browser GUI, flags physically-implausible annotation
data, applies rule-based fixes, predicts a vehicle's likely path before it
entered the Lidar's field of view, and exports the corrected trace plus an
OpenDRIVE/OpenSCENARIO bundle for simulation.

## Quick start

```bash
pip install -r requirements.txt
PYTHONPATH=backend python3 -m trace_fixer.main   # serves on http://localhost:8000
```

Open `http://localhost:8000` in a browser. A sample trace (`sample1`) is
bundled under `data/traces/sample1/` and loads automatically. Use "Upload
trace…" in the top bar to add another ADMA CSV + annotation XML pair (each
upload gets its own `data/traces/<id>/` directory).

Run the test suite with:

```bash
python3 -m pytest
```

## Using the GUI

1. **Pick a trace** from the dropdown (or upload a new pair).
2. **Run validation** to flag implausible vehicle motion, collisions, and
   off-road excursions in the issue list. Click an issue to jump the
   timeline to it and highlight the vehicle.
3. **Predict outside FOV** extrapolates a plausible path for any vehicle
   before it entered the ~120° front-bumper Lidar cone (already moving when
   first observed) *and* after it left the cone (most commonly the ego
   overtaking it, or it overtaking the ego, while it's presumably still on
   the road). Predicted segments render dashed/purple and are excluded from
   export unless you ask for them (`include_predictions` on the annotation
   export).
4. **Apply fixes** smooths flagged vehicle tracks, clamps positions back
   inside the annotated road corridor, and drops trailing observations that
   still overlap the ego vehicle after smoothing (a common "lost the track
   right as it merged into our lane" artifact). Re-run validation any time
   to see what's left.
5. **Sync offset** nudges the annotation clock against the ADMA clock (see
   *Time alignment* below) — drag while watching the replay.
6. **Export** the fixed ADMA CSV, fixed annotation XML, or an
   OpenDRIVE + OpenSCENARIO `.zip` for simulation.

"Reset trace" reloads the original files from disk, discarding all fixes/
predictions/offset changes made in the session.

## Architecture

```
backend/trace_fixer/
  parsers/adma_csv.py         ADMA CSV -> EgoTrace (lat/lon/heading/velocity)
  parsers/annotation_xml.py   Annotation XML -> vehicles, lane markings,
                               border lines, static objects (all ego-relative)
  geo/transform.py            lat/lon -> local ENU meters, ego pose
                               interpolation, ego-relative <-> global
  geo/populate.py             fills in global (x, y, heading) on every
                               annotation element, using the ego trace as
                               the reference frame -- single source of truth
                               for scene/validation/prediction/fix/export
  geo/sync.py                 ADMA-clock <-> annotation-clock offset
  geo/road_corridor.py        derives drivable-corridor bounds from the
                               annotated Road Edge / Guardrail polylines
  geo/collision.py            oriented-bounding-box overlap test (SAT)
  validation/checks.py        kinematic feasibility, collision, off-road
  validation/fixes.py         smoothing, off-road clamp, trailing-collision
                               trim
  prediction/extrapolate.py   backward, lane-following trajectory prediction
  export/adma_writer.py       EgoTrace -> ADMA CSV
  export/annotation_writer.py surgically patches the *original* XML tree
                               (only touches what was fixed/predicted)
  export/opendrive.py         minimal piecewise-linear OpenDRIVE road
  export/openscenario.py      OpenSCENARIO FollowTrajectoryAction replay
  scene.py                    ties it together into one JSON payload
  store.py                    in-memory trace registry (data/traces/<id>/)
  api.py                      FastAPI app + REST endpoints
frontend/                     vanilla JS + canvas 2D top-down viewer
                               (no build step)
```

## Coordinate & unit conventions

These were reverse-engineered from the sample trace against the ADMA 3.0
"Data Packets" manual (which doesn't state sign conventions explicitly) by
cross-checking `INS_ANGLE_TRUE_HEADING` and `INS_Vel_Frame_X/Y` against GPS
displacement over time:

- `INS_Pos_Abs_Latitude` / `_Longitude`: LSB `1e-7` deg (per the manual).
- `INS_Vel_Frame_X` / `_Y` / `_Z`: LSB `0.005` m/s (per the manual) — **and**
  X/Y are **North/East** local-level-frame components, not vehicle-frame
  forward/lateral velocity, despite the "Frame" name suggesting otherwise.
- `INS_ANGLE_TRUE_HEADING`: LSB `0.01` deg (per the manual, same convention
  as `Tilt_Yaw`/`GPS_Course_Over_Ground`) — **and** it turns out to be a
  counterclockwise-from-north angle, the opposite rotational sense of a
  normal compass bearing. Empirically: `yaw_deg (CCW from East, standard
  math convention) = (90 + INS_ANGLE_TRUE_HEADING) % 360`. Verified to
  match the velocity-implied heading to within ~0.1° across the whole
  sample trace.
- Local ground plane used throughout the codebase: `x = East`, `y = North`
  (meters), anchored at the trace's first ADMA sample (equirectangular
  projection — fine at single-log scale).
- Annotation bounding boxes (`xp` forward, `yp` left, `zrot` heading offset
  from ego, all in radians/meters) are in the **vehicle body frame**:
  `global = ego_xy + R(yaw) @ (xp, yp)`.
- `ADTF_CHUNK_TIME` (ADMA) and the `chunktime` attribute (annotation) are
  both ADTF pipeline clocks, ~100 Hz for ADMA and much sparser for
  annotation keyframes; see *Time alignment* below.

## Time alignment ("not fully synced" bottleneck)

Both files carry a "chunktime" field, which in a clean recording session is
the same ADTF pipeline clock and can be used directly with zero offset —
that's the default. In practice the two loggers can start slightly apart or
drift, so the GUI exposes an adjustable `sync_offset_us`, applied to the
annotation clock before it's used to sample the ego trace. There's no
ground truth in a single trace to fit an automatic offset against, so this
is a manual nudge-while-watching-the-replay control rather than an
auto-correlation.

## Validation & fixes: what's checked, what's fixed

Deliberately simple, explainable rule-based checks rather than a learned
model — there's no labeled "this annotation is wrong" data to train against,
and explainable rules are what an annotation QA team can act on directly.

| Check | What it flags | Auto-fixed? |
|---|---|---|
| Kinematic (vehicles) | Implied speed / acceleration / yaw-rate between consecutive observations exceeds highway-driving thresholds | Yes — spline smoothing of position, heading re-derived from the smoothed path tangent |
| Kinematic (ego/ADMA) | Same, on the ADMA trace itself | No — flagged only; the ego trace is the foundation everything else is built on, so it isn't auto-edited without review |
| Off-road | Vehicle crosses the nearest annotated Road Edge / Guardrail boundary | Yes — lateral clamp back inside the corridor (+ margin) |
| Collision (vehicle↔ego) | Bounding boxes overlap | Trailing overlaps (track ends inside the ego box — a common "lost track as it merged" artifact) are trimmed. Mid-track overlaps are flagged only |
| Collision (vehicle↔vehicle) | Bounding boxes overlap | Flagged only (no auto-fix — resolving which of two vehicles is "wrong" isn't well-defined without more context, including between two independently-predicted pre-FOV segments) |

## Known limitations / scope (v1)

- **OpenDRIVE is intentionally minimal**: a piecewise-linear reference line
  along the (fixed) ego path, constant lane count/width. This is meant to
  be good enough for this tool's own in-GUI sanity-check simulation, *not*
  a replacement for the HERE-derived map Applied Intuition already builds
  from the ADMA trace. The reference line runs along the edge of the ego's
  lane rather than precisely through its center (a half-lane-width
  simplification).
- **Prediction is per-vehicle** (both directions), using a constant-speed,
  lane-tangent-following model. It doesn't reason about other traffic, so
  two independently-predicted vehicles can end up flagged as colliding —
  that's a real signal ("these two tracks are ambiguous while unobserved"),
  not a bug, and is left for manual review rather than silently resolved.
- **Ego (ADMA) trace fixes are flag-only** in v1 (see table above).
- Lane markings/road edges are stored in the annotation as sparse
  ego-relative keyframes (not one per frame); the GUI renders the union of
  all keyframe snapshots as static road geometry, which is a good
  approximation but not a continuously-updated live corridor.
