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

Open `http://localhost:8000` in a browser. Two sample traces are bundled
and load automatically: `sample1` (`data/traces/sample1/`, radians-unit
annotation export) and `sample2` (`data/traces/sample2/`, degrees-unit
annotation export — see *Why vehicle heading can look "botched"* below).
Add more traces either by:

- **Upload trace…** in the top bar — for one-off pairs; each upload gets its
  own `data/traces/<id>/` directory (copied onto the server).
- **Scan directory…** in the top bar — for a whole corpus at once. Give it a
  path *on the machine running the server* (this is a local tool: the
  browser and server are assumed to be operated by the same person) and it
  recursively finds every `adma.csv` and matches it to its annotation XML by
  trace name, registering thousands of traces in well under a second
  without copying any of it — each trace is only actually parsed the moment
  you open it. See *Bulk directory scanning* below for the expected layout.

Run the test suite with:

```bash
python3 -m pytest
```

## Using the GUI

1. **Pick a trace** from the trace picker (top bar) — it's a searchable
   list, not a plain dropdown, so it stays usable with a corpus of
   thousands (type to filter; it queries the server rather than holding
   every trace client-side). The **◀ / ▶** buttons beside it step to the
   previous/next trace in the current list — if you've typed a search
   filter, stepping stays within those filtered results.
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
6. **Export** the fixed ADMA CSV, fixed annotation XML, an
   OpenDRIVE + OpenSCENARIO `.zip`, or a **problem report** (`.txt` or
   `.xml`) listing every flagged issue — category, severity, vehicle,
   time range, description, fixed/open — for an annotation QA team to
   triage without opening the XML.

"Reset trace" reloads the original files from disk, discarding all fixes/
predictions/offset changes made in the session.

### Playback controls

Restart/replay (⏮), step back/forward 1s, play/pause, a loop toggle, and a
speed selector, all in the timeline bar. Keyboard shortcuts (ignored while
typing in a text field): **Space** play/pause, **←/→** step back/forward 1s,
**Home** jump to the start.

### Bulk directory scanning

`Scan directory…` expects (and tolerates minor structural variation on) a
layout like:

```
<root>/adma/ADMA/<trace_name>/adma.csv
<root>/annotations/Annotations/<trace_name>__ref-QC_IND.xml
```

Every `adma.csv` file's trace name is its parent directory's name; every
`*.xml` file is matched to a trace name by stripping known annotation-suffix
patterns (`__refQC_IND`, `__ref-QC_IND`, case-insensitive) and falling back
to a longest-prefix match for anything else. Unmatched files on either side
are reported in the scan summary rather than silently dropped. Scanned
traces are registered *by reference* — nothing is copied or parsed until you
actually open one, so scanning ~20,000 files takes well under a second.

### Batch fix + predict

Open the trace picker, check the traces you want (or **Select shown** to
grab every trace currently listed — narrow it with a search first if you
only want a subset of a big corpus), then **Fix + predict selected**. Each
selected trace is processed server-side, one at a time, through the full
validate → fix → predict outside FOV → re-validate pipeline (the same steps
the individual buttons run), with live progress and a final "issues before
→ after" summary; failures on individual traces (e.g. an unparseable file)
are reported by name rather than aborting the batch. If the trace you're
currently viewing was included, it's reloaded afterward so the GUI reflects
the result.

Each processed trace's corrected ADMA + annotation files are also written
automatically into the project's gitignored `output/` directory, mirroring
the input corpus layout:

```
output/adma/ADMA/<trace_id>/adma.csv
output/annotations/Annotations/<original annotation filename>
```

The annotation filename is preserved exactly as scanned (suffix variant and
all) so the output corpus can be handed off, or re-scanned as input
elsewhere, the same way the source was. This is specific to the batch
action — the per-trace export buttons still stream a single file to your
browser's normal download location instead.

## Architecture

```
backend/trace_fixer/
  parsers/adma_csv.py         ADMA CSV -> EgoTrace (lat/lon/heading/velocity)
  parsers/annotation_xml.py   Annotation XML -> vehicles, lane markings,
                               border lines, static objects (all ego-relative);
                               auto-detects vehicle zrot's unit (rad vs deg)
                               per file and normalizes to radians
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
  export/report.py            problem-report export (txt / xml)
  export/batch_output.py      writes corrected files into output/, mirroring
                               the input corpus layout (batch action only)
  scan.py                     bulk directory walk + ADMA<->annotation
                               filename matching, for large corpora
  scene.py                    ties it together into one JSON payload
  store.py                    trace registry: data/traces/<id>/ (copied,
                               from uploads) plus by-reference entries
                               (from directory scans)
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
  from ego, positions in meters) are in the **vehicle body frame**:
  `global = ego_xy + R(yaw) @ (xp, yp)`. `zrot`'s unit varies by export
  (radians in some files, degrees in others) and is auto-detected per file
  — see *Why vehicle heading can look "botched"* below; internally it's
  always normalized to radians.
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

### Why vehicle heading can look "botched" -- two separate causes

Two distinct issues surfaced here, on two different sample traces, and it's
worth being precise about which is which since only one of them was a bug
in Trace Fixer.

**1. `zrot` unit inconsistency across annotation exports (a real Trace
Fixer bug, now fixed).** Not every export encodes vehicle `zrot` in
radians. `sample1` (structurefile minorversion 8) does; other exports
(minorversion 7, e.g. `sample2`) encode it in **degrees** instead, with no
explicit unit field to tell them apart. Trace Fixer originally assumed
radians unconditionally, which for a degrees file turns a harmless value
like `-8.47` into "-8.47 radians" (-485°, over a full extra rotation) --
vehicles rendered rotating in place or pointing perpendicular to their
direction of travel, exactly as reported. This is now auto-detected per
file (`parsers.annotation_xml.detect_vehicle_zrot_unit`): a properly
bounded relative heading can never legitimately exceed +/-pi as radians, so
if any raw value in the file does, the file must be in degrees. Verified
against both conventions by comparing decoded heading to the direction of
travel implied independently by each vehicle's own position deltas: sample
files misread with the wrong unit produce a near-random ~90°+ average
error; correctly detected, the error is a few degrees, in line with normal
annotation noise. `VehicleObs.zrot` is always normalized to radians
internally regardless of source convention; export converts back to
whichever unit the original file used, so a "degrees" file is written back
in degrees (see `export.annotation_writer`).

**2. Long held/keyframed `zrot` values within a single file (a genuine
source-data characteristic, not a bug).** Independent of the unit issue,
`zrot` is not tracked continuously frame-by-frame even within one file.
Plotting it out shows long runs of an exact, bit-for-bit-identical value
across dozens to hundreds of consecutive frames (e.g. one `sample1` vehicle
holds the same value for 295 frames straight), interrupted by occasional
single-frame spikes, while the vehicle's tracked *position* changes
smoothly every frame throughout. That pattern -- long constant holds, sharp
jumps, no frame-to-frame drift -- is the signature of a sparse,
held/keyframed value from the labeling tool, not per-frame sensor noise or
a decoding error: `heading_deg = ego_yaw + zrot` reproduces it faithfully
because that's genuinely what's in the file.

**Apply fixes** already corrects #2 (and would mask #1 too, which is why
it's worth fixing #1 at the source instead of relying on that): it
re-derives each vehicle's heading from the tangent of its *smoothed
position path* rather than trusting `zrot` at all, then re-encodes the
corrected heading back into the exported `zrot` (in the file's own unit).
After fixing, stored heading matches the position-implied direction of
travel to within ~0.1° for every vehicle in `sample1`. If you want to see
the raw, unfixed annotation's heading error for yourself, run validation
before fixing -- the erratic values show up as `kinematic` / yaw-rate
issues, which is the same signal that flags this automatically.

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
