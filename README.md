# PreTwin

A tool for reviewing, validating, and repairing recorded vehicle-trajectory
logs before they're used to build simulation scenarios. Each recorded log is
a pair of files:

- **ADMA file** (`.csv`) — the GeneSys ADMA reference INS/GPS system's ego
  trajectory (position, heading, velocity).
- **Annotation file** (`.xml`) — a manually-labeled scene export (Lidar +
  vision) with per-frame vehicle bounding boxes, lane markings, road edges,
  and static objects, all in the ego vehicle's own reference frame.

PreTwin parses both, reconstructs a single consistent global-coordinate
scene, replays it in a browser GUI, flags physically-implausible annotation
data, applies rule-based fixes, predicts a vehicle's likely path before it
entered the Lidar's field of view, and exports the corrected trace plus an
OpenDRIVE/OpenSCENARIO bundle for simulation.

## Quick start

```bash
pip install -r requirements.txt
PYTHONPATH=backend python3 -m trace_fixer.main   # serves on http://localhost:8000
```

(The Python package is still named `trace_fixer` internally — only the
displayed product name changed to PreTwin.)

Open `http://localhost:8000` in a browser. Two sample traces are bundled
and load automatically: `sample1` and `sample2` (`data/traces/sample1/`,
`sample2/`) — see *Why vehicle heading can look "botched"* below for why
they're both worth keeping around even though their annotation exports
turned out to use the same `zrot` convention. Add more traces either by:

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

The right-hand panel is where you inspect and fix a trace; a left-hand
**Ego Odometry** panel shows a few raw, always-cheap-to-compute ego stats
(elapsed time, speed, compass heading, distance traveled) as the trace
plays — nothing that requires re-running validation or analysis, so it
updates every frame for free.

1. **Pick a trace** from the trace picker (top bar) — it's a searchable
   list, not a plain dropdown, so it stays usable with a corpus of
   thousands (type to filter; it queries the server rather than holding
   every trace client-side). The **◀ / ▶** buttons beside it step to the
   previous/next trace in the current list — if you've typed a search
   filter, stepping stays within those filtered results. The list itself
   is paginated 200 at a time; use the **◀ / ▶** page controls below it to
   page through a larger corpus (searching narrows the page count too).
2. **Vehicles panel** lists every vehicle in the scene (id, object type, and
   the time range it's observed over). Click a vehicle to jump the timeline
   to its first observation, highlight it in the viewport with the same
   blue selection ring used for issue clicks, resume playback, and keep the
   camera centered on it as it moves. Click the same vehicle again to
   deselect it — the ring disappears and the camera goes back to following
   the ego.
3. **Events panel** lists the behavioral events detected in the trace —
   braking, overtakes, short-headway/near-miss, cut-ins, standstills, sharp
   turns; see *Trace catalog* below for what each one means. Click one to
   jump the timeline to it and highlight the vehicle involved (or the ego,
   for an ego-only event like braking or a standstill). This is a different
   list from Issues below: events are "what happened," issues are "what's
   wrong."
4. **Run validation** to flag implausible vehicle motion, collisions, and
   off-road excursions in the issue list. Click an issue to jump the
   timeline to it and highlight the vehicle.
5. **Predict outside FOV** extrapolates a plausible path for any vehicle
   before it entered the ~120° front-bumper Lidar cone (already moving when
   first observed) *and* after it left the cone (most commonly the ego
   overtaking it, or it overtaking the ego, while it's presumably still on
   the road). Predicted segments render dashed/purple and are excluded from
   export unless you ask for them (`include_predictions` on the annotation
   export).
6. **Apply fixes** smooths flagged vehicle tracks, clamps positions back
   inside the annotated road corridor, and drops trailing observations that
   still overlap the ego vehicle after smoothing (a common "lost the track
   right as it merged into our lane" artifact). Re-run validation any time
   to see what's left.
7. **Sync offset** nudges the annotation clock against the ADMA clock (see
   *Time alignment* below) — drag while watching the replay.
8. **Export** the fixed ADMA CSV, fixed annotation XML, an
   OpenDRIVE + OpenSCENARIO `.zip`, or a **trace summary** (`.txt` or
   `.xml`) — see *Trace summary report* below. Every export writes into
   `output/` and never triggers a browser download — see *Output directory*
   below.

"Reset trace" reloads the original files from disk, discarding all fixes/
predictions/offset changes made in the session.

### Playback controls

Restart/replay (⏮), step back/forward 1s, play/pause, a loop toggle, and a
speed selector, all in the timeline bar. Keyboard shortcuts (ignored while
typing in a text field): **Space** play/pause, **←/→** step back/forward 1s,
**Home** jump to the start. The play button itself turns into a replay
button (↻) once the timeline reaches the end, so pressing it (or Space)
again starts over from the beginning instead of doing nothing.

Dragging the timeline seeks *and* resumes playback from that point with the
camera centered back on the ego vehicle (step back/forward still just pause
at the new time, for frame-by-frame inspection). Clicking a vehicle in the
**Vehicles** panel does the same but keeps the camera centered on that
vehicle instead of the ego, for as long as it's in view — manually panning,
zooming, or hitting recenter (⊕) drops back to following the ego. Clicking
the same vehicle again in the Vehicles panel deselects it and hands the
camera back to the ego, the same as recenter.

A small readout in the bottom-left corner of the viewport shows the ego's
live GPS coordinates (lat, lon) as the trace plays; click it to copy them
to the clipboard.

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

Type a path directly into the field, or click **Browse…** to navigate the
filesystem *on the machine running the server* (the normal case for this
tool, since it's meant to run locally) — it opens a small folder browser
under the field: click a subfolder to descend into it, the ↑ button to go
up, and **Use this folder** to fill the path field with wherever you've
navigated to. It only lists folders, not files, and skips hidden (dot)
directories.

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
the result. Every artifact — corrected ADMA + annotation, OpenDRIVE +
OpenSCENARIO, and a trace summary report — is written to `output/` for each
processed trace; see *Output directory* below.

This works well up to however many traces you're willing to select by hand
in the trace picker. For a whole corpus — including one larger than the
picker's 200-per-page display — the Scan directory panel has three buttons
that each run a background job over *every* trace currently registered with
the server (not just what's shown or selected), and return immediately; the
GUI polls for progress (`n/total processed`, current trace name, any
failures) until it finishes:

- **Fix Traces** — the same fix + predict + write-to-`output/` pipeline as
  above, over the whole corpus.
- **Build Catalog** — validates every trace and records it into the *trace
  catalog* (see below) — location, road/weather/light conditions,
  phenomenon tags, and issue counts — without touching `output/` at all.
  Cheaper than Fix Traces when you just want to triage a corpus, not
  produce corrected files yet.
- **Fix and Build Catalog** — both, in one pass per trace (one parse
  instead of two), so the catalog reflects the *corrected* trace.

They're plain buttons rather than a separate command-line script so the
whole workflow — scan, inspect a few, catalog or process everything — stays
inside one tool. A large corpus (thousands of traces) is processed one at a
time, and each trace is dropped from the server's memory cache right after
it's handled, so memory stays bounded regardless of corpus size. Only one
such run can be in flight at a time; starting another while one is running
is rejected until it finishes.

### Trace catalog

Click **Catalog…** in the top bar to open a searchable, filterable table of
every trace the tool has ever scanned or processed — meant for triaging a
large corpus without opening each trace one at a time. It's backed by a
small SQLite database at `output/catalog.sqlite`, populated three ways:

- Scanning a directory adds an identity row per matched trace (name + file
  paths) immediately — cheap enough for tens of thousands of traces since
  nothing is parsed.
- Opening a single trace in the GUI, or including it in a batch, doesn't by
  itself update the catalog.
- **Build Catalog** / **Fix and Build Catalog** (see above) is what fills in
  the rest: location (the trace's first GPS fix), duration, vehicle count,
  road type / weather / light conditions (read from the annotation file's
  per-frame metadata, when present), a set of **phenomenon** tags, and
  **issue** counts by category and severity (the same categories the issue
  list uses: `kinematic`, `collision`, `off_road`, `sync`).

Phenomena are behavioral tags detected the same way the trace summary
report's braking/overtake events are (see below), plus five more added
specifically for the catalog:

- `moderate_braking` / `hard_braking`, `vehicle_overtakes_ego` /
  `ego_overtakes_vehicle` — as in the trace summary report.
- `short_headway` / `near_miss` — a vehicle ahead in roughly the ego's own
  lane closer than 1.0s / 0.5s away at the ego's current speed.
- `cut_in` — a vehicle merges from outside the ego's lane into it while
  already close ahead (distinct from an overtake, which crosses the ego's
  centerline rather than merging into the lane ahead of it).
- `standstill` — the ego stopped or crawling (under 0.5 m/s) for 3+ seconds.
- `sharp_turn` — a sustained ego heading change of 45°+ within a 3-second
  window.

The search box filters by trace name; the phenomenon/issue chips filter by
tag, AND'd together (a trace must have every selected tag to match) — click
a chip again to remove it. Click **Open** on any row to load that trace and
close the catalog. Re-processing a trace replaces its phenomena/issues
rather than accumulating them, so the catalog always reflects the trace's
*current* state, not a history of every pass over it.

### Trace summary report

The `.txt` / `.xml` export (per-trace button, or written automatically by
the batch action) is a standalone summary meant for a QA/review team,
covering:

- **Scene composition** — object count by type (car, truck, ...).
- **Ego braking events** — sustained deceleration above ~3 m/s² (moderate)
  or ~6 m/s² (hard/AEB-like), detected from the ADMA speed profile.
- **Overtake events** — a vehicle crossing from behind the ego to ahead of
  it (`vehicle_overtakes_ego`) or vice versa (`ego_overtakes_vehicle`),
  while staying within roughly two lane-widths laterally; tags which
  vehicle, when, and which direction.
- **Data-quality issues** — the same validation findings as the GUI issue
  list (category, severity, vehicle, time range, fixed/open).

### Output directory

Every export — the per-trace buttons *and* the batch action — writes into
the project's gitignored `output/` directory, in a subdirectory per
artifact type, and *only* there: none of them trigger a browser download
into your Downloads folder. Clicking an export button writes the file
server-side and reports the path it landed at in the status line (e.g.
"Saved adma to output/adma/ADMA/sample1/adma.csv").

```
output/adma/ADMA/<trace_id>/adma.csv
output/annotations/Annotations/<original annotation filename>
output/scenarios/<trace_id>/<trace_id>.xodr
output/scenarios/<trace_id>/<trace_id>.xosc
output/reports/<trace_id>/<trace_id>_summary.<txt|xml>
output/catalog.sqlite
```

The ADMA/annotation layout mirrors the input corpus convention (annotation
filename preserved exactly as scanned, suffix variant and all) so that
output can be handed off, or re-scanned as input elsewhere, the same way
the source was. `catalog.sqlite` is the trace catalog (see above) — the one
thing in `output/` that isn't a per-trace export, since it's a single
database file covering the whole corpus.

## Architecture

```
backend/trace_fixer/
  parsers/adma_csv.py         ADMA CSV -> EgoTrace (lat/lon/heading/velocity)
  parsers/annotation_xml.py   Annotation XML -> vehicles, lane markings,
                               border lines, static objects (all ego-relative);
                               normalizes vehicle zrot (always degrees in
                               the raw file) to radians
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
  analysis.py                 object counts, ego braking/overtake/short-
                               headway/cut-in/standstill/sharp-turn events --
                               feeds export/report.py and catalog.py
  catalog.py                  persistent per-trace SQLite catalog: identity,
                               location/metadata, phenomenon tags, issue
                               counts -- output/catalog.sqlite
  browse.py                   server-side directory listing for the Scan
                               directory panel's folder browser
  export/adma_writer.py       EgoTrace -> ADMA CSV
  export/annotation_writer.py surgically patches the *original* XML tree
                               (only touches what was fixed/predicted)
  export/opendrive.py         minimal piecewise-linear OpenDRIVE road
  export/openscenario.py      OpenSCENARIO FollowTrajectoryAction replay
  export/report.py            trace summary export (txt / xml): scene
                               composition, braking/overtake events, issues
  export/batch_output.py      resolves + writes every export into output/,
                               mirroring the input corpus layout for
                               ADMA/annotation (used by every export path,
                               not just the batch action)
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
  `global = ego_xy + R(yaw) @ (xp, yp)`. `zrot` is always degrees in the
  raw file (not obvious -- see *Why vehicle heading can look "botched"*
  below) and is normalized to radians internally.
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

### Why vehicle heading can look "botched"

**Short version: `zrot` (vehicle heading offset from ego) is always in
degrees in the raw annotation file.** Every export tested so far --
`sample1`, `sample2`, and five more real annotation-only files spanning
2019-2020 recordings and both `structurefile` minorversions seen (7 and
8) -- fits degrees. `VehicleObs.zrot` is normalized to radians internally
regardless; export converts back to degrees when writing fixed/predicted
values, so the round trip stays consistent with the source file.

**This took two passes to get right, and it's worth being honest about
why.** The first pass used a magnitude-only heuristic: flag a file as
degrees if any raw `zrot` value exceeds +/-pi, since that's impossible for
a properly bounded relative heading as radians (e.g. a raw value of `-8.47`
misread as radians is -485°, over a full extra rotation -- vehicles
rendered rotating in place or pointing perpendicular to their direction of
travel). That check is *necessary* but turned out not to be *sufficient*:
`sample1`'s raw values all individually happen to stay under pi (max
~2.8), so the heuristic waved it through as "radians" -- and seemed to
confirm itself, since `sample1` still had elevated-but-bounded heading
error (~20°) that looked superficially like ordinary annotation noise
rather than a wrong unit.

The tell was cross-checking decoded heading against each vehicle's own
*position-implied* direction of travel (computed independently of `zrot`,
from consecutive position deltas, against ADMA ground truth) under both
hypotheses side by side:

| Sample | Misread as radians | Correctly read as degrees |
|---|---|---|
| `sample1` | 19.9° mean error | **0.5° mean error** |
| `sample2` | 90°+ mean error (~random) | 6.4° mean error |

`sample1`'s "radians" reading wasn't a plausible-looking noise floor after
all -- it was a wrong unit that happened to produce numbers small enough
not to trip the magnitude check. Every other file tested (five more
uploaded for exactly this consistency check, all `structurefile`
minorversion 7) is unambiguous by the magnitude check alone (raw values up
to ~39), and also degrees. With zero confirmed radians files across seven
exports, `detect_vehicle_zrot_unit` in `parsers/annotation_xml.py` now
always returns `"deg"` -- kept as a function rather than inlined as a
constant in case a genuine radians file ever turns up in the ~20,000-file
corpus this is meant to scale to, at which point it needs to become a real
per-file check again (the function's docstring has the full history).

One separate, genuine (non-bug) source-data characteristic remains worth
knowing about: `zrot` is not re-estimated every single frame even within
one file. Plotting it out shows long runs of an exact, bit-for-bit-identical
value across dozens to hundreds of consecutive frames, interrupted by
occasional single-frame spikes, while the vehicle's tracked *position*
changes smoothly every frame throughout -- the signature of a sparse,
held/keyframed value from the labeling tool. Now that the unit is correct,
these held values are small enough (fractions of a degree to a few degrees)
that they don't actually produce visibly-wrong headings on their own; **Apply
fixes** still improves on them by re-deriving heading from the tangent of
each vehicle's *smoothed position path* rather than trusting `zrot` at all,
which is the more precise source of truth regardless. After fixing, stored
heading matches the position-implied direction of travel to within ~0.1°
for every vehicle in `sample1`.

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
