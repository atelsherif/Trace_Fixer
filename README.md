# PreTwinner

A tool for reviewing, validating, and repairing recorded vehicle-trajectory
logs before they're used to build simulation scenarios. Each recorded log is
a pair of files:

- **ADMA file** (`.csv`) — the GeneSys ADMA reference INS/GPS system's ego
  trajectory (position, heading, velocity).
- **Annotation file** (`.xml`) — a manually-labeled scene export (Lidar +
  vision) with per-frame vehicle bounding boxes, lane markings, road edges,
  and static objects, all in the ego vehicle's own reference frame.

PreTwinner parses both, reconstructs a single consistent global-coordinate
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
displayed product name changed, first to PreTwin and now to PreTwinner.)

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

### Logo

Drop a logo file at `frontend/logo.png` and it's picked up automatically —
no code change needed, no restart required (a static file, served
straight from `frontend/`): it becomes both the browser-tab favicon and
the small mark next to "PreTwinner" in the top bar. Until that file
exists, both spots quietly fall back to no icon / text-only (an
`onerror` handler in `index.html`, not a missing-image icon), so its
absence is never an error.

Recommended: **square**, **PNG**, **at least 256×256px** (512×512 is
safer for a crisp favicon on high-DPI displays) — the topbar mark is
rendered at 22×22px (`#app-logo` in `style.css`) and the browser picks
whatever favicon size it wants from the same file, so one reasonably
large square image covers both uses; no need to pre-generate multiple
sizes or an `.ico`. Transparent background works fine for the topbar
mark; for the favicon, a background that reads on both light and dark
browser chrome (most logos with real content already do) is safer than
fully transparent.

## Using the GUI

The layout is three columns. A thin titlebar above the viewport names the
currently loaded trace. The **left** panel is the "what is this trace"
side — an **Ego Odometry** readout plus the Vehicles, Static Objects, and
Events lists, none of which require re-running validation or analysis, so
they're cheap to keep visible while you scrub through playback. The
**right** panel is the "do something about it" side — validate, predict,
fix, and export.

The viewport draws a world-anchored 10m ground grid (so a highway trace
with the camera locked to the ego reads as *moving*, not as a static
picture of near-identical lane markings), the ego's full route as a faint
line with the driven portion solid over it, and a 3-second fading trail
behind every visible vehicle — enough to read a cut-in or a lane change
from a single frame without scrubbing.

1. **Pick a trace** from the trace picker (top bar) — it's a searchable
   list, not a plain dropdown, so it stays usable with a corpus of
   thousands (type to filter; it queries the server rather than holding
   every trace client-side). The **◀ / ▶** buttons beside it step to the
   previous/next trace in the current list — if you've typed a search
   filter, stepping stays within those filtered results. The list itself
   is paginated 200 at a time; use the **◀ / ▶** page controls below it to
   page through a larger corpus (searching narrows the page count too).
2. **Ego Odometry** (left panel) shows elapsed time, speed, forward/
   lateral/vertical velocity, compass heading, and distance traveled —
   see *Coordinate & unit conventions* below for what forward/lateral/
   vertical actually mean (it's not simply the raw ADMA velocity channels).
3. **Vehicles panel** (left) lists every vehicle in the scene (id, object
   type, and the time range it's observed over). Click a vehicle to jump
   the timeline to its first observation, highlight it in the viewport with
   the same blue selection ring used for issue clicks, resume playback, and
   keep the camera centered on it as it moves. Click the same vehicle again
   to deselect it — the ring disappears and the camera goes back to
   following the ego.
4. **Static Objects panel** (left) lists every static object (traffic
   signs, reflective markers, and similar roadside fixtures the annotation
   carries) in the scene, same shape as the Vehicles panel (id + type).
   Since these don't move, clicking one doesn't touch the timeline or
   playback — it re-centers the camera on the object once (turning off
   ego-follow so it stays put while the trace keeps playing) and draws a
   blue highlight ring around it, enabling the **Static objects** viewport
   checkbox first if it was off. Click the same object again to deselect it
   and hand the camera back to following the ego.
5. **Events panel** (left) lists the behavioral events detected in the
   trace — braking, overtakes, short-headway/near-miss, cut-ins,
   standstills, sharp turns; see *Trace catalog* below for what each one
   means. Click one to jump the timeline to it and highlight the vehicle
   involved (or the ego, for an ego-only event like braking or a
   standstill). This is a different list from Issues: events are "what
   happened," issues are "what's wrong."
6. **Run validation** (right panel) to flag implausible vehicle motion,
   collisions, and off-road excursions in the issue list. Click an issue to
   jump the timeline to it and highlight the vehicle.
7. **Predict outside FOV** extrapolates a plausible path for any vehicle
   before it entered the ~120° front-bumper Lidar cone (already moving when
   first observed) *and* after it left the cone (most commonly the ego
   overtaking it, or it overtaking the ego, while it's presumably still on
   the road). Predicted segments render dashed/purple and are excluded from
   export unless you ask for them (`include_predictions` on the annotation
   export). A vehicle last (or first) seen *behind* the ego gets 3x the
   base horizon in that direction (`REAR_HORIZON_MULTIPLIER` in
   `prediction/extrapolate.py`) — rear/side sensor coverage is typically
   shorter-range than the front cone, so a trailing vehicle (a tailgater,
   or one the ego is pulling away from) tends to drop out of the track
   sooner, which is exactly the case where a longer predicted trail
   matters most. The **Horizon (s)** field sets the base value the
   multiplier is applied to (default 6s, so a trailing vehicle defaults
   to 18s); **Horizon (m)** is an optional second cap, in meters (e.g. a
   sensor's real-world range) — whichever of the two limits is hit first
   stops the prediction, and either can be left blank to disable it (both
   blank falls back to a 4s time-only default). Re-running validation
   afterward is automatic either way; the status line reports the before
   → after issue count so a prediction that introduced new ones is
   visible immediately rather than a silent surprise.

   **Avoid predicted collisions** (checked by default) makes a predicted
   vehicle brake — down to a full stop if needed, never swerve — rather
   than driving straight through the ego or another vehicle's box; see
   `REAR_HORIZON_MULTIPLIER`'s neighboring constants in
   `prediction/extrapolate.py` for the braking/recovery rates, both kept
   well under the validator's own "unrealistic acceleration" threshold so
   the governor's own braking never introduces a *new* kinematic issue.
   It runs `predict_all` in two passes: an ungoverned first pass gives
   every vehicle some full-length predicted path, so the second, governed
   pass has real data on every *other* vehicle to react to, not just
   whichever ones happen to be processed earlier in a single pass. A
   conflict closing faster than realistic braking can resolve still shows
   up as a collision issue afterward — that's the honest outcome for a
   speed-only governor with no steering, not a bug.
8. **Apply fixes** smooths flagged vehicle tracks, clamps positions back
   inside the annotated road corridor, and drops trailing observations that
   still overlap the ego vehicle after smoothing (a common "lost the track
   right as it merged into our lane" artifact). Re-run validation any time
   to see what's left.
9. **Sync offset** nudges the annotation clock against the ADMA clock (see
   *Time alignment* below) — drag while watching the replay.
10. **Export**, one artifact type per button: **Fixed Trace
    (ADMA+Annotation)** writes both the corrected ADMA CSV and annotation XML
    in one action; **OpenDRIVE** (with the **Enrich with OpenStreetMap
    (online)** checkbox next to it — optional, off by default, see *Online
    map enrichment* below for what it does and doesn't affect); **OpenSCENARIO**
    and **ADP scenario (.scn.yaml)** (for ADP, which doesn't read `.xosc` —
    see *ADP YAML export* below, including the optional **Map key** field
    above the two, and the **POV** selector next to it — see *Vehicle
    point-of-view export* below); and a **trace summary** as `.txt` or
    `.xml` — see *Trace summary report* below. Every export writes into
    `output/` and never triggers a browser download — see *Output
    directory* below.
11. **Map (OSM)** (viewport controls, top-left of the canvas) overlays
    nearby OpenStreetMap roads on the visualizer for the currently loaded
    trace, once you check it — see *Map overlay in the visualizer* below.
12. **Alternative Scenarios** (below Export) generates a menu of harder/
    different scenarios from the current trace by perturbing one
    surrounding vehicle's trajectory at a time, previewable and separately
    exportable — see *Alternative scenarios (ODD variants)* below.

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

Every `adma.csv` file's trace name comes from its nearest *distinctive*
ancestor directory — nearest first, skipping generic container names
(`adma/`, `data/`, `raw/`, `logs/`, `recordings/`, …). Taking the immediate
parent unconditionally collapses a corpus laid out as
`<root>/<trace>/adma/adma.csv` into a single trace called "adma": every
`adma.csv` shares one derived name, and all but the first are dropped as
collisions. Every `*.xml` file is matched to a trace name by stripping known
annotation-suffix patterns (`__refQC_IND`, `__ref-QC_IND`, case-insensitive)
and falling back to a longest-prefix match for anything else.

The scan summary reports the raw `adma.csv` file count alongside the number
of distinct trace names it produced, any name collisions, and a sample of
the unmatched files on both sides — so a corpus that scans to a
surprisingly low number of pairs can be diagnosed from the GUI rather than
guessed at. It also separately reports **duplicates**: a second `adma.csv`
or annotation file resolving to a trace name a *first* one already
claimed (e.g. a reprocessed/re-exported copy of the same trace living
elsewhere in the corpus) is a real match, just not the one that won the
slot — distinct from "unmatched" (it never failed to resolve a trace name
at all), and previously invisible in every diagnostic. Each example names
both the dropped path and the path that won, so a specific missing pair
can be traced to its duplicate.

The walk follows symlinked subdirectories (cycle-safe — a symlink pointing
back at one of its own ancestors is skipped on its second visit rather
than followed forever). This matters because large real corpora are
routinely organized with them (a shared-storage mount, a dedup layer, a
"latest" pointer tree), and without it, everything past a first symlink
in the tree is silently invisible to a scan — no error, no diagnostic,
just a much lower match count than the same corpus produced before,
which is indistinguishable from a dozen other causes until you know to
look for it.

Scanned traces are registered *by reference* — nothing is copied or
parsed until you actually open one, so scanning ~20,000 files takes well
under a second.

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
picker's 200-per-page display — the Scan directory panel runs a background
job over *every* trace currently registered with the server (not just what's
shown or selected) and returns immediately; the GUI polls for progress
(`n/total processed`, current trace name, any failures) until it finishes:

- **Build Catalog** — validates every trace and records it into the *trace
  catalog* (see below) — location, road/weather/light conditions,
  phenomenon tags, and issue counts — without touching `output/` at all.
  Cheaper than a full run when you just want to triage a corpus, not
  produce corrected files yet.
- **Fix and Export** — a configurable run over the whole corpus:
  - **Fix Issues** / **Predict trajectories** (both on by default) — which
    steps of validate → fix → predict outside FOV → re-validate to actually
    run. Turning both off still validates before and after (so the "before/
    after" counts stay meaningful) without changing anything.
  - **Export types** — which artifacts to write to `output/` for each
    processed trace, independently checkable: **Fixed Trace
    (ADMA+Annotation)** (checked by default), **OpenDRIVE**, **OpenSCENARIO**,
    **ADP scenario (.scn.yaml)**, **Trace summary (.txt)**, **Trace summary
    (.xml)** — the same artifact types as the single-trace Export panel (see
    *Using the GUI* above), just applied corpus-wide. Next to OpenDRIVE, an
    **Enrich with OpenStreetMap (online)** checkbox mirrors the single-trace
    one (see *Online map enrichment* below) — off by default, only takes
    effect when OpenDRIVE is checked, and a lookup failure for one trace
    falls back to the offline result for that trace without affecting the
    rest of the run. ADP scenario's `map.key` isn't configurable here — each
    trace gets its own trace_id default, same as leaving the single-trace
    Map key field blank, since a whole corpus can't sensibly share one map
    key anyway.
  - **Also build catalog** — folds cataloging the *corrected* trace into the
    same pass (one parse instead of two), equivalent to the old "Fix and
    Build Catalog" button.
  - The status line afterward summarizes what actually ran (e.g. "fixed
    issues; wrote Fixed Trace, ADP scenario"), reflecting the checkboxes as
    they were at the moment you clicked **Run**.

It's a plain panel rather than a separate command-line script so the whole
workflow — scan, inspect a few, catalog or process everything — stays inside
one tool. A large corpus (thousands of traces) is processed one at a time,
and each trace is dropped from the server's memory cache right after it's
handled, so memory stays bounded regardless of corpus size. Only one such
run can be in flight at a time; starting another while one is running is
rejected until it finishes.

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
- **Build Catalog**, or **Fix and Export** with **Also build catalog**
  checked (see above), is what fills in the rest: location (the trace's
  first GPS fix), duration, vehicle count,
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

- **Scene composition** — moving-object count by type (car, truck, ...).
- **Static objects** — count by type of the annotation's static objects
  (traffic signs, reflective markers, and similar roadside fixtures) —
  same shape as scene composition, just for things that don't move.
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
output/adma/ADMA/<trace_id>/adma[__<provenance>].csv
output/annotations/Annotations/<original annotation stem>[__<provenance>].xml
output/scenarios/<trace_id>/<trace_id>[__<provenance>].xodr
output/scenarios/<trace_id>/<trace_id>[__<provenance>].xosc
output/scenarios/<trace_id>/<trace_id>[__<provenance>].scn.yaml
output/reports/<trace_id>/<trace_id>_summary[__<provenance>].<txt|xml>
output/exports.jsonl
output/catalog.sqlite
```

The ADMA/annotation layout mirrors the input corpus convention (annotation
filename preserved exactly as scanned, suffix variant and all) so that
output can be handed off, or re-scanned as input elsewhere, the same way
the source was. `catalog.sqlite` is the trace catalog (see above) — the one
thing in `output/` that isn't a per-trace export, since it's a single
database file covering the whole corpus.

#### Export provenance

`<provenance>` records *which version of the trace* an export came from,
and it is what keeps exports from overwriting each other:

| Suffix | Means |
| --- | --- |
| *(none)* | the trace exactly as recorded |
| `__fixed` | after **Apply fixes** |
| `__predicted` | after **Predict outside FOV** |
| `__fixed__predicted` | both — what the batch pipeline writes by default |
| `__pov<id>` | re-rooted to a vehicle's point of view (see *Vehicle point-of-view export*) |

Without it, exporting a trace, fixing it, and exporting again writes both
results to the same path: the second silently replaces the first, and
neither file says which it is. Alternative scenarios don't take a suffix —
a variant's identity already lives in its own `trace_id`
(`<trace_id>__<variant_id>`).

Each batch run stamps every artifact it writes with one shared suffix, so
a `.xosc` and the `.xodr` it references stay a matched set even when the
original and the fixed version of a trace have both been exported.

The Export panel says which version its buttons will write ("Exporting:
original trace" / "fixed trace" / "variant: Harder cut-in"), and its
**Recent exports** list shows what this trace has already produced.

`output/exports.jsonl` is the same record on disk — one JSON line per
export with the timestamp, artifact kind, trace, provenance and file paths.
It is append-only, so a long session's history survives and concurrent
batch writers can't clobber each other's entries. `GET /api/exports`
(optionally `?trace_id=…`) reads it back, newest first.

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
  export/road_geometry.py     reconstructs road geometry (curvature-real
                               reference line, annotation-derived lane
                               sections with a sanity-checked fallback,
                               static-object placement) -- see "OpenDRIVE
                               generation" below
  export/opendrive.py         serializes road_geometry's plan to .xodr
  export/map_enrichment.py    optional, opt-in online road lookup (OSM
                               today) behind a swappable provider interface
                               -- see "Online map enrichment" below
  export/openscenario.py      OpenSCENARIO FollowTrajectoryAction replay
  export/adp_yaml.py          ADP (Applied Intuition Simian) .scn.yaml --
                               alternative to OpenSCENARIO for ADP, which
                               doesn't read .xosc -- see "ADP YAML export"
                               below
  export/report.py            trace summary export (txt / xml): scene
                               composition, braking/overtake events, issues
  export/batch_output.py      resolves + writes every export into output/,
                               mirroring the input corpus layout for
                               ADMA/annotation (used by every export path,
                               not just the batch action); stamps each
                               filename with the trace's provenance and
                               logs it to output/exports.jsonl
  variants.py                 generates "alternative scenario" Trace copies
                               by perturbing one surrounding vehicle's
                               trajectory at a time -- see "Alternative
                               scenarios (ODD variants)" below
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
  The GUI's Ego Odometry panel shows genuine forward/lateral vehicle-frame
  velocity, computed by rotating `(INS_Vel_Frame_Y, INS_Vel_Frame_X)` —
  i.e. `(East, North)` — into the vehicle frame using the pose's heading,
  the same rotation used to place annotation boxes; see `scene.py`. Sanity
  check on the (steady, near-zero-slip highway) sample traces: forward
  velocity matches total speed to within centimeters/second and lateral
  velocity stays under ~0.1 m/s throughout.
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

## OpenDRIVE generation

`export/road_geometry.py` builds the road model that `export/opendrive.py`
serializes to XML. Two independent signals feed it, trusted very
differently:

- **The reference line** always follows the ego's own recorded path, using
  its *real* IMU/GPS heading at each point rather than a heading derived
  from the straight-line direction between two samples. That lets the
  plan-view geometry be emitted as a sequence of constant-curvature `<arc>`
  segments (falling back to `<line>` where curvature is negligible)
  instead of a jointed polyline — curvature is computed directly as
  `heading change / arc length` between consecutive samples, so it has no
  dependency on annotation quality at all; it's the same trusted ego trace
  used throughout the rest of the tool.

- **Lane count and width**, in contrast, come from annotation data (the
  lane-marking polylines), which is sparser and can be noisy, gappy, or
  occluded. This is deliberately *not* trusted blindly — see "Why this
  needs to be conservative" below. The road is divided into 30m windows;
  each window's lane-marking points are projected onto the reference line
  (giving each point a road-relative `s`/`t`) and clustered laterally
  (points within 1.5m of each other are the same physical boundary,
  further apart is a different one). The two clusters bracketing the
  ego's own path (`t=0`) are its lane's edges; walking outward from there
  gives the full lane count and each lane's width. A window's estimate is
  used only if it passes *every* check:
  - at least 20 marking points contributed to it,
  - the resulting lane count and every lane's width are within realistic
    bounds (1-6 lanes, 2.5-4.5m each),
  - it agrees with vehicles' `obj_lane` labels ("EGO lane", "1st Right",
    ...) observed in that window, when there are enough to check,
  - it agrees with the annotation's own directly-authored
    `frame_meta.num_lanes` for that stretch, when available (sparser still,
    but a stronger signal than anything geometrically derived, since it
    isn't subject to the same occlusion/tracking noise),
  - the same *structure* (lane count on each side) persists for at least 2
    consecutive windows, so one noisy window can't fragment the road into
    a flickering sequence of spurious lane sections. This checks lane
    *counts*, not exact widths: two independently-estimated 30m windows of
    the same real lane essentially never land within centimeters of each
    other (real marking-detection noise is that large on its own), so an
    earlier version of this check required near-identical widths too --
    which made almost every genuinely-persistent stretch look like a
    one-off blip and silently discarded it in favor of whatever state
    preceded it. Observed on a real corpus trace where this erased
    annotation-derived lane data across roughly half the road, in turn
    leaving no left lane modeled for a lane that was genuinely there and
    putting a real vehicle several meters off-road.

  A window that fails any check falls back to the trace's overall majority
  declared lane count and a constant default width (3.5m) — the same
  behavior the generator always used to have. Consecutive windows with
  matching widths merge into a single OpenDRIVE `<laneSection>`; a
  persistent-but-noisy stretch instead survives as several adjacent
  sections, each with its own honest per-window numbers, rather than one
  fabricated constant or no annotation-derived section at all.

- **Where the road sits relative to the ego.** The reference line *is* the
  ego's path, but the ego drives in a lane, not down the road's centre —
  so emitting lanes only to the right of the reference line and a
  `<laneOffset>` of zero puts every vehicle beside the ego off the road in
  any viewer. Two things fix that:

  - The measured distance from the ego to its own lane's left edge is kept
    (rather than discarded after being used to bracket the ego's lane) and
    emitted as a `<laneOffset>` per lane section, shifting the road's
    centre line off the ego's path to where it actually belongs.
  - Lanes measured to the *left* of the ego are emitted as real `<left>`
    lanes with positive ids, in descending id order (outermost first), and
    the `<lanes>` element is written in the order the spec requires:
    every `<laneOffset>` first, then each `<laneSection>` with its
    `left` → `center` → `right` children.

  When no annotation-derived cross-section is available, the fallback
  splits the majority lane count around the ego using `frame_meta.ego_lane`
  — which is **1-based counting from the rightmost lane** (verified
  empirically: `sample1` has `ego_lane=2` of `num_lanes=2` with exactly one
  lane measured to its right; a middle-lane case like `sample2`'s is
  symmetric and can't disambiguate the convention on its own). Getting this
  backwards is not a subtle degradation — it moves the entire road to the
  wrong side of the traffic. Measured on the sample traces, on-road vehicle
  placement went from 89.6% → 96.2% (`sample1`) and 35.1% → 80.6%
  (`sample2`) once offsets, left lanes and the correct `ego_lane`
  convention were all in place.

  **Why this needs to be conservative**: a jaggedly-wrong road (an
  unbounded width spike from one bad frame, a phantom lane from an
  occlusion gap) is worse for a downstream simulator or planner than a
  smoothly-wrong one (constant width/count everywhere, today's original
  behavior) — so a low-confidence window degrading to the old constant-width
  fallback is the deliberately-chosen failure mode, not a shortcut. This
  was validated against real bugs, not hypothetically: an early version of
  this estimator, before the `frame_meta`/persistence checks existed,
  produced a false "lane count drops to 1" read on `sample1` that
  contradicted the trace's own constant `frame_meta.num_lanes=2` — caused
  by an assumption that the ego always drives at its lane's edge (it
  doesn't; it drives near lane center, with real boundaries on both
  sides). `tests/test_road_geometry.py` has a regression test asserting
  every annotation-derived section agrees with `frame_meta` wherever both
  exist, specifically to catch a recurrence of that bug.

- **Static objects** (traffic signs, reflective markers, highway
  accessories — whatever the annotation itself labeled) are placed into
  the road's `<objects>` element: one representative observation per
  object (they don't move, so any one observation's position/dimensions
  are as good as any other) projected onto the road as `s`/`t`. The
  mapping from the annotation's free-text type to ASAM's `e_objectType`
  enum is best-effort — there's no dedicated category for some of these
  (a delineator post, for instance) — so the original annotation label is
  always preserved in the object's `name` attribute regardless of how
  confident the `type` mapping is. Traffic-sign *meaning* (what a sign
  actually says) isn't attempted; the annotation doesn't carry that, and a
  fuller implementation would need ASAM's `<signals>` element with a real
  country-specific sign-code catalog instead.

### Online map enrichment (optional)

Everything above works entirely offline, using only the trace's own data —
that's still the default. Checking **Enrich with OpenStreetMap (online)**
next to the OpenDRIVE export button (or passing `?enrich=osm` to the
`/export/opendrive` endpoint directly) additionally looks up the trace's
road on [OpenStreetMap](https://www.openstreetmap.org) via the public
Overpass API before generating the file:

- The matched way's `name`/`ref` tag becomes the road's name in the
  generated `.xodr`, instead of the generic default.
- Its `lanes` tag becomes a fallback lane count for any stretch where the
  annotation itself has no trustworthy estimate — used exactly like the
  existing constant-lane-count fallback (see *OpenDRIVE generation*
  above), just a better-informed guess than a blind default when it's
  available. It never overrides a real annotation-derived estimate.
- Matching is a lightweight nearest-way comparison (which of the roads
  returned for the trace's GPS bounding box does the ego's own path
  actually run alongside), not full map-matching — good enough to pick
  "this highway" out of whatever else the query returned, not perfect in
  dense road networks.

**By default this has no effect on the live visualization or interactive
GUI performance.** Export enrichment is wired into exactly one place — the
scenario export endpoint — behind an off-by-default opt-in; nothing under
normal use (loading a trace, playback, validation, fixing) calls it,
ever. A slow or unavailable network never breaks an export either: any
failure (offline, timeout, DNS, a malformed response) is caught and
logged, and the export falls back to the exact offline result silently.
Successful lookups are cached to disk (`output/map_cache/`, keyed by the
trace's rounded GPS bounding box) so re-exporting the same trace or corpus
doesn't re-query Overpass every time — both for your own performance and
because Overpass is shared public infrastructure with fair-use
expectations.

OpenStreetMap has no lane-level boundary geometry (see the map-provider
comparison this was designed around, further up this file) — this
integration is scoped to what OSM actually has: road identity and coarse
attributes, not curb-level geometry. It's deliberately built behind a
small provider interface (`export/map_enrichment.py`'s
`MapEnrichmentProvider`) specifically so a HERE-backed provider (if the
richer HD Live Map product turns out to be available) can be added later
as a second implementation of that same interface, without changing
`road_geometry.py`, `opendrive.py`, or the API/GUI wiring at all.

#### Map overlay in the visualizer (also opt-in, also online)

Separately, once a trace is loaded, the **Map (OSM)** checkbox above the
canvas fetches the same kind of OpenStreetMap road data for that one trace
and draws it as blue background lines underneath the annotation-derived
lane markings and vehicles — a quick sanity check of how the recorded
lanes line up against the real road, not a replacement for either.

- It's per-trace and on demand only: nothing is fetched until you check
  the box, there's no batch/background version, and unchecking or
  switching to a different trace clears it (switching back re-fetches).
- Same never-block guarantee as export enrichment: a failed or
  unavailable fetch shows the reason next to the checkbox and leaves the
  box unchecked, it never breaks playback or the rest of the GUI.
- Backed by a new read-only endpoint,
  `GET /api/traces/{trace_id}/map_overlay?provider=osm`, which reuses
  `export/map_enrichment.py`'s provider abstraction and converts the
  returned road geometry into the same local (x, y) frame as the rest of
  the scene JSON, so the frontend only ever draws plain canvas
  coordinates.

## ADP YAML export (alternative to OpenSCENARIO)

ADP (Applied Intuition's "Simian" platform) doesn't read `.xosc` files, so
**ADP scenario (.scn.yaml)**, next to the OpenSCENARIO button, exports the
same fixed/predicted trace as ADP's own `.scn.yaml` scenario format instead
— `GET /api/traces/{trace_id}/export/adp_yaml`, writing to
`output/scenarios/<trace_id>/<trace_id>.scn.yaml`.

There's no public schema for this format — ADP's own docs live behind a
login this tool can't reach. `export/adp_yaml.py` was reverse-engineered
instead from real `.scn.yaml` files, including two ADP's own
log-extraction pipeline produced from real driving recordings (identifiable
by `metadata.comments` mentioning an "Originating simulation run"). Being
the closest analog to what this tool does — turning a real ADMA +
annotation log into a scenario — that pipeline's own pattern is what the
generator mirrors: ego and vehicle paths are resampled at a fixed ~15m arc
length into `pose_b_spline` waypoints paired with `ramp_velocity` phases,
vehicles that enter or leave the recording partway through get
`hide`/`until_true` wrapping timed to their actual first/last observation,
and everyone shares a generic `kinematic_bicycle` motion model, all matching
the sample files' own structure rather than being invented from scratch.
Coordinates are absolute UTM easting/northing: this tool's internal local
frame (meters, anchored at the trace's first GPS sample) is added to one
UTM point computed at that anchor, rather than re-projecting every point,
since the two agree to within the UTM grid convergence angle over a single
trace's length.

Two things in the output are unavoidably best-effort guesses, not derived
facts, and are worth checking before trusting a generated file:

- **`map.key`** — ADP resolves this against its own map registry, which
  this tool has no access to. Per how this project's OpenDRIVE export is
  actually used: importing a trace's `.xodr` into ADP registers a map keyed
  by the `.xodr` filename, so this defaults to the trace_id (matching
  `<trace_id>.xodr`) — pass a **Map key** in the GUI (or `map_key` on the
  endpoint) if the file was imported under a different name. Left at the
  default, the exported YAML also gets a `metadata.comments` note saying so,
  and the GUI's status line flags it the same way.
- **Vehicle visual assets** (`model.spectral_model_spec.spectral_model`) —
  ADP's asset catalog isn't accessible from here either.
  `_guess_vehicle_model` maps each vehicle's annotated type and dimensions
  onto a small set of generic assets actually seen in the sample files
  (`GENERIC_YARIS` for cars, `CONFIGURABLE_SEMI_TRUCK` for anything
  truck/trailer-like or longer than 9m — this one takes an exact
  `truck_length` parameter, so it fits the annotated length precisely
  regardless of the asset guess, `GENERIC_DAELIM_MOTORBIKE` for
  motorcycles/bikes), falling back to a plain untextured box
  (`model.static`, the same asset-agnostic shape OpenDRIVE static objects
  already use, and always what's used for pedestrians and for annotation
  static objects) whenever nothing in that small set is a plausible match.
  A wrong guess only affects the simulation's visuals, never the
  trajectory geometry — a one-line hand-edit in the output file.

## Vehicle point-of-view export

Both OpenSCENARIO and ADP scenario export take an optional **POV**
selector (in the GUI, above the OpenSCENARIO/ADP scenario buttons; on the
API, `pov_vehicle_id` on `/export/openscenario` and `/export/adp_yaml`).
Picking a vehicle re-roots the exported scenario from that vehicle's point
of view instead of the recorded ego:

- That vehicle's own recorded path becomes "Ego" in the file.
- The real ego is re-cast as a regular vehicle/obstacle entity.
- The export is truncated to that vehicle's own observed time window (its
  first through last annotated observation) — the only span for which its
  trajectory is actually known — and every other vehicle is filtered to
  the same window (one with no observations inside it is dropped
  entirely, rather than emitted as an entity that's never actually
  present).

The road itself (the companion `.xodr`) is reused completely unchanged —
its reference line still follows the real ego's own driven path regardless
of which agent is "Ego" in the scenario file, since the road geometry
doesn't depend on viewpoint. Written to a separate `<trace_id>_pov<id>`
file (`.xosc` / `.scn.yaml`) so it never overwrites the normal, ego's-own-
view export. Since a lot of vehicles in a real recording aren't present
for the whole clip, this is deliberately scoped to *that vehicle's own
window* rather than trying to backfill or extrapolate what it would have
seen outside it.

## Alternative scenarios (ODD variants)

The **Alternative Scenarios** panel (below Export) generates a small menu
of harder/different scenarios from the currently loaded trace by
perturbing one surrounding vehicle's already-recorded trajectory at a
time — the ego's own path and the road never change, only what nearby
traffic does. See `variants.py`'s module docstring for the full mechanics;
in short:

- **Generate 5 (presets)** tries five fixed, named, explainable variants,
  each targeting a real behavior this tool already detects (see *Trace
  summary report* below) and making it measurably harder:
  - **Harder cut-in** / **Delayed merge** — the nearest cut-in vehicle
    merges sooner-and-tighter, or later-and-looser.
  - **Closer following** — the lead vehicle in the ego's lane keeps a
    tighter gap for its whole observed window.
  - **Hard brake ahead** — the lead vehicle brakes hard partway through,
    closing the gap sharply and not fully recovering it.
  - **Tighter passing clearance** — the passing/overtaking vehicle stays
    laterally closer to the ego while going around.

  A preset that finds no matching vehicle in this particular trace (e.g.
  no cut-in ever happens) is skipped, not forced — the remaining slots are
  backfilled with randomized variants so a 5-variant request still returns
  5, where the trace has enough vehicles to support it.
- **Randomize** generates 5 random perturbations (random vehicle + random
  kind + random magnitude within safe bounds) instead, seeded for
  reproducibility from the API (`seed` on `/variants/generate`).
- Click a variant to preview it in the viewport (the titlebar shows which
  one you're looking at) and reveal its own export buttons — **Fixed
  Trace**, **OpenSCENARIO**, **ADP scenario** — writing to a path keyed by
  that variant so it never collides with the source trace's own exports
  or another variant's. Click the same variant again, or **← Back to
  original trace**, to return to the real trace.
- Each variant's issue count (shown next to it, when higher than the
  source trace's own) reflects a full re-validation of the perturbed
  trace — a "harder" variant that ends up genuinely colliding or going
  off-road is flagged the same way any other trace would be, which is
  often the point of generating it.

Every variant is a complete, independent copy of the trace (`Trace`
object) with global coordinates and validation re-run on it, so every
existing export path works on it completely unchanged — nothing in
`export/opendrive.py`, `export/openscenario.py`, `export/adp_yaml.py`, or
`export/report.py` needed to know variants exist at all.

Known simplification: only each vehicle's position (`x_rel`/`y_rel`) is
perturbed; its own recorded heading (`zrot`) isn't re-derived to match, so
a sharply time-compressed lateral move won't show a correspondingly
steeper heading. Good enough for a harder/different *positional* scenario,
not a substitute for a real vehicle dynamics model.

## Known limitations / scope (v1)

- **OpenDRIVE defaults to using only the trace itself** — a real map
  provider (OpenStreetMap today) is opt-in, not required: real lane-level
  geometry where the annotation supports it, a sane fallback where it
  doesn't. See *OpenDRIVE generation*
  below for how. Still not a replacement for a real HD map: the reference
  line runs along the ego's own driven path rather than precisely through
  the road's true center, and curvature is a sequence of constant-curvature
  arcs fit to real heading, not full clothoid continuity.
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
