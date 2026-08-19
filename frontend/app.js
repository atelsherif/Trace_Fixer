"use strict";

const state = {
  traceId: null,
  scene: null,
  timeS: 0,
  playing: false,
  playbackSpeed: 1,
  loop: false,
  lastFrameMs: null,
  camera: { zoom: 8, followEgo: true, followVehicleId: null, centerX: 0, centerY: 0, headingUp: true },
  drag: null,
  selectedVehicleId: null,
  showLanes: true,
  showStatic: false,
  showMapOverlay: false,
  mapOverlay: null,
  searchQuery: "",
  selectedTraceIds: new Set(),
  batchRunning: false,
};

const el = (id) => document.getElementById(id);
const canvas = () => el("viewport");

// ---------- API helpers ----------

async function apiGet(path) {
  const r = await fetch(path);
  if (!r.ok) throw new Error(`${path} -> ${r.status}`);
  return r.json();
}

async function apiPost(path, body) {
  const r = await fetch(path, {
    method: "POST",
    headers: body ? { "Content-Type": "application/json" } : undefined,
    body: body ? JSON.stringify(body) : undefined,
  });
  if (!r.ok) throw new Error(`${path} -> ${r.status}`);
  return r.json();
}

function setStatus(msg) {
  el("status-line").textContent = msg;
}

async function runExport(label, path) {
  setStatus(`Exporting ${label}…`);
  try {
    const data = await apiGet(path);
    let msg = `Saved ${label} to ${data.output_path}`;
    if (data.enrichment_requested) {
      if (data.enrichment) {
        msg += ` (enriched with ${data.enrichment})`;
      } else {
        msg += ` (${data.enrichment_requested} enrichment failed: ${data.enrichment_error || "unknown reason"}; used offline data)`;
      }
    }
    setStatus(msg);
  } catch (err) {
    setStatus(`Export failed: ${err.message}`);
  }
}

// ---------- Trace picker (searchable, scales to large corpora) ----------

const TRACE_PAGE_SIZE = 200;

async function queryTraces(q, offset = 0) {
  const params = new URLSearchParams({ limit: String(TRACE_PAGE_SIZE), offset: String(offset) });
  if (q) params.set("q", q);
  return apiGet(`/api/traces?${params.toString()}`);
}

async function initTracePicker() {
  const data = await queryTraces("", 0);
  renderTraceListbox(data.trace_ids, data.total, 0);
  if (data.trace_ids.length > 0) {
    await loadTrace(data.trace_ids[0]);
  } else {
    el("trace-picker-label").textContent = "No traces";
  }
}

let lastListedIds = [];
let lastListedTotal = 0;
let lastListedOffset = 0;

function renderTraceListbox(ids, total, offset = 0) {
  lastListedIds = ids;
  lastListedTotal = total;
  lastListedOffset = offset;
  const box = el("trace-listbox");
  box.innerHTML = "";
  if (ids.length === 0) {
    const empty = document.createElement("div");
    empty.className = "trace-listbox-empty";
    empty.textContent = "No matching traces.";
    box.appendChild(empty);
  }
  for (const id of ids) {
    const item = document.createElement("div");
    item.className = "trace-listbox-item" + (id === state.traceId ? " active" : "");

    const checkbox = document.createElement("input");
    checkbox.type = "checkbox";
    checkbox.className = "trace-listbox-checkbox";
    checkbox.checked = state.selectedTraceIds.has(id);
    checkbox.title = "Select for batch fix + predict";
    checkbox.addEventListener("click", (e) => e.stopPropagation());
    checkbox.addEventListener("change", () => {
      if (checkbox.checked) state.selectedTraceIds.add(id); else state.selectedTraceIds.delete(id);
      updateBatchControl();
    });

    const name = document.createElement("span");
    name.className = "trace-listbox-name";
    name.textContent = id;

    item.appendChild(checkbox);
    item.appendChild(name);
    item.addEventListener("click", async () => {
      closeTracePicker();
      await loadTrace(id);
    });
    box.appendChild(item);
  }
  const shownStart = ids.length ? offset + 1 : 0;
  const shownEnd = offset + ids.length;
  el("trace-picker-footer").textContent =
    total > 0 ? `Showing ${shownStart}–${shownEnd} of ${total} trace(s).` : "No matching traces.";
  const pageCount = Math.max(1, Math.ceil(total / TRACE_PAGE_SIZE));
  const pageNum = Math.floor(offset / TRACE_PAGE_SIZE) + 1;
  el("trace-page-label").textContent = pageCount > 1 ? `Page ${pageNum} of ${pageCount}` : "";
  el("trace-page-prev").disabled = offset <= 0;
  el("trace-page-next").disabled = shownEnd >= total;
}

function updateBatchControl() {
  const n = state.selectedTraceIds.size;
  el("batch-selected-count").textContent = `${n} selected`;
  el("batch-run").disabled = n === 0 || state.batchRunning;
}

function openTracePicker() {
  el("trace-picker-panel").classList.remove("hidden");
  el("trace-search").focus();
}
function closeTracePicker() {
  el("trace-picker-panel").classList.add("hidden");
}

let traceSearchDebounce = null;
function onTraceSearchInput(value) {
  state.searchQuery = value.trim();
  clearTimeout(traceSearchDebounce);
  traceSearchDebounce = setTimeout(async () => {
    const data = await queryTraces(state.searchQuery, 0);
    renderTraceListbox(data.trace_ids, data.total, 0);
  }, 150);
}

async function loadTrace(traceId) {
  state.traceId = traceId;
  el("trace-picker-label").textContent = traceId;
  el("viewport-trace-name").textContent = traceId;
  const scene = await apiGet(`/api/traces/${traceId}/scene`);
  applyScene(scene);
  state.timeS = 0;
  state.selectedVehicleId = null;
  state.camera.followEgo = true;
  state.camera.followVehicleId = null;
  state.mapOverlay = null;
  state.showMapOverlay = false;
  el("show-map-overlay").checked = false;
  el("map-overlay-status").textContent = "";
  setStatus(`Loaded ${traceId}: ${scene.vehicles.length} vehicles, ${scene.duration_s.toFixed(1)}s.`);
}

async function stepTrace(direction) {
  if (!state.traceId) return;
  const params = new URLSearchParams({ direction });
  if (state.searchQuery) params.set("q", state.searchQuery);
  try {
    const data = await apiGet(`/api/traces/${state.traceId}/neighbor?${params.toString()}`);
    if (data.trace_id !== state.traceId) await loadTrace(data.trace_id);
  } catch (err) {
    setStatus(`Could not navigate: ${err.message}`);
  }
}

async function runBatch() {
  const ids = Array.from(state.selectedTraceIds);
  if (!ids.length || state.batchRunning) return;
  state.batchRunning = true;
  updateBatchControl();
  const results = [];
  for (let i = 0; i < ids.length; i++) {
    const id = ids[i];
    el("batch-status").textContent = `Fixing + predicting ${i + 1}/${ids.length}: ${id}…`;
    try {
      const r = await fetch(`/api/traces/${id}/batch_fix_predict`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ horizon_s: 4.0, step_s: 0.2 }),
      });
      const data = await r.json();
      if (!r.ok) throw new Error(data.detail || `HTTP ${r.status}`);
      results.push(data);
    } catch (err) {
      results.push({ trace_id: id, error: err.message });
    }
  }
  state.batchRunning = false;
  updateBatchControl();

  const failed = results.filter((r) => r.error);
  const ok = results.filter((r) => !r.error);
  const totalBefore = ok.reduce((s, r) => s + r.before_issue_count, 0);
  const totalAfter = ok.reduce((s, r) => s + r.after_issue_count, 0);
  el("batch-status").textContent =
    `Done: ${ok.length}/${ids.length} trace(s) processed, issues ${totalBefore} → ${totalAfter}. ` +
    `Corrected files written to output/adma/ADMA/… and output/annotations/Annotations/….` +
    (failed.length ? ` ${failed.length} failed: ${failed.map((f) => f.trace_id).join(", ")}` : "");
  setStatus(el("batch-status").textContent);

  if (state.traceId && state.selectedTraceIds.has(state.traceId)) {
    await loadTrace(state.traceId); // refresh the currently displayed trace if it was included
  }
}

function precomputeEgoDistance(path) {
  // Cumulative arc length along the ego path, computed once per scene load
  // (not per frame) so the odometry panel's "distance traveled" is a cheap
  // O(1) interpolated lookup like every other ego field -- see egoAt().
  let dist = 0;
  path[0].dist_m = 0;
  for (let i = 1; i < path.length; i++) {
    const a = path[i - 1], b = path[i];
    dist += Math.hypot(b.x - a.x, b.y - a.y);
    b.dist_m = dist;
  }
}

function applyScene(scene) {
  state.scene = scene;
  if (scene.ego.path.length) precomputeEgoDistance(scene.ego.path);
  el("timeline").max = scene.duration_s.toFixed(3);
  if (state.timeS > scene.duration_s) state.timeS = 0;
  renderIssueList();
  renderVehicleList();
  renderEventList();
  updateTimeLabel();
}

// ---------- Interpolation ----------

function wrapDeg(a) {
  return ((a + 180) % 360 + 360) % 360 - 180;
}

function lerpHeadingDeg(h0, h1, frac) {
  return h0 + wrapDeg(h1 - h0) * frac;
}

function interpAtTime(samples, t, tKey, getters) {
  // samples: sorted array by tKey. Returns interpolated object or null if out of range.
  if (!samples.length) return null;
  if (t <= samples[0][tKey]) return samples[0];
  if (t >= samples[samples.length - 1][tKey]) return samples[samples.length - 1];
  let lo = 0, hi = samples.length - 1;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (samples[mid][tKey] <= t) lo = mid; else hi = mid;
  }
  const a = samples[lo], b = samples[hi];
  const span = b[tKey] - a[tKey];
  const frac = span <= 0 ? 0 : (t - a[tKey]) / span;
  return getters(a, b, frac);
}

function egoAt(t) {
  const path = state.scene.ego.path;
  return interpAtTime(path, t, "t_s", (a, b, f) => ({
    x: a.x + (b.x - a.x) * f,
    y: a.y + (b.y - a.y) * f,
    heading_deg: lerpHeadingDeg(a.heading_deg, b.heading_deg, f),
    speed_mps: a.speed_mps + (b.speed_mps - a.speed_mps) * f,
    v_fwd_mps: a.v_fwd_mps + (b.v_fwd_mps - a.v_fwd_mps) * f,
    v_lat_mps: a.v_lat_mps + (b.v_lat_mps - a.v_lat_mps) * f,
    v_vert_mps: a.v_vert_mps + (b.v_vert_mps - a.v_vert_mps) * f,
    lat: a.lat + (b.lat - a.lat) * f,
    lon: a.lon + (b.lon - a.lon) * f,
    dist_m: a.dist_m + (b.dist_m - a.dist_m) * f,
  }));
}

function vehicleAt(vehicle, t) {
  const obs = vehicle.observations;
  if (!obs.length) return null;
  if (t < obs[0].t_s - 0.5 || t > obs[obs.length - 1].t_s + 0.5) return null;
  return interpAtTime(obs, t, "t_s", (a, b, f) => ({
    x: a.x + (b.x - a.x) * f,
    y: a.y + (b.y - a.y) * f,
    heading_deg: lerpHeadingDeg(a.heading_deg, b.heading_deg, f),
    length: a.length + (b.length - a.length) * f,
    width: a.width + (b.width - a.width) * f,
    synthetic: a.synthetic || b.synthetic,
    obj_lane: a.obj_lane,
  }));
}

// ---------- Rendering ----------

function resizeCanvas() {
  const c = canvas();
  const rect = c.parentElement.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  c.width = rect.width * dpr;
  c.height = rect.height * dpr;
  c.style.width = rect.width + "px";
  c.style.height = rect.height + "px";
}

function activeIssuesAt(t) {
  if (!state.scene) return new Set();
  const ids = new Set();
  for (const issue of state.scene.issues) {
    if (issue.vehicle_id != null && t >= issue.t_start_s - 0.3 && t <= issue.t_end_s + 0.3) {
      ids.add(issue.vehicle_id);
    }
  }
  return ids;
}

function draw() {
  const c = canvas();
  const ctx = c.getContext("2d");
  const dpr = window.devicePixelRatio || 1;
  ctx.setTransform(1, 0, 0, 1, 0, 0);
  ctx.clearRect(0, 0, c.width, c.height);
  if (!state.scene) return;

  const ego = egoAt(state.timeS);
  updateGpsReadout(ego);
  updateOdometry(ego);
  if (state.camera.followVehicleId != null) {
    const followed = state.scene.vehicles.find((v) => v.id === state.camera.followVehicleId);
    const v = followed ? vehicleAt(followed, state.timeS) : null;
    if (v) {
      state.camera.centerX = v.x;
      state.camera.centerY = v.y;
    }
  } else if (state.camera.followEgo && ego) {
    state.camera.centerX = ego.x;
    state.camera.centerY = ego.y;
  }

  const zoom = state.camera.zoom * dpr;
  ctx.save();
  ctx.translate(c.width / 2, c.height * 0.68);
  ctx.scale(zoom, -zoom);
  if (state.camera.headingUp && ego) {
    ctx.rotate(Math.PI / 2 - (ego.heading_deg * Math.PI) / 180);
  }
  ctx.translate(-state.camera.centerX, -state.camera.centerY);

  const lineWidthWorld = 1 / zoom;

  if (state.showMapOverlay && state.mapOverlay) drawMapOverlay(ctx, state.mapOverlay.ways, lineWidthWorld);

  if (state.showLanes) drawLines(ctx, state.scene.lane_markings, "#4a5568", lineWidthWorld, false);
  drawLines(ctx, state.scene.border_lines, "#c98a3c", lineWidthWorld * 1.6, true);

  if (state.showStatic) drawStatic(ctx, state.scene.static_objects, lineWidthWorld);

  const flagged = activeIssuesAt(state.timeS);
  for (const vehicle of state.scene.vehicles) {
    const v = vehicleAt(vehicle, state.timeS);
    if (!v) continue;
    const isFlagged = flagged.has(vehicle.id);
    const isSelected = state.selectedVehicleId === vehicle.id;
    drawBox(ctx, v.x, v.y, v.heading_deg, v.length, v.width, {
      fill: v.synthetic ? "#b98cf233" : isFlagged ? "#ff5a5a55" : "#6ee7a855",
      stroke: v.synthetic ? "#b98cf2" : isFlagged ? "#ff5a5a" : "#6ee7a8",
      dashed: v.synthetic,
      lineWidth: (isSelected ? 3 : 1.5) * lineWidthWorld,
    });
    if (isSelected) {
      ctx.save();
      ctx.strokeStyle = "#4da3ff";
      ctx.lineWidth = 2.5 * lineWidthWorld;
      ctx.beginPath();
      ctx.arc(v.x, v.y, Math.max(v.length, v.width) * 0.75, 0, Math.PI * 2);
      ctx.stroke();
      ctx.restore();
    }
  }

  if (ego) {
    drawBox(ctx, ego.x, ego.y, ego.heading_deg, state.scene.ego.length, state.scene.ego.width, {
      fill: "#4da3ff88",
      stroke: "#4da3ff",
      lineWidth: 2 * lineWidthWorld,
    });
  }

  ctx.restore();
}

function drawLines(ctx, lineGroups, color, lineWidth, dashed) {
  ctx.save();
  ctx.strokeStyle = color;
  ctx.lineWidth = lineWidth;
  if (dashed) ctx.setLineDash([lineWidth * 4, lineWidth * 3]);
  for (const group of lineGroups) {
    for (const snap of group.snapshots) {
      if (snap.points.length < 2) continue;
      ctx.beginPath();
      ctx.moveTo(snap.points[0][0], snap.points[0][1]);
      for (let i = 1; i < snap.points.length; i++) ctx.lineTo(snap.points[i][0], snap.points[i][1]);
      ctx.stroke();
    }
  }
  ctx.restore();
}

function drawMapOverlay(ctx, ways, lineWidthWorld) {
  // Background context from an online map provider (see map_enrichment.py) --
  // drawn under the annotation-derived lane markings/border lines so the
  // ground truth this tool is actually validating always stays on top.
  ctx.save();
  ctx.strokeStyle = "#3d6fa8";
  ctx.lineWidth = lineWidthWorld * 2;
  ctx.setLineDash([]);
  for (const way of ways) {
    if (way.points.length < 2) continue;
    ctx.beginPath();
    ctx.moveTo(way.points[0][0], way.points[0][1]);
    for (let i = 1; i < way.points.length; i++) ctx.lineTo(way.points[i][0], way.points[i][1]);
    ctx.stroke();
  }
  ctx.restore();
}

function drawStatic(ctx, objects, lineWidthWorld) {
  ctx.save();
  ctx.fillStyle = "#5a6478";
  for (const obj of objects) {
    for (const o of obj.observations) {
      ctx.beginPath();
      ctx.arc(o.x, o.y, Math.max(0.3, o.width / 2), 0, Math.PI * 2);
      ctx.fill();
    }
  }
  ctx.restore();
}

function drawBox(ctx, x, y, headingDeg, length, width, opts) {
  ctx.save();
  ctx.translate(x, y);
  ctx.rotate((headingDeg * Math.PI) / 180);
  ctx.fillStyle = opts.fill;
  ctx.strokeStyle = opts.stroke;
  ctx.lineWidth = opts.lineWidth;
  if (opts.dashed) ctx.setLineDash([opts.lineWidth * 3, opts.lineWidth * 2]);
  ctx.beginPath();
  ctx.rect(-length / 2, -width / 2, length, width);
  ctx.fill();
  ctx.stroke();
  // forward-direction nose marker
  ctx.beginPath();
  ctx.moveTo(length / 2, 0);
  ctx.lineTo(length / 2 - width * 0.4, width * 0.3);
  ctx.lineTo(length / 2 - width * 0.4, -width * 0.3);
  ctx.closePath();
  ctx.fillStyle = opts.stroke;
  ctx.fill();
  ctx.restore();
}

// ---------- GPS readout ----------

let lastGpsText = "";

function updateGpsReadout(ego) {
  if (!ego) return;
  lastGpsText = `${ego.lat.toFixed(6)}, ${ego.lon.toFixed(6)}`;
  if (el("gps-readout").classList.contains("copied")) return; // "Copied: ..." feedback showing -- don't stomp it
  el("gps-readout-text").textContent = `Lat ${ego.lat.toFixed(6)}, Lon ${ego.lon.toFixed(6)}`;
}

// ---------- Ego odometry panel ----------

const COMPASS_POINTS = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"];

function compassLabel(bearingDeg) {
  const idx = Math.round(bearingDeg / 45) % 8;
  return COMPASS_POINTS[idx];
}

// ego.heading_deg is the math-convention yaw used for canvas rotation
// (CCW from East -- see drawBox/heading-up camera logic), not a compass
// bearing. Converting back: yaw_deg = (90 + compass_deg) % 360 (see
// geo/transform.py's docstring), so compass_deg = (450 - yaw_deg) % 360.
function yawToCompassBearing(yawDeg) {
  return ((450 - yawDeg) % 360 + 360) % 360;
}

function updateOdometry(ego) {
  el("odo-time").textContent = `${state.timeS.toFixed(2)} s`;
  if (!ego) return;
  el("odo-speed").textContent = `${ego.speed_mps.toFixed(1)} m/s (${(ego.speed_mps * 3.6).toFixed(0)} km/h)`;
  el("odo-velocity").textContent =
    `${ego.v_fwd_mps.toFixed(1)} / ${ego.v_lat_mps.toFixed(1)} / ${ego.v_vert_mps.toFixed(1)} m/s`;
  const bearing = yawToCompassBearing(ego.heading_deg);
  el("odo-heading").textContent = `${bearing.toFixed(0)}° ${compassLabel(bearing)}`;
  const distM = ego.dist_m || 0;
  el("odo-distance").textContent = distM >= 1000 ? `${(distM / 1000).toFixed(2)} km` : `${distM.toFixed(0)} m`;
}

let copyFeedbackTimeout = null;

async function copyGpsReadout() {
  if (!lastGpsText) return;
  try {
    await navigator.clipboard.writeText(lastGpsText);
  } catch {
    return; // clipboard access denied/unavailable -- fail silently, nothing else to do
  }
  const btn = el("gps-readout");
  btn.classList.add("copied");
  el("gps-readout-text").textContent = `Copied: ${lastGpsText}`;
  clearTimeout(copyFeedbackTimeout);
  // the next animation frame's updateGpsReadout() call naturally restores
  // the live text once "copied" no longer needs to be shown -- no need to
  // remember/restore the old text here, and it'd be stale if we did (the
  // trace may have kept playing while the feedback was showing).
  copyFeedbackTimeout = setTimeout(() => btn.classList.remove("copied"), 1200);
}

// ---------- Issue list ----------

function renderIssueList() {
  const list = el("issue-list");
  list.innerHTML = "";
  const issues = state.scene.issues;
  el("issue-count").textContent = issues.length;
  if (!issues.length) {
    const li = document.createElement("li");
    li.className = "issue-empty";
    li.textContent = "No issues found. Run validation to check this trace.";
    list.appendChild(li);
    return;
  }
  for (const issue of issues) {
    const li = document.createElement("li");
    li.className = `issue-item sev-${issue.severity}${issue.fixed ? " fixed" : ""}`;
    const meta = document.createElement("div");
    meta.className = "issue-meta";
    meta.textContent = `${issue.category} · ${issue.severity}${issue.vehicle_id != null ? " · veh " + issue.vehicle_id : " · ego"}`;
    const desc = document.createElement("div");
    desc.textContent = `${issue.description} (t=${issue.t_start_s.toFixed(1)}s)`;
    li.appendChild(meta);
    li.appendChild(desc);
    li.addEventListener("click", () => {
      state.timeS = Math.max(0, issue.t_start_s);
      state.selectedVehicleId = issue.vehicle_id;
      state.camera.followEgo = issue.vehicle_id == null;
      state.camera.followVehicleId = null;
      setPlaying(false);
      updateTimeLabel();
      renderVehicleList();
      draw();
    });
    list.appendChild(li);
  }
}

// ---------- Vehicle list ----------

function renderVehicleList() {
  const list = el("vehicle-list");
  list.innerHTML = "";
  const vehicles = state.scene.vehicles;
  el("vehicle-count").textContent = vehicles.length;
  if (!vehicles.length) {
    const li = document.createElement("li");
    li.className = "issue-empty";
    li.textContent = "No vehicles in this scene.";
    list.appendChild(li);
    return;
  }
  for (const vehicle of vehicles) {
    const obs = vehicle.observations;
    const li = document.createElement("li");
    li.className = `issue-item vehicle-item${state.selectedVehicleId === vehicle.id ? " selected" : ""}`;
    const meta = document.createElement("div");
    meta.className = "issue-meta";
    meta.textContent = `veh ${vehicle.id} · ${vehicle.obj_type}`;
    const desc = document.createElement("div");
    desc.textContent = obs.length
      ? `${obs[0].t_s.toFixed(1)}s – ${obs[obs.length - 1].t_s.toFixed(1)}s`
      : "no observations";
    li.appendChild(meta);
    li.appendChild(desc);
    li.addEventListener("click", () => {
      if (state.selectedVehicleId === vehicle.id) {
        // clicking the already-selected vehicle again deselects it and
        // hands the camera back to following the ego
        state.selectedVehicleId = null;
        state.camera.followVehicleId = null;
        state.camera.followEgo = true;
      } else {
        state.selectedVehicleId = vehicle.id;
        state.camera.followEgo = false;
        state.camera.followVehicleId = vehicle.id;
        if (obs.length) state.timeS = Math.max(0, obs[0].t_s);
        setPlaying(true);
      }
      updateTimeLabel();
      renderVehicleList();
      draw();
    });
    list.appendChild(li);
  }
}

// ---------- Event list ----------

function renderEventList() {
  const list = el("event-list");
  list.innerHTML = "";
  const events = state.scene.events;
  el("event-count").textContent = events.length;
  if (!events.length) {
    const li = document.createElement("li");
    li.className = "issue-empty";
    li.textContent = "No notable events detected in this trace.";
    list.appendChild(li);
    return;
  }
  for (const event of events) {
    const li = document.createElement("li");
    li.className = "issue-item event-item";
    const meta = document.createElement("div");
    meta.className = "issue-meta";
    meta.textContent = `${humanizeTag(event.type)}${event.vehicle_id != null ? " · veh " + event.vehicle_id : " · ego"}`;
    const desc = document.createElement("div");
    desc.textContent = `${event.description} (t=${event.t_start_s.toFixed(1)}s)`;
    li.appendChild(meta);
    li.appendChild(desc);
    li.addEventListener("click", () => {
      state.timeS = Math.max(0, event.t_start_s);
      state.selectedVehicleId = event.vehicle_id;
      state.camera.followEgo = event.vehicle_id == null;
      state.camera.followVehicleId = event.vehicle_id;
      setPlaying(false);
      updateTimeLabel();
      renderVehicleList();
      draw();
    });
    list.appendChild(li);
  }
}

// ---------- Timeline / playback ----------

function updateTimeLabel() {
  const dur = state.scene ? state.scene.duration_s : 0;
  el("time-label").textContent = `${state.timeS.toFixed(2)} / ${dur.toFixed(2)} s`;
  el("timeline").value = state.timeS;
  updatePlayButtonIcon();
}

function isAtEnd() {
  return !!state.scene && state.timeS >= state.scene.duration_s - 1e-6;
}

function updatePlayButtonIcon() {
  const btn = el("play-pause");
  if (state.playing) {
    btn.innerHTML = "&#10074;&#10074;"; // pause
    btn.title = "Pause (Space)";
  } else if (isAtEnd()) {
    btn.innerHTML = "&#8635;"; // replay
    btn.title = "Replay from the start (Space)";
  } else {
    btn.innerHTML = "&#9654;"; // play
    btn.title = "Play (Space)";
  }
}

function setPlaying(playing) {
  state.playing = playing;
  state.lastFrameMs = null;
  updatePlayButtonIcon();
}

function togglePlayPause() {
  if (!state.playing && isAtEnd()) {
    state.timeS = 0; // pressing play/replay after the trace finished starts over
  }
  setPlaying(!state.playing);
  updateTimeLabel();
}

function stepTime(deltaS) {
  if (!state.scene) return;
  state.timeS = Math.max(0, Math.min(state.scene.duration_s, state.timeS + deltaS));
  setPlaying(false);
  updateTimeLabel();
  draw();
}

function tick(nowMs) {
  if (state.playing && state.scene) {
    if (state.lastFrameMs != null) {
      const dt = (nowMs - state.lastFrameMs) / 1000;
      state.timeS += dt * state.playbackSpeed;
      if (state.timeS > state.scene.duration_s) {
        if (state.loop) {
          state.timeS = state.timeS % state.scene.duration_s;
        } else {
          state.timeS = state.scene.duration_s;
          setPlaying(false);
        }
      }
    }
    state.lastFrameMs = nowMs;
    updateTimeLabel();
  }
  draw();
  requestAnimationFrame(tick);
}

// ---------- Wiring ----------

function wireControls() {
  window.addEventListener("resize", () => { resizeCanvas(); draw(); });

  // -- trace picker --
  el("trace-picker-btn").addEventListener("click", () => {
    const panel = el("trace-picker-panel");
    if (panel.classList.contains("hidden")) openTracePicker(); else closeTracePicker();
  });
  el("trace-search").addEventListener("input", (e) => onTraceSearchInput(e.target.value));
  document.addEventListener("click", (e) => {
    const picker = document.querySelector(".trace-picker");
    if (picker && !picker.contains(e.target)) closeTracePicker();
    const scan = document.querySelector(".scan-picker");
    if (scan && !scan.contains(e.target)) el("scan-panel").classList.add("hidden");
  });

  el("trace-prev").addEventListener("click", () => stepTrace("prev"));
  el("trace-next").addEventListener("click", () => stepTrace("next"));

  el("trace-page-prev").addEventListener("click", async () => {
    const offset = Math.max(0, lastListedOffset - TRACE_PAGE_SIZE);
    const data = await queryTraces(state.searchQuery, offset);
    renderTraceListbox(data.trace_ids, data.total, offset);
  });
  el("trace-page-next").addEventListener("click", async () => {
    const offset = lastListedOffset + TRACE_PAGE_SIZE;
    const data = await queryTraces(state.searchQuery, offset);
    renderTraceListbox(data.trace_ids, data.total, offset);
  });

  el("batch-select-shown").addEventListener("click", () => {
    for (const id of lastListedIds) state.selectedTraceIds.add(id);
    renderTraceListbox(lastListedIds, lastListedTotal, lastListedOffset);
    updateBatchControl();
  });
  el("batch-clear-selection").addEventListener("click", () => {
    state.selectedTraceIds.clear();
    renderTraceListbox(lastListedIds, lastListedTotal, lastListedOffset);
    updateBatchControl();
  });
  el("batch-run").addEventListener("click", runBatch);

  // -- scan directory --
  el("scan-btn").addEventListener("click", () => {
    el("scan-panel").classList.toggle("hidden");
    if (!el("scan-panel").classList.contains("hidden")) el("scan-path").focus();
  });
  el("scan-run").addEventListener("click", runScan);
  el("scan-path").addEventListener("keydown", (e) => { if (e.key === "Enter") runScan(); });

  el("scan-browse-btn").addEventListener("click", () => {
    const panel = el("dir-browser");
    if (panel.classList.contains("hidden")) openDirBrowser(); else panel.classList.add("hidden");
  });
  el("dir-browser-select").addEventListener("click", () => {
    if (dirBrowserPath) el("scan-path").value = dirBrowserPath;
    el("dir-browser").classList.add("hidden");
  });
  el("dir-browser-cancel").addEventListener("click", () => el("dir-browser").classList.add("hidden"));

  el("scan-batch-catalog").addEventListener("click", () => runBatchAll("catalog"));
  el("scan-batch-fix").addEventListener("click", () => runBatchAll("fix"));
  el("scan-batch-fix-catalog").addEventListener("click", () => runBatchAll("fix_catalog"));

  // -- catalog --
  el("catalog-btn").addEventListener("click", openCatalog);
  el("catalog-close").addEventListener("click", closeCatalog);
  el("catalog-modal").addEventListener("click", (e) => {
    if (e.target.id === "catalog-modal") closeCatalog();
  });
  window.addEventListener("keydown", (e) => {
    if (e.key === "Escape" && !el("catalog-modal").classList.contains("hidden")) closeCatalog();
  });
  el("catalog-search").addEventListener("input", (e) => {
    clearTimeout(catalogSearchDebounce);
    catalogSearchDebounce = setTimeout(() => {
      catalogState.q = e.target.value.trim();
      loadCatalogPage(0);
    }, 150);
  });
  el("catalog-page-prev").addEventListener("click", () => {
    loadCatalogPage(Math.max(0, catalogState.offset - CATALOG_PAGE_SIZE));
  });
  el("catalog-page-next").addEventListener("click", () => {
    loadCatalogPage(catalogState.offset + CATALOG_PAGE_SIZE);
  });

  // -- playback --
  el("play-pause").addEventListener("click", togglePlayPause);
  el("restart").addEventListener("click", () => {
    state.timeS = 0;
    updateTimeLabel();
    draw();
    setPlaying(true);
  });
  el("step-back").addEventListener("click", () => stepTime(-1));
  el("step-forward").addEventListener("click", () => stepTime(1));
  el("loop-toggle").addEventListener("change", (e) => { state.loop = e.target.checked; });

  window.addEventListener("keydown", (e) => {
    const tag = (document.activeElement && document.activeElement.tagName) || "";
    if (tag === "INPUT" || tag === "SELECT" || tag === "TEXTAREA") return;
    if (e.code === "Space") { e.preventDefault(); togglePlayPause(); }
    else if (e.code === "ArrowLeft") { e.preventDefault(); stepTime(-1); }
    else if (e.code === "ArrowRight") { e.preventDefault(); stepTime(1); }
    else if (e.code === "Home") { e.preventDefault(); state.timeS = 0; setPlaying(false); updateTimeLabel(); draw(); }
  });

  el("timeline").addEventListener("input", (e) => {
    state.timeS = parseFloat(e.target.value);
    state.camera.followEgo = true;
    state.camera.followVehicleId = null;
    setPlaying(true);
    updateTimeLabel();
    draw();
  });

  el("playback-speed").addEventListener("change", (e) => {
    state.playbackSpeed = parseFloat(e.target.value);
  });

  el("zoom-in").addEventListener("click", () => { state.camera.zoom *= 1.3; draw(); });
  el("zoom-out").addEventListener("click", () => { state.camera.zoom /= 1.3; draw(); });
  el("recenter").addEventListener("click", () => {
    state.camera.followEgo = true;
    state.camera.followVehicleId = null;
    draw();
  });
  el("heading-up").addEventListener("change", (e) => { state.camera.headingUp = e.target.checked; draw(); });
  el("gps-readout").addEventListener("click", copyGpsReadout);
  el("show-lanes").addEventListener("change", (e) => { state.showLanes = e.target.checked; draw(); });
  el("show-static").addEventListener("change", (e) => { state.showStatic = e.target.checked; draw(); });
  el("show-map-overlay").addEventListener("change", async (e) => {
    const checked = e.target.checked;
    const statusEl = el("map-overlay-status");
    if (!checked) {
      state.showMapOverlay = false;
      statusEl.textContent = "";
      draw();
      return;
    }
    if (!state.traceId) {
      e.target.checked = false;
      return;
    }
    if (state.mapOverlay) {
      state.showMapOverlay = true;
      draw();
      return;
    }
    statusEl.textContent = "Fetching map…";
    try {
      const data = await apiGet(`/api/traces/${state.traceId}/map_overlay?provider=osm`);
      if (data.error) {
        e.target.checked = false;
        statusEl.textContent = `Map unavailable: ${data.error}`;
        return;
      }
      state.mapOverlay = data;
      state.showMapOverlay = true;
      statusEl.textContent = `${data.ways.length} road(s) from ${data.provider}.`;
      draw();
    } catch (err) {
      e.target.checked = false;
      statusEl.textContent = `Map unavailable: ${err.message || err}`;
    }
  });

  const c = canvas();
  c.addEventListener("wheel", (e) => {
    e.preventDefault();
    const factor = e.deltaY < 0 ? 1.1 : 0.9;
    state.camera.zoom = Math.max(0.5, Math.min(60, state.camera.zoom * factor));
    draw();
  }, { passive: false });

  c.addEventListener("mousedown", (e) => {
    state.drag = { x: e.clientX, y: e.clientY, cx: state.camera.centerX, cy: state.camera.centerY };
    state.camera.followEgo = false;
    state.camera.followVehicleId = null;
  });
  window.addEventListener("mousemove", (e) => {
    if (!state.drag) return;
    const dpr = window.devicePixelRatio || 1;
    const dx = (e.clientX - state.drag.x) * dpr;
    const dy = (e.clientY - state.drag.y) * dpr;
    const zoom = state.camera.zoom * dpr;
    // Note: this ignores heading-up rotation for simplicity; drag panning is
    // most useful in north-up mode.
    state.camera.centerX = state.drag.cx - dx / zoom;
    state.camera.centerY = state.drag.cy + dy / zoom;
    draw();
  });
  window.addEventListener("mouseup", () => { state.drag = null; });

  el("btn-validate").addEventListener("click", async () => {
    setStatus("Running validation…");
    const res = await apiPost(`/api/traces/${state.traceId}/validate`);
    applyScene(res.scene);
    setStatus(`Validation found ${res.issue_count} issue(s).`);
  });

  el("btn-predict").addEventListener("click", async () => {
    setStatus("Predicting trajectories before/after the sensor FOV…");
    const res = await apiPost(`/api/traces/${state.traceId}/predict`, { horizon_s: 4.0, step_s: 0.2 });
    applyScene(res.scene);
    const perVehicle = Object.entries(res.added).map(([vid, dirs]) => {
      const parts = Object.keys(dirs);
      return `veh ${vid} (${parts.join(" + ")})`;
    });
    setStatus(perVehicle.length ? `Added predictions: ${perVehicle.join(", ")}.` : "No vehicles needed prediction.");
  });

  el("btn-clear-predict").addEventListener("click", async () => {
    const res = await apiPost(`/api/traces/${state.traceId}/predict/clear`);
    applyScene(res.scene);
    setStatus("Cleared predicted segments.");
  });

  el("btn-fix").addEventListener("click", async () => {
    setStatus("Applying fixes…");
    const res = await apiPost(`/api/traces/${state.traceId}/fix`);
    applyScene(res.scene);
    setStatus(res.summary.length ? res.summary.join("\n") : "No fixes were needed.");
  });

  el("btn-reset").addEventListener("click", async () => {
    const res = await apiPost(`/api/traces/${state.traceId}/reset`);
    applyScene(res.scene);
    setStatus("Trace reset to original files.");
  });

  el("sync-offset").addEventListener("change", async (e) => {
    const ms = parseFloat(e.target.value);
    el("sync-offset-value").textContent = ms;
    setStatus("Applying sync offset…");
    const res = await apiPost(`/api/traces/${state.traceId}/sync_offset`, { offset_us: Math.round(ms * 1000) });
    applyScene(res.scene);
    setStatus(`Sync offset set to ${ms} ms.`);
  });
  el("sync-offset").addEventListener("input", (e) => {
    el("sync-offset-value").textContent = e.target.value;
  });

  el("export-adma").addEventListener("click", () => runExport("adma", `/api/traces/${state.traceId}/export/adma`));
  el("export-annotation").addEventListener("click", () =>
    runExport("annotation", `/api/traces/${state.traceId}/export/annotation`)
  );
  el("export-scenario").addEventListener("click", () => {
    const enrich = el("export-enrich-osm").checked ? "?enrich=osm" : "";
    runExport("scenario", `/api/traces/${state.traceId}/export/scenario${enrich}`);
  });
  el("export-report-txt").addEventListener("click", () =>
    runExport("trace summary", `/api/traces/${state.traceId}/export/report?format=txt`)
  );
  el("export-report-xml").addEventListener("click", () =>
    runExport("trace summary", `/api/traces/${state.traceId}/export/report?format=xml`)
  );

  el("upload-btn").addEventListener("click", () => el("upload-adma").click());
  el("upload-adma").addEventListener("change", () => {
    if (el("upload-adma").files.length) el("upload-annotation").click();
  });
  el("upload-annotation").addEventListener("change", async () => {
    const admaFile = el("upload-adma").files[0];
    const annotationFile = el("upload-annotation").files[0];
    if (!admaFile || !annotationFile) return;
    const form = new FormData();
    form.append("adma_file", admaFile);
    form.append("annotation_file", annotationFile);
    form.append("name", admaFile.name.replace(/\.csv$/i, ""));
    setStatus("Uploading trace…");
    const r = await fetch("/api/traces", { method: "POST", body: form });
    if (!r.ok) { setStatus("Upload failed."); return; }
    const data = await r.json();
    await loadTrace(data.trace_id);
  });
}

async function runScan() {
  const path = el("scan-path").value.trim();
  if (!path) return;
  el("scan-status").textContent = "Scanning… this can take a moment for large corpora.";
  try {
    const r = await fetch("/api/traces/scan", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ path }),
    });
    const data = await r.json();
    if (!r.ok) {
      el("scan-status").textContent = `Scan failed: ${data.detail || r.status}`;
      return;
    }
    el("scan-status").textContent =
      `Found ${data.adma_found} ADMA file(s), ${data.xml_found} annotation file(s) — ` +
      `matched ${data.matched} pair(s). ${data.total_traces} trace(s) now available.`;
    const listing = await queryTraces("", 0);
    renderTraceListbox(listing.trace_ids, listing.total, 0);
  } catch (err) {
    el("scan-status").textContent = `Scan failed: ${err.message}`;
  }
}

// ---------- Directory browser (Scan directory "Browse…") ----------

let dirBrowserPath = null;

async function openDirBrowser() {
  el("dir-browser").classList.remove("hidden");
  await loadDirBrowser(el("scan-path").value.trim() || null);
}

async function loadDirBrowser(path) {
  try {
    const data = await apiGet(`/api/browse_dir${path ? `?path=${encodeURIComponent(path)}` : ""}`);
    dirBrowserPath = data.path;
    el("dir-browser-path").textContent = data.path;
    el("dir-browser-up").disabled = !data.parent;
    el("dir-browser-up").onclick = () => { if (data.parent) loadDirBrowser(data.parent); };
    const list = el("dir-browser-list");
    list.innerHTML = "";
    if (!data.dirs.length) {
      const empty = document.createElement("div");
      empty.className = "dir-browser-empty";
      empty.textContent = "No subfolders here.";
      list.appendChild(empty);
    }
    for (const name of data.dirs) {
      const item = document.createElement("div");
      item.className = "dir-browser-item";
      item.textContent = name;
      item.addEventListener("click", () => loadDirBrowser(`${data.path}/${name}`));
      list.appendChild(item);
    }
  } catch (err) {
    el("dir-browser-path").textContent = `Error: ${err.message}`;
  }
}

// ---------- Batch ALL matched traces: catalog / fix / fix+catalog ----------

const BATCH_ALL_BUTTON_IDS = ["scan-batch-catalog", "scan-batch-fix", "scan-batch-fix-catalog"];
const BATCH_ALL_DONE_SUFFIX = {
  catalog: "Catalog updated.",
  fix: "Corrected files written to output/.",
  fix_catalog: "Corrected files written to output/ and catalog updated.",
};

let batchAllPolling = null;

function setBatchAllButtonsDisabled(disabled) {
  for (const id of BATCH_ALL_BUTTON_IDS) el(id).disabled = disabled;
}

async function runBatchAll(mode) {
  try {
    const r = await fetch(`/api/batch/all?mode=${encodeURIComponent(mode)}`, { method: "POST" });
    const data = await r.json();
    if (!r.ok) {
      el("scan-batch-all-status").textContent = `Could not start: ${data.detail || r.status}`;
      return;
    }
    setBatchAllButtonsDisabled(true);
    el("scan-batch-all-status").textContent = `Starting: 0/${data.total}…`;
    pollBatchAllStatus();
  } catch (err) {
    el("scan-batch-all-status").textContent = `Could not start: ${err.message}`;
  }
}

function pollBatchAllStatus() {
  clearInterval(batchAllPolling);
  batchAllPolling = setInterval(async () => {
    const data = await apiGet("/api/batch/all/status");
    if (!data.total) return;
    if (data.running) {
      el("scan-batch-all-status").textContent =
        `Processing ${data.done}/${data.total}` +
        (data.current ? ` (${data.current})` : "") +
        (data.failed.length ? `, ${data.failed.length} failed so far` : "") + "…";
      return;
    }
    clearInterval(batchAllPolling);
    setBatchAllButtonsDisabled(false);
    const doneSuffix = BATCH_ALL_DONE_SUFFIX[data.mode] || "";
    el("scan-batch-all-status").textContent =
      `Done: ${data.done}/${data.total} processed` +
      (data.failed.length ? `, ${data.failed.length} failed: ${data.failed.map((f) => f.trace_id).join(", ")}` : "") +
      (doneSuffix ? ` ${doneSuffix}` : "");
  }, 700);
}

// ---------- Catalog browser ----------

const CATALOG_PAGE_SIZE = 200;

const catalogState = {
  q: "",
  phenomena: new Set(),
  issueCategories: new Set(),
  offset: 0,
  total: 0,
};

function humanizeTag(s) {
  return s.replace(/_/g, " ");
}

async function loadCatalogTags() {
  const data = await apiGet("/api/catalog/tags");

  const phenomenonBox = el("catalog-phenomenon-filters");
  phenomenonBox.innerHTML = "";
  for (const name of data.phenomena) {
    phenomenonBox.appendChild(buildCatalogChipToggle(name, catalogState.phenomena));
  }

  const issueBox = el("catalog-issue-filters");
  issueBox.innerHTML = "";
  for (const name of data.issue_categories) {
    issueBox.appendChild(buildCatalogChipToggle(name, catalogState.issueCategories));
  }
}

function buildCatalogChipToggle(name, targetSet) {
  const chip = document.createElement("button");
  chip.type = "button";
  chip.className = "catalog-chip-toggle";
  chip.textContent = humanizeTag(name);
  chip.addEventListener("click", () => {
    if (targetSet.has(name)) targetSet.delete(name); else targetSet.add(name);
    chip.classList.toggle("active");
    loadCatalogPage(0);
  });
  return chip;
}

async function loadCatalogStats() {
  const data = await apiGet("/api/catalog/stats");
  el("catalog-stats").textContent = `${data.processed} of ${data.total} trace(s) cataloged`;
}

async function loadCatalogPage(offset) {
  const params = new URLSearchParams({ limit: String(CATALOG_PAGE_SIZE), offset: String(offset) });
  if (catalogState.q) params.set("q", catalogState.q);
  for (const p of catalogState.phenomena) params.append("phenomenon", p);
  for (const c of catalogState.issueCategories) params.append("issue_category", c);

  const data = await apiGet(`/api/catalog?${params.toString()}`);
  catalogState.offset = offset;
  catalogState.total = data.total;
  renderCatalogTable(data.rows);
  renderCatalogFooter(data.rows.length);
}

function renderCatalogFooter(shownCount) {
  const { offset, total } = catalogState;
  const shownStart = shownCount ? offset + 1 : 0;
  const shownEnd = offset + shownCount;
  el("catalog-footer-label").textContent =
    total > 0 ? `Showing ${shownStart}–${shownEnd} of ${total} trace(s).` : "No matching traces.";
  const pageCount = Math.max(1, Math.ceil(total / CATALOG_PAGE_SIZE));
  const pageNum = Math.floor(offset / CATALOG_PAGE_SIZE) + 1;
  el("catalog-page-label").textContent = pageCount > 1 ? `Page ${pageNum} of ${pageCount}` : "";
  el("catalog-page-prev").disabled = offset <= 0;
  el("catalog-page-next").disabled = shownEnd >= total;
}

function renderCatalogTable(rows) {
  const tbody = el("catalog-tbody");
  tbody.innerHTML = "";
  if (!rows.length) {
    const tr = document.createElement("tr");
    const td = document.createElement("td");
    td.colSpan = 8;
    td.className = "catalog-table-empty";
    td.textContent = "No traces match these filters.";
    tr.appendChild(td);
    tbody.appendChild(tr);
    return;
  }

  for (const row of rows) {
    const tr = document.createElement("tr");

    const nameTd = document.createElement("td");
    nameTd.textContent = row.trace_id;
    tr.appendChild(nameTd);

    const locTd = document.createElement("td");
    locTd.textContent =
      row.first_lat != null && row.first_lon != null ? `${row.first_lat.toFixed(5)}, ${row.first_lon.toFixed(5)}` : "—";
    tr.appendChild(locTd);

    const durTd = document.createElement("td");
    durTd.textContent = row.duration_s != null ? `${row.duration_s.toFixed(1)}s` : "—";
    tr.appendChild(durTd);

    const vehTd = document.createElement("td");
    vehTd.textContent = row.vehicle_count != null ? row.vehicle_count : "—";
    tr.appendChild(vehTd);

    const condTd = document.createElement("td");
    condTd.textContent = [row.road_type, row.weather, row.light_conditions].filter(Boolean).join(" / ") || "—";
    tr.appendChild(condTd);

    const phenomenaTd = document.createElement("td");
    if (row.phenomena.length) {
      for (const p of row.phenomena) {
        const tag = document.createElement("span");
        tag.className = "catalog-tag phenomenon";
        tag.textContent = `${humanizeTag(p.phenomenon)} ×${p.count}`;
        phenomenaTd.appendChild(tag);
      }
    } else if (row.processed_at) {
      phenomenaTd.textContent = "none";
    } else {
      phenomenaTd.textContent = "not yet cataloged";
    }
    tr.appendChild(phenomenaTd);

    const issuesTd = document.createElement("td");
    if (row.issues.length) {
      for (const iss of row.issues) {
        const tag = document.createElement("span");
        tag.className = "catalog-tag issue";
        tag.textContent = `${humanizeTag(iss.category)} (${iss.severity}) ×${iss.count}`;
        issuesTd.appendChild(tag);
      }
    } else if (row.processed_at) {
      issuesTd.textContent = "none";
    } else {
      issuesTd.textContent = "not yet cataloged";
    }
    tr.appendChild(issuesTd);

    const openTd = document.createElement("td");
    const openBtn = document.createElement("button");
    openBtn.textContent = "Open";
    openBtn.addEventListener("click", async () => {
      closeCatalog();
      await loadTrace(row.trace_id);
    });
    openTd.appendChild(openBtn);
    tr.appendChild(openTd);

    tbody.appendChild(tr);
  }
}

let catalogTagsLoaded = false;

async function openCatalog() {
  el("catalog-modal").classList.remove("hidden");
  if (!catalogTagsLoaded) {
    await loadCatalogTags();
    catalogTagsLoaded = true;
  }
  await Promise.all([loadCatalogStats(), loadCatalogPage(0)]);
}

function closeCatalog() {
  el("catalog-modal").classList.add("hidden");
}

let catalogSearchDebounce = null;

async function init() {
  resizeCanvas();
  wireControls();
  await initTracePicker();
  try {
    const status = await apiGet("/api/batch/all/status");
    if (status.running) {
      setBatchAllButtonsDisabled(true);
      el("scan-batch-all-status").textContent = `Processing ${status.done}/${status.total}…`;
      pollBatchAllStatus();
    }
  } catch {
    // best-effort resume of an in-progress batch after a page reload
  }
  requestAnimationFrame(tick);
}

init().catch((err) => setStatus(`Error: ${err.message}`));
