"use strict";

const state = {
  traceId: null,
  scene: null,
  timeS: 0,
  playing: false,
  playbackSpeed: 1,
  loop: false,
  lastFrameMs: null,
  camera: { zoom: 8, followEgo: true, centerX: 0, centerY: 0, headingUp: true },
  drag: null,
  selectedVehicleId: null,
  showLanes: true,
  showStatic: false,
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

// ---------- Trace picker (searchable, scales to large corpora) ----------

async function queryTraces(q) {
  const params = new URLSearchParams({ limit: "200" });
  if (q) params.set("q", q);
  return apiGet(`/api/traces?${params.toString()}`);
}

async function initTracePicker() {
  const data = await queryTraces("");
  renderTraceListbox(data.trace_ids, data.total);
  if (data.trace_ids.length > 0) {
    await loadTrace(data.trace_ids[0]);
  } else {
    el("trace-picker-label").textContent = "No traces";
  }
}

let lastListedIds = [];
let lastListedTotal = 0;

function renderTraceListbox(ids, total) {
  lastListedIds = ids;
  lastListedTotal = total;
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
  const shown = ids.length;
  el("trace-picker-footer").textContent =
    total > shown ? `Showing ${shown} of ${total} traces — keep typing to narrow down.` : `${total} trace(s) available.`;
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
    const data = await queryTraces(state.searchQuery);
    renderTraceListbox(data.trace_ids, data.total);
  }, 150);
}

async function loadTrace(traceId) {
  state.traceId = traceId;
  el("trace-picker-label").textContent = traceId;
  const scene = await apiGet(`/api/traces/${traceId}/scene`);
  applyScene(scene);
  state.timeS = 0;
  state.camera.followEgo = true;
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

function applyScene(scene) {
  state.scene = scene;
  el("timeline").max = scene.duration_s.toFixed(3);
  if (state.timeS > scene.duration_s) state.timeS = 0;
  renderIssueList();
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
  if (state.camera.followEgo && ego) {
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
      setPlaying(false);
      updateTimeLabel();
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
}

function setPlaying(playing) {
  state.playing = playing;
  el("play-pause").innerHTML = playing ? "&#10074;&#10074;" : "&#9654;";
  state.lastFrameMs = null;
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

  el("batch-select-shown").addEventListener("click", () => {
    for (const id of lastListedIds) state.selectedTraceIds.add(id);
    renderTraceListbox(lastListedIds, lastListedTotal);
    updateBatchControl();
  });
  el("batch-clear-selection").addEventListener("click", () => {
    state.selectedTraceIds.clear();
    renderTraceListbox(lastListedIds, lastListedTotal);
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

  // -- playback --
  el("play-pause").addEventListener("click", () => setPlaying(!state.playing));
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
    if (e.code === "Space") { e.preventDefault(); setPlaying(!state.playing); }
    else if (e.code === "ArrowLeft") { e.preventDefault(); stepTime(-1); }
    else if (e.code === "ArrowRight") { e.preventDefault(); stepTime(1); }
    else if (e.code === "Home") { e.preventDefault(); state.timeS = 0; setPlaying(false); updateTimeLabel(); draw(); }
  });

  el("timeline").addEventListener("input", (e) => {
    state.timeS = parseFloat(e.target.value);
    setPlaying(false);
    updateTimeLabel();
    draw();
  });

  el("playback-speed").addEventListener("change", (e) => {
    state.playbackSpeed = parseFloat(e.target.value);
  });

  el("zoom-in").addEventListener("click", () => { state.camera.zoom *= 1.3; draw(); });
  el("zoom-out").addEventListener("click", () => { state.camera.zoom /= 1.3; draw(); });
  el("recenter").addEventListener("click", () => { state.camera.followEgo = true; draw(); });
  el("heading-up").addEventListener("change", (e) => { state.camera.headingUp = e.target.checked; draw(); });
  el("show-lanes").addEventListener("change", (e) => { state.showLanes = e.target.checked; draw(); });
  el("show-static").addEventListener("change", (e) => { state.showStatic = e.target.checked; draw(); });

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

  el("export-adma").addEventListener("click", () => {
    window.location.href = `/api/traces/${state.traceId}/export/adma`;
  });
  el("export-annotation").addEventListener("click", () => {
    window.location.href = `/api/traces/${state.traceId}/export/annotation`;
  });
  el("export-scenario").addEventListener("click", () => {
    window.location.href = `/api/traces/${state.traceId}/export/scenario`;
  });
  el("export-report-txt").addEventListener("click", () => {
    window.location.href = `/api/traces/${state.traceId}/export/report?format=txt`;
  });
  el("export-report-xml").addEventListener("click", () => {
    window.location.href = `/api/traces/${state.traceId}/export/report?format=xml`;
  });

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
    const listing = await queryTraces("");
    renderTraceListbox(listing.trace_ids, listing.total);
  } catch (err) {
    el("scan-status").textContent = `Scan failed: ${err.message}`;
  }
}

async function init() {
  resizeCanvas();
  wireControls();
  await initTracePicker();
  requestAnimationFrame(tick);
}

init().catch((err) => setStatus(`Error: ${err.message}`));
