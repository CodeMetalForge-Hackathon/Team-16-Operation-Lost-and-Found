const hostInput = document.querySelector("#pi-host");
const connectButton = document.querySelector("#connect");
const systemState = document.querySelector(".system-state");
const systemStatus = document.querySelector("#system-status");
const cards = [...document.querySelectorAll(".screen-card")];
const timers = new Map();
const storageKey = "team16.piHost.v2";

function normalizeHost(value) {
  let host = value.trim();
  if (!host) return "";
  if (!/^https?:\/\//i.test(host)) host = "http://" + host;
  try {
    return new URL(host).hostname;
  } catch {
    return "";
  }
}

function sourceFor(card, host) {
  return "http://" + host + ":" + card.dataset.port + "/?dashboard=" + Date.now();
}

function setCardState(card, state, label) {
  card.classList.toggle("is-live", state === "live");
  const badge = card.querySelector(".feed-state");
  badge.dataset.state = state;
  badge.textContent = label;
}

function updateSystemState() {
  const states = cards.map((card) => card.querySelector(".feed-state").dataset.state);
  const liveCount = states.filter((state) => state === "live").length;
  systemState.classList.toggle("online", liveCount > 0);
  systemState.classList.toggle("error", liveCount === 0 && states.includes("error"));
  if (liveCount === cards.length) systemStatus.textContent = "All screens live";
  else if (liveCount > 0) systemStatus.textContent = liveCount + " of " + cards.length + " screens live";
  else if (states.includes("connecting")) systemStatus.textContent = "Connecting to Pi";
  else if (states.includes("error")) systemStatus.textContent = "Pi screens unavailable";
  else systemStatus.textContent = "Awaiting Pi address";
}

function connectCard(card, host) {
  const image = card.querySelector("img");
  clearTimeout(timers.get(card));
  setCardState(card, "connecting", "Connecting");
  updateSystemState();
  image.onload = () => {
    clearTimeout(timers.get(card));
    localStorage.setItem(storageKey, host);
    setCardState(card, "live", "Live");
    updateSystemState();
  };
  image.onerror = () => {
    clearTimeout(timers.get(card));
    image.removeAttribute("src");
    setCardState(card, "error", "Unavailable");
    updateSystemState();
  };
  image.src = sourceFor(card, host);
  timers.set(card, setTimeout(() => {
    if (!card.classList.contains("is-live")) {
      image.removeAttribute("src");
      setCardState(card, "error", "Timed out");
      updateSystemState();
    }
  }, 8000));
}

function connectAll() {
  const host = normalizeHost(hostInput.value);
  if (!host) {
    hostInput.focus();
    hostInput.setAttribute("aria-invalid", "true");
    systemStatus.textContent = "Enter a valid Pi address";
    return;
  }
  hostInput.removeAttribute("aria-invalid");
  hostInput.value = host;
  cards.forEach((card) => connectCard(card, host));
  pollCalibration(host, document.querySelector("[data-action=recalibrate]"));
  watchFeeds(host);
  pollSdr(host);
  pollPi(host);
  pollMap(host);
  pollIns();
}

// Keeps the camera tiles honest: if the detector on the Pi stops making frames the tiles say so instead of
// showing a frozen picture as Live, and when it is back they reconnect by themselves (no Retry needed).
let watchTimer = null;
async function watchFeeds(host) {
  clearTimeout(watchTimer);
  let producing = false;
  try {
    const response = await fetch("http://" + host + ":8081/calibrate/status", { cache: "no-store" });
    const info = await response.json();
    producing = response.ok && !/^Fusion process/.test(info.message || "");
  } catch { /* Pi or web process unreachable */ }
  for (const card of cards) {
    const state = card.querySelector(".feed-state").dataset.state;
    if (producing && state === "error") connectCard(card, host);
    if (!producing && state === "live") {
      card.querySelector("img").removeAttribute("src");
      setCardState(card, "error", "Detector restarting");
      updateSystemState();
    }
  }
  if (producing && calStatus.textContent.startsWith("Detector is restarting")) {
    pollCalibration(host, document.querySelector("[data-action=recalibrate]"));
  }
  if (!producing && !calStatus.classList.contains("busy")) {
    calStatus.className = "cal-status bad";
    calStatus.textContent = "Detector is restarting on the Pi. The screens reconnect by themselves.";
  }
  if (normalizeHost(hostInput.value) === host) watchTimer = setTimeout(() => watchFeeds(host), 3000);
}

connectButton.addEventListener("click", connectAll);
hostInput.addEventListener("keydown", (event) => {
  if (event.key === "Enter") connectAll();
});

cards.forEach((card) => {
  card.addEventListener("click", (event) => {
    const action = event.target.closest("[data-action]")?.dataset.action;
    if (!action) return;
    const host = normalizeHost(hostInput.value);
    if (action === "retry") {
      if (host) connectCard(card, host);
      else hostInput.focus();
    }
    if (action === "open" && host) {
      window.open(sourceFor(card, host), "_blank", "noopener,noreferrer");
    }
    if (action === "recalibrate") {
      if (host) startCalibration(card, host);
      else hostInput.focus();
    }
    if (action === "fullscreen") {
      card.querySelector(".viewport").requestFullscreen?.();
    }
  });
});

const calStatus = document.querySelector("#cal-status");
let calTimer = null;

function showCalibration(info, button) {
  const busy = info.state === "collecting";
  calStatus.className = "cal-status " + (busy ? "busy" : /^Failed/.test(info.message) ? "bad" : info.calibrated ? "ok" : "");
  if (busy) {
    calStatus.textContent = "Calibrating " + info.samples + "/" + info.needed + " frames, " + info.secs_left +
      "s left. Stand alone in view of both cameras and move slowly side to side.";
  } else {
    calStatus.textContent = info.message || (info.calibrated ? "Alignment: calibrated" : "Alignment: not calibrated (rough guess)");
  }
  if (button) button.disabled = busy;
}

async function calibrationCall(host, path) {
  const response = await fetch("http://" + host + ":8081" + path, { cache: "no-store" });
  if (!response.ok) throw new Error("HTTP " + response.status);
  return response.json();
}

async function pollCalibration(host, button) {
  clearTimeout(calTimer);
  try {
    const info = await calibrationCall(host, "/calibrate/status");
    showCalibration(info, button);
    if (info.state === "collecting") calTimer = setTimeout(() => pollCalibration(host, button), 1000);
  } catch {
    calStatus.className = "cal-status bad";
    calStatus.textContent = "Could not reach the calibration service on the Pi";
    if (button) button.disabled = false;
  }
}

async function startCalibration(card, host) {
  const button = card.querySelector("[data-action=recalibrate]");
  button.disabled = true;
  try {
    showCalibration(await calibrationCall(host, "/calibrate"), button);
    calTimer = setTimeout(() => pollCalibration(host, button), 1000);
  } catch {
    calStatus.className = "cal-status bad";
    calStatus.textContent = "Could not reach the calibration service on the Pi";
    button.disabled = false;
  }
}

function setTile(el, text, cls) {
  el.textContent = text;
  el.className = cls || "";
}

function fmtDuration(s) {
  if (s == null) return "—";
  s = Math.round(s);
  if (s < 60) return s + " s";
  if (s < 3600) return Math.floor(s / 60) + " min " + (s % 60) + " s";
  return Math.floor(s / 3600) + " h " + Math.floor((s % 3600) / 60) + " min";
}

function cssVar(name) {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim();
}

function drawChart(canvas, points, opts) {
  const dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth, h = canvas.clientHeight;
  if (!w || !h) return;
  canvas.width = Math.round(w * dpr);
  canvas.height = Math.round(h * dpr);
  const g = canvas.getContext("2d");
  g.scale(dpr, dpr);
  g.clearRect(0, 0, w, h);
  const padL = 38, padB = 16, padT = 6, padR = 6;
  const pw = w - padL - padR, ph = h - padT - padB;
  const now = Date.now() / 1000, span = 300;
  const [lo, hi] = opts.range;
  const x = (t) => padL + ((t - (now - span)) / span) * pw;
  const y = (v) => padT + (1 - (Math.min(Math.max(v, lo), hi) - lo) / (hi - lo)) * ph;
  g.font = "10px system-ui, sans-serif";
  g.fillStyle = cssVar("--muted");
  g.strokeStyle = cssVar("--grid");
  g.lineWidth = 1;
  const step = opts.step;
  for (let v = Math.ceil(lo / step) * step; v <= hi; v += step) {
    g.beginPath(); g.moveTo(padL, y(v)); g.lineTo(w - padR, y(v)); g.stroke();
    g.fillText(String(v), 4, y(v) + 3);
  }
  g.fillText("-5 min", padL, h - 3);
  g.fillText("now", w - padR - 20, h - 3);
  if (opts.floor != null) {
    g.setLineDash([4, 4]); g.strokeStyle = cssVar("--muted");
    g.beginPath(); g.moveTo(padL, y(opts.floor)); g.lineTo(w - padR, y(opts.floor)); g.stroke();
    g.setLineDash([]);
  }
  if (!points.length) {
    g.fillStyle = cssVar("--muted");
    g.fillText(opts.empty, padL + 8, padT + ph / 2);
    return;
  }
  g.strokeStyle = cssVar(opts.color);
  g.lineWidth = 1.6;
  g.beginPath();
  let prev = null;
  for (const p of points) {
    const px = x(p.t), py = y(p.v);
    if (prev === null || p.t - prev > opts.gap) g.moveTo(px, py); else g.lineTo(px, py);
    prev = p.t;
  }
  g.stroke();
  if (opts.dots) {
    g.fillStyle = cssVar(opts.color);
    for (const p of points) { g.beginPath(); g.arc(x(p.t), y(p.v), 2, 0, 6.3); g.fill(); }
  }
}

// ---------------------------------------------------------------- SDR phone detect (:8085, bridge to the rf hub)
const sdrEls = {
  state: document.querySelector("#sdr-state"), note: document.querySelector("#sdr-note"),
  hub: document.querySelector("#s-hub"), close: document.querySelector("#s-close"),
  closeRssi: document.querySelector("#s-close-rssi"), wifi: document.querySelector("#s-wifi"),
  ble: document.querySelector("#s-ble"), bt: document.querySelector("#s-bt"),
  cellRate: document.querySelector("#s-cell-rate"), cellBest: document.querySelector("#s-cell-best"),
  rate: document.querySelector("#s-rate"), total: document.querySelector("#s-total"),
  cellNote: document.querySelector("#s-cell-note"), devices: document.querySelector("#s-devices"),
  events: document.querySelector("#s-events"), retry: document.querySelector("#sdr-retry"),
};
let sdrTimer = null;
let sdrFails = 0;
const CAT_LABEL = { wifi: "Wi-Fi", ble: "BLE", bt: "BT", cell: "Cell" };

function fillList(ol, rows, emptyText) {
  ol.replaceChildren();
  if (!rows.length) {
    const li = document.createElement("li");
    li.className = "muted"; li.textContent = emptyText;
    ol.append(li);
    return;
  }
  for (const [left, right] of rows) {
    const li = document.createElement("li");
    const a = document.createElement("span"), b = document.createElement("span");
    a.textContent = left; b.textContent = right;
    li.append(a, b);
    ol.append(li);
  }
}

function renderSdr(d) {
  const down = Object.entries(d.hub).filter(([, up]) => !up).map(([c]) => CAT_LABEL[c]);
  sdrEls.state.dataset.state = d.hub_all_up ? "live" : d.hub_any_up ? "connecting" : "error";
  sdrEls.state.textContent = d.hub_all_up ? "Listening" : d.hub_any_up ? "Partial" : "Hub offline";
  setTile(sdrEls.hub, d.hub_all_up ? "ALL 4 UP" : d.hub_any_up ? down.join(", ") + " down" : "OFFLINE",
          d.hub_all_up ? "good" : d.hub_any_up ? "warn" : "bad");
  const c = d.closest;
  setTile(sdrEls.close, c ? (c.name || c.mac) : "—", c ? "good" : "");
  setTile(sdrEls.closeRssi, c ? c.rssi + " dBm (" + CAT_LABEL[c.cat] + ")" : "—");
  setTile(sdrEls.wifi, String(d.active.wifi));
  setTile(sdrEls.ble, String(d.active.ble));
  setTile(sdrEls.bt, String(d.active.bt));
  setTile(sdrEls.cellRate, String(d.per_min.cell), d.per_min.cell ? "good" : "");
  const best = d.cell.channels.slice().sort((a, b) => b.power_db - a.power_db)[0];
  setTile(sdrEls.cellBest, best ? (best.band ? best.band + " · " : "") + best.mhz + " MHz" : "—");
  const perMin = Object.values(d.per_min).reduce((a, b) => a + b, 0);
  setTile(sdrEls.rate, String(perMin));
  setTile(sdrEls.total, String(Object.values(d.counts).reduce((a, b) => a + b, 0)));

  const devs = d.devices.slice().sort((a, b) => (b.rssi ?? -999) - (a.rssi ?? -999)).slice(0, 12)
    .map((x) => [CAT_LABEL[x.cat] + " " + (x.name || x.mac), (x.rssi == null ? "?" : x.rssi + " dBm") + " · " + Math.round(x.age_s) + " s"]);
  fillList(sdrEls.devices, devs, "No devices heard in the last 5 min");
  const evs = d.events.slice().reverse().slice(0, 12).map((e) => {
    const when = new Date(e.t * 1000).toLocaleTimeString([], { hour12: false });
    return e.cat === "cell"
      ? [when + " Cell " + (e.hz / 1e6).toFixed(0) + " MHz", e.power_db + " dB"]
      : [when + " " + CAT_LABEL[e.cat] + " " + (e.name || e.mac), e.rssi == null ? "?" : e.rssi + " dBm"];
  });
  fillList(sdrEls.events, evs, "No hits yet");

  const pts = d.cell.history.map((p) => ({ t: p.t, v: p.v }));
  const vals = pts.map((p) => p.v);
  const lo = vals.length ? Math.floor((Math.min(...vals) - 3) / 10) * 10 : -110;
  const hi = vals.length ? Math.max(lo + 30, Math.ceil((Math.max(...vals) + 3) / 10) * 10) : -60;
  drawChart(document.querySelector("#c-cell"), pts, { range: [lo, hi], step: 10, color: "--amber", gap: 30, dots: true, empty: "no cellular hits" });
  sdrEls.cellNote.textContent = d.cell.channels.length
    ? d.cell.channels.map((x) => (x.band || "") + " " + x.mhz + " MHz: " + x.power_db + " dB").join(" · ")
    : "Pluto, no hits yet";
  sdrEls.note.className = "cal-status " + (d.hub_all_up ? "ok" : "bad");
  sdrEls.note.textContent = d.hub_all_up
    ? "Receive-only. Signal is relative strength, not direction: walk toward the strongest reading."
    : "RF hub not fully running on the Pi (" + (down.join(", ") || "none") + " not connected)";
}

async function pollSdr(host) {
  clearTimeout(sdrTimer);
  try {
    const response = await fetch("http://" + host + ":8085/sdr", { cache: "no-store" });
    if (!response.ok) throw new Error("HTTP " + response.status);
    sdrFails = 0;
    renderSdr(await response.json());
  } catch {
    sdrFails += 1;
    sdrEls.state.dataset.state = sdrFails > 2 ? "error" : "connecting";
    sdrEls.state.textContent = sdrFails > 2 ? "Unavailable" : "Connecting";
    sdrEls.note.className = "cal-status bad";
    sdrEls.note.textContent = "Could not reach the SDR feed on the Pi (port 8085)";
  }
  if (normalizeHost(hostInput.value) === host) sdrTimer = setTimeout(() => pollSdr(host), 1000);
}

sdrEls.retry.addEventListener("click", () => {
  const host = normalizeHost(hostInput.value);
  if (host) { sdrFails = 0; pollSdr(host); } else hostInput.focus();
});

// ---------------------------------------------------------------- INS dead reckoning (:8084)
const insEls = {
  state: document.querySelector("#ins-state"), msg: document.querySelector("#ins-msg"),
  form: document.querySelector("#ins-form"), lat: document.querySelector("#ins-lat"),
  lon: document.querySelector("#ins-lon"), alt: document.querySelector("#ins-alt"),
  host: document.querySelector("#ins-host"), set: document.querySelector("#ins-set"),
  heading: document.querySelector("#ins-heading"), mag: document.querySelector("#ins-mag"),
  dd: document.querySelector("#i-dd"), altNow: document.querySelector("#i-alt"),
  speed: document.querySelector("#i-speed"), hdg: document.querySelector("#i-hdg"),
  sigma: document.querySelector("#i-sigma"), off: document.querySelector("#i-off"),
  fixes: document.querySelector("#i-fixes"), mode: document.querySelector("#i-mode"),
  src: document.querySelector("#i-src"), modes: document.querySelector("#i-modes"),
  retry: document.querySelector("#ins-retry"), plot: document.querySelector("#c-ins"),
};
const insHostKey = "team16.insHost.v1";
let insTimer = null;
let insFails = 0;
let insFormDirty = false;

try { insEls.host.value = localStorage.getItem(insHostKey) || ""; } catch { /* storage unavailable */ }

function insTarget() {
  return normalizeHost(insEls.host.value) || normalizeHost(hostInput.value);
}

function drawInsPlot(d) {
  const canvas = insEls.plot;
  const dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth, h = canvas.clientHeight;
  if (!w || !h) return;
  canvas.width = Math.round(w * dpr); canvas.height = Math.round(h * dpr);
  const g = canvas.getContext("2d");
  g.scale(dpr, dpr);
  g.clearRect(0, 0, w, h);
  const sx = d.start.x, sy = d.start.y;
  const pts = d.path.map((p) => [p[0] - sx, p[1] - sy]);
  const cur = [d.x - sx, d.y - sy];
  const sig = d.sigma_h_m || 0;
  let minX = Math.min(0, cur[0] - sig, ...pts.map((p) => p[0])), maxX = Math.max(0, cur[0] + sig, ...pts.map((p) => p[0]));
  let minY = Math.min(0, cur[1] - sig, ...pts.map((p) => p[1])), maxY = Math.max(0, cur[1] + sig, ...pts.map((p) => p[1]));
  const span = Math.max(maxX - minX, maxY - minY, 40) * 1.15;
  const cx = (minX + maxX) / 2, cy = (minY + maxY) / 2;
  const scale = Math.min(w, h) / span;
  const X = (e) => w / 2 + (e - cx) * scale, Y = (n) => h / 2 - (n - cy) * scale;
  const nice = [10, 20, 50, 100, 200, 500, 1000, 2000, 5000].find((s) => s * scale > 48) || 5000;
  g.font = "10px system-ui, sans-serif";
  g.strokeStyle = cssVar("--grid"); g.fillStyle = cssVar("--muted"); g.lineWidth = 1;
  for (let e = Math.ceil((cx - w / 2 / scale) / nice) * nice; e < cx + w / 2 / scale; e += nice) {
    g.beginPath(); g.moveTo(X(e), 0); g.lineTo(X(e), h); g.stroke(); g.fillText(e + " m", X(e) + 3, h - 4);
  }
  for (let n = Math.ceil((cy - h / 2 / scale) / nice) * nice; n < cy + h / 2 / scale; n += nice) {
    g.beginPath(); g.moveTo(0, Y(n)); g.lineTo(w, Y(n)); g.stroke(); g.fillText(n + " m", 4, Y(n) - 3);
  }
  g.strokeStyle = cssVar("--cyan"); g.lineWidth = 1.8; g.beginPath();
  pts.forEach((p, i) => (i ? g.lineTo(X(p[0]), Y(p[1])) : g.moveTo(X(p[0]), Y(p[1]))));
  g.stroke();
  if (sig > 0) {
    g.strokeStyle = cssVar("--amber"); g.setLineDash([4, 4]);
    g.beginPath(); g.arc(X(cur[0]), Y(cur[1]), Math.max(2, sig * scale), 0, 6.3); g.stroke(); g.setLineDash([]);
  }
  g.fillStyle = cssVar("--green"); g.beginPath(); g.arc(X(0), Y(0), 5, 0, 6.3); g.fill();
  g.fillStyle = cssVar("--red"); g.beginPath(); g.arc(X(cur[0]), Y(cur[1]), 5, 0, 6.3); g.fill();
}

function renderIns(d) {
  const live = d.input_age_s != null && d.input_age_s < 3;
  insEls.state.dataset.state = live ? "live" : "connecting";
  insEls.state.textContent = live ? (d.flying ? "Flying" : "Receiving") : "No sensor input";
  insEls.dd.textContent = d.dd + " (DD)";
  insEls.altNow.textContent = d.z.toFixed(1) + " m";
  insEls.speed.textContent = d.walking ? "—" : d.speed_ms.toFixed(1) + " m/s";
  insEls.hdg.textContent = d.heading_deg.toFixed(0) + "°";
  const sig = d.sigma_h_m;
  insEls.sigma.textContent = "±" + sig + " m";
  insEls.sigma.className = sig < 10 ? "good" : sig < 60 ? "warn" : "bad";
  const dx = d.x - d.start.x, dy = d.y - d.start.y;
  const dist = Math.hypot(dx, dy), brg = (Math.atan2(dx, dy) * 180 / Math.PI + 360) % 360;
  insEls.off.textContent = dist.toFixed(0) + " m @ " + brg.toFixed(0) + "°";
  insEls.fixes.textContent = d.fixes + (d.fixes_rejected ? " (" + d.fixes_rejected + " rejected)" : "");
  insEls.mode.textContent = d.calibrating ? "CALIBRATING – hold still"
    : d.walking ? "WALKING · " + d.steps + " steps · " + d.distance_m + " m"
    : d.flying ? "FLYING" : "LANDED";
  insEls.src.textContent = d.source;
  insEls.modes.querySelectorAll("button").forEach((b) => b.classList.toggle("on", b.dataset.mode === d.mode));
  if (!insFormDirty && document.activeElement !== insEls.lat && document.activeElement !== insEls.lon) {
    insEls.lat.value = d.start.lat; insEls.lon.value = d.start.lon; insEls.alt.value = d.start.alt;
    insEls.heading.value = d.start.heading;
  }
  if (document.activeElement !== insEls.mag) insEls.mag.checked = d.use_mag;
  insEls.msg.className = "cal-status " + (live ? "ok" : "bad");
  insEls.msg.textContent = live
    ? "Dead reckoning from the start point; each lidar fix pulls the position back and shrinks the uncertainty."
    : "INS service is up but no IMU data is arriving. Check the sensor connection.";
  drawInsPlot(d);
}

async function pollIns() {
  clearTimeout(insTimer);
  const host = insTarget();
  if (host) {
    try {
      const response = await fetch("http://" + host + ":8084/ins", { cache: "no-store" });
      if (!response.ok) throw new Error("HTTP " + response.status);
      insFails = 0;
      renderIns(await response.json());
    } catch {
      insFails += 1;
      insEls.state.dataset.state = insFails > 2 ? "error" : "connecting";
      insEls.state.textContent = insFails > 2 ? "Not connected" : "Connecting";
      insEls.msg.className = "cal-status bad";
      insEls.msg.textContent = "INS service not reachable at " + host + ":8084 – start dr_run.py there (on the Pi once the IMU is wired to it).";
    }
  }
  insTimer = setTimeout(pollIns, 1000);
}

async function insCall(path) {
  const host = insTarget();
  if (!host) { hostInput.focus(); throw new Error("Enter the Pi address (or an INS service address) first"); }
  const response = await fetch("http://" + host + ":8084" + path, { cache: "no-store" });
  const body = await response.json();
  if (!response.ok) throw new Error(body.error || "HTTP " + response.status);
  return body;
}

insEls.form.addEventListener("submit", async (event) => {
  event.preventDefault();
  insEls.set.disabled = true;
  try {
    const q = new URLSearchParams({ lat: insEls.lat.value.trim(), lon: insEls.lon.value.trim() });
    if (insEls.alt.value.trim()) q.set("alt", insEls.alt.value.trim());
    if (insEls.heading.value.trim()) q.set("heading", insEls.heading.value.trim());
    const d = await insCall("/start?" + q);
    insFormDirty = false;
    insEls.msg.className = "cal-status ok";
    insEls.msg.textContent = "Start set to " + d.dd + " (decimal degrees).";
    renderIns({ ...d, input_age_s: 0, source: d.source || "" });
  } catch (err) {
    insEls.msg.className = "cal-status bad";
    insEls.msg.textContent = "Could not set the start: " + err.message;
  }
  insEls.set.disabled = false;
});
insEls.mag.addEventListener("change", async () => {
  try { await insCall("/magnetometer?on=" + (insEls.mag.checked ? 1 : 0)); pollIns(); } catch (err) {
    insEls.msg.className = "cal-status bad"; insEls.msg.textContent = "Could not change the compass setting: " + err.message;
  }
});
[insEls.lat, insEls.lon, insEls.alt, insEls.heading].forEach((el) => el.addEventListener("input", () => { insFormDirty = true; }));
insEls.host.addEventListener("change", () => {
  try { localStorage.setItem(insHostKey, insEls.host.value.trim()); } catch { /* storage unavailable */ }
  insFails = 0; pollIns();
});
insEls.retry.addEventListener("click", () => { insFails = 0; pollIns(); });
insEls.modes.addEventListener("click", async (event) => {
  const mode = event.target.closest("button")?.dataset.mode;
  if (!mode) return;
  try { await insCall("/mode?mode=" + mode); pollIns(); } catch (err) {
    insEls.msg.className = "cal-status bad"; insEls.msg.textContent = "Mode change failed: " + err.message;
  }
});

const queryHost = new URLSearchParams(location.search).get("host");
// ---- Raspberry Pi health (:8087, pi_stats.py on the Pi)
const piEls = {
  state: document.querySelector("#pi-state"), note: document.querySelector("#pi-note"),
  cpu: document.querySelector("#p-cpu"), temp: document.querySelector("#p-temp"), mem: document.querySelector("#p-mem"),
  load: document.querySelector("#p-load"), thr: document.querySelector("#p-thr"), hailo: document.querySelector("#p-hailo"),
  det: document.querySelector("#p-det"), restarts: document.querySelector("#p-restarts"), cell: document.querySelector("#p-cell"),
  up: document.querySelector("#p-up"), cores: document.querySelector("#p-cores"), top: document.querySelector("#p-top"),
  retry: document.querySelector("#pi-retry"),
};
let piTimer = null, piFails = 0;

function renderPi(d) {
  piEls.state.dataset.state = "live";
  piEls.state.textContent = "Live";
  const cpu = d.cpu.total;
  setTile(piEls.cpu, cpu == null ? "—" : cpu.toFixed(0) + " %  @ " + (d.cpu_mhz || "?") + " MHz", cpu > 90 ? "bad" : cpu > 70 ? "warn" : "");
  setTile(piEls.temp, d.temp_c == null ? "—" : d.temp_c.toFixed(1) + " °C", d.temp_c > 80 ? "bad" : d.temp_c > 70 ? "warn" : "");
  setTile(piEls.mem, d.mem.used_mb + " / " + d.mem.total_mb + " MB (" + d.mem.used_pct + " %)", d.mem.used_pct > 90 ? "bad" : "");
  setTile(piEls.load, d.load.map((x) => x.toFixed(1)).join(" / "), d.load[0] > d.cores_n ? "warn" : "");
  const t = d.throttle;
  if (!t) setTile(piEls.thr, "—");
  else if (t.now.length) setTile(piEls.thr, "NOW: " + t.now.join(", "), "bad");
  else if (t.since_boot.length) setTile(piEls.thr, "OK now (since boot: " + t.since_boot.join(", ") + ")", "warn");
  else setTile(piEls.thr, "OK, no under-voltage", "good");
  const hl = d.hailo, det = hl.detector, sup = hl.supervisor || {};
  setTile(piEls.hailo, hl.on_bus ? "On bus" + (hl.link ? " (" + hl.link + ")" : "") : "MISSING from the bus", hl.on_bus ? "good" : "bad");
  if (!det || det.age_s == null) setTile(piEls.det, "not running", "bad");
  else if (!det.ok) setTile(piEls.det, "STALLED " + det.age_s.toFixed(0) + " s", "bad");
  else if (det.ai && det.ai !== "ready") setTile(piEls.det, "AI " + det.ai + " (video live)", "warn");
  else setTile(piEls.det, (det.fps != null ? det.fps + " fps" : "running") + (sup.hailo_unavailable ? " (no Hailo)" : ""), sup.hailo_unavailable ? "warn" : "good");
  const r = sup.watchdogs_15min;
  const as = det && det.ai_stalls_15min;
  setTile(piEls.restarts, as == null ? "—" : as + " AI stall" + (as === 1 ? "" : "s") + (r ? ", " + r + " restart" + (r === 1 ? "" : "s") : ""), r > 0 || as > 10 ? "bad" : as > 0 ? "warn" : "good");
  setTile(piEls.cell, d.cell_scan_running ? "RUNNING" : "off", d.cell_scan_running ? "warn" : "");
  setTile(piEls.up, fmtDuration(d.uptime_s));
  piEls.cores.replaceChildren();
  d.cpu.cores.forEach((c, i) => {
    const li = document.createElement("li");
    const label = document.createElement("span"); label.textContent = "Core " + i;
    const track = document.createElement("span"); track.className = "bar";
    const fill = document.createElement("i"); fill.style.width = Math.min(100, c) + "%";
    if (c > 90) fill.className = "hot";
    track.append(fill);
    const val = document.createElement("span"); val.textContent = c.toFixed(0) + " %";
    li.append(label, track, val);
    piEls.cores.append(li);
  });
  piEls.top.replaceChildren();
  d.top.forEach((p) => {
    const li = document.createElement("li");
    li.textContent = p.name + "  " + p.cpu.toFixed(0) + " % cpu";
    piEls.top.append(li);
  });
  const hist = (key) => d.history.filter((p) => p[key] != null).map((p) => ({ t: p.t, v: p[key] }));
  drawChart(document.querySelector("#c-pcpu"), hist("cpu"), { range: [0, 100], step: 25, color: "--cyan", gap: 8, empty: "collecting…" });
  drawChart(document.querySelector("#c-ptemp"), hist("temp"), { range: [30, 90], step: 20, color: "--amber", gap: 8, floor: 80, empty: "collecting…" });
  const issues = [];
  if (!hl.on_bus) issues.push("the Hailo chip is not on the PCIe bus (needs a power-cycle)");
  if (det && !det.ok) issues.push("the detector is stalled");
  if (sup.watchdogs_15min > 2) issues.push("the detector has had to be restarted " + sup.watchdogs_15min + " times in 15 min");
  if (det && det.ai_stalls_15min > 3) issues.push("the AI chip has stalled " + det.ai_stalls_15min + " times in 15 min (the video stays live, the boxes drop out for a few seconds)");
  if (t && t.now.length) issues.push("the Pi is " + t.now.join(" and "));
  if (hl.driver_errors) issues.push(hl.driver_errors + " Hailo driver error" + (hl.driver_errors === 1 ? "" : "s") + " since boot");
  piEls.note.className = "cal-status" + (issues.length ? " bad" : "");
  piEls.note.textContent = issues.length ? "Attention: " + issues.join("; ") + "." : "All good: Hailo on the bus, detector producing frames, no throttling.";
}

async function pollPi(host) {
  clearTimeout(piTimer);
  try {
    const response = await fetch("http://" + host + ":8087/stats", { cache: "no-store" });
    if (!response.ok) throw new Error("HTTP " + response.status);
    piFails = 0;
    renderPi(await response.json());
  } catch {
    piFails += 1;
    piEls.state.dataset.state = piFails > 2 ? "error" : "connecting";
    piEls.state.textContent = piFails > 2 ? "Unavailable" : "Connecting";
    piEls.note.className = "cal-status bad";
    piEls.note.textContent = "Pi stats not reachable at " + host + ":8087 (pi_stats.py on the Pi). If the whole Pi is frozen this is the first thing to go dark.";
  }
  if (normalizeHost(hostInput.value) === host) piTimer = setTimeout(() => pollPi(host), 2000);
}

piEls.retry.addEventListener("click", () => { const h = normalizeHost(hostInput.value); if (h) { piFails = 0; pollPi(h); } else hostInput.focus(); });

// ---- Lidar room map (:8086, posted by the lidar laptop). Lidar positions + map only, no fusion with the INS.
const mapEls = {
  state: document.querySelector("#map-state"), note: document.querySelector("#map-note"),
  lock: document.querySelector("#m-lock"), xy: document.querySelector("#m-xy"), z: document.querySelector("#m-z"),
  hdg: document.querySelector("#m-hdg"), dist: document.querySelector("#m-dist"),
  canvas: document.querySelector("#c-map"), rotNote: document.querySelector("#map-rot-note"), rotate: document.querySelector("#map-rotate"), retry: document.querySelector("#map-retry"), clear: document.querySelector("#map-clear"),
};
let mapTimer = null, mapFails = 0, mapDoc = null, mapTrail = [], mapDist = 0, mapLast = null;

function readPose(p) {
  if (!p) return null;
  let x, y, z, yawCw = null;
  if (Array.isArray(p)) { [x, y, z] = p; if (p.length > 3) yawCw = -p[3]; }
  else {
    x = p.x_m ?? p.x; y = p.y_m ?? p.y; z = p.z_m ?? p.z ?? 0;
    if (p.yaw_cw_deg != null) yawCw = p.yaw_cw_deg;
    else if (p.heading_cw_deg != null) yawCw = p.heading_cw_deg;
    else if (p.yaw_deg != null) yawCw = -p.yaw_deg;   // yaw is counter-clockwise in the lidar stack
    else if (p.yaw != null) yawCw = -p.yaw;
  }
  x = Number(x); y = Number(y); z = Number(z);
  if (!Number.isFinite(x) || !Number.isFinite(y)) return null;
  return { x, y, z: Number.isFinite(z) ? z : 0, yawCw: yawCw == null ? null : Number(yawCw) };
}

let mapRot = 0;   // quarter turns clockwise, remembered in this browser
try { mapRot = Number(localStorage.getItem("team16.mapRot")) || 0; } catch { /* storage unavailable */ }
// rotate a map-metre point (x right, y up) by mapRot quarter turns clockwise
function rotPt(x, y) {
  switch (((mapRot % 4) + 4) % 4) {
    case 1: return [y, -x];
    case 2: return [-x, -y];
    case 3: return [-y, x];
    default: return [x, y];
  }
}

function drawMap() {
  const canvas = mapEls.canvas, d = mapDoc;
  const dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth, h = canvas.clientHeight;
  if (!w || !h) return;
  canvas.width = Math.round(w * dpr); canvas.height = Math.round(h * dpr);
  const g = canvas.getContext("2d");
  g.scale(dpr, dpr);
  g.clearRect(0, 0, w, h);
  if (!d || !d.width || !d.height) {
    g.fillStyle = cssVar("--muted"); g.font = "13px system-ui, sans-serif";
    g.fillText("Waiting for the lidar laptop to post a map…", 16, 28);
    return;
  }
  const mpp = d.meters_per_pixel || 0.1, ox = d.origin?.px ?? 0, oy = d.origin?.py ?? 0;
  const cur = readPose(d.pose);
  // view = the map grid plus anywhere the track has been, so the position never falls off the plot
  let X0 = -ox * mpp, X1 = (d.width - ox) * mpp, Y0 = (oy - d.height) * mpp, Y1 = oy * mpp;
  for (const p of cur ? [...mapTrail, cur] : mapTrail) { X0 = Math.min(X0, p.x - 0.5); X1 = Math.max(X1, p.x + 0.5); Y0 = Math.min(Y0, p.y - 0.5); Y1 = Math.max(Y1, p.y + 0.5); }
  const corners = [rotPt(X0, Y0), rotPt(X1, Y0), rotPt(X0, Y1), rotPt(X1, Y1)];
  const x0 = Math.min(...corners.map((c) => c[0])), x1 = Math.max(...corners.map((c) => c[0]));
  const y0 = Math.min(...corners.map((c) => c[1])), y1 = Math.max(...corners.map((c) => c[1]));
  const pad = 14, scale = Math.min((w - 2 * pad) / (x1 - x0), (h - 2 * pad) / (y1 - y0));   // px per metre
  const left = (w - (x1 - x0) * scale) / 2, top = (h - (y1 - y0) * scale) / 2;
  const sx = (xm, ym) => { const r = rotPt(xm, ym); return left + (r[0] - x0) * scale; };
  const sy = (xm, ym) => { const r = rotPt(xm, ym); return top + (y1 - r[1]) * scale; };
  const cell = mpp * scale;
  g.strokeStyle = cssVar("--grid"); g.strokeRect(left, top, (x1 - x0) * scale, (y1 - y0) * scale);
  g.strokeStyle = cssVar("--grid"); g.fillStyle = cssVar("--muted"); g.font = "10px system-ui, sans-serif";
  for (let e = Math.ceil(X0); e <= X1; e++) { g.beginPath(); g.moveTo(sx(e, Y0), sy(e, Y0)); g.lineTo(sx(e, Y1), sy(e, Y1)); g.stroke(); }
  for (let n = Math.ceil(Y0); n <= Y1; n++) { g.beginPath(); g.moveTo(sx(X0, n), sy(X0, n)); g.lineTo(sx(X1, n), sy(X1, n)); g.stroke(); }
  g.fillText("1 m grid", left + 4, top + (y1 - y0) * scale - 4);
  g.fillStyle = cssVar("--map-cell");
  const sz = Math.max(1.5, cell);
  for (const [col, row] of d.outline || []) {
    const xm = (col - ox + 0.5) * mpp, ym = (oy - row - 0.5) * mpp;   // cell centre, metres
    g.fillRect(sx(xm, ym) - sz / 2, sy(xm, ym) - sz / 2, sz, sz);
  }
  g.fillStyle = cssVar("--green"); g.beginPath(); g.arc(sx(0, 0), sy(0, 0), 4, 0, 6.3); g.fill();   // map origin
  g.strokeStyle = cssVar("--cyan"); g.lineWidth = 2; g.beginPath();
  mapTrail.forEach((p, i) => (i ? g.lineTo(sx(p.x, p.y), sy(p.x, p.y)) : g.moveTo(sx(p.x, p.y), sy(p.x, p.y))));
  g.stroke();
  if (cur) {
    const X = sx(cur.x, cur.y), Y = sy(cur.x, cur.y);
    if (cur.yawCw != null) {   // heading is clockwise from +X; each clockwise map turn adds 90 degrees on screen
      const a = ((cur.yawCw + 90 * (((mapRot % 4) + 4) % 4)) * Math.PI) / 180, len = 16;
      g.strokeStyle = cssVar("--red"); g.lineWidth = 2.5; g.beginPath();
      g.moveTo(X, Y); g.lineTo(X + Math.cos(a) * len, Y + Math.sin(a) * len); g.stroke();
    }
    g.fillStyle = cssVar("--red"); g.beginPath(); g.arc(X, Y, 6, 0, 6.3); g.fill();
  }
  const turns = ((mapRot % 4) + 4) % 4;
  mapEls.rotNote.textContent = turns ? "(rotated " + turns * 90 + "° clockwise)" : "(+X right, +Y up)";
}

function renderMap(d) {
  mapDoc = d;
  const cur = d.locked === false ? null : readPose(d.pose);
  if (Array.isArray(d.trail)) {   // the feed keeps every pose ever posted, so use that as the track
    mapTrail = d.trail.map(readPose).filter(Boolean);
    mapDist = 0;
    for (let i = 1; i < mapTrail.length; i++) mapDist += Math.hypot(mapTrail[i].x - mapTrail[i - 1].x, mapTrail[i].y - mapTrail[i - 1].y);
  } else if (cur) {
    if (mapLast) {
      const step = Math.hypot(cur.x - mapLast.x, cur.y - mapLast.y);
      if (step > 0.03) { mapDist += step; mapTrail.push(cur); mapLast = cur; }
    } else { mapTrail.push(cur); mapLast = cur; }
    if (mapTrail.length > 3000) mapTrail.shift();
  }
  mapEls.state.dataset.state = "live";
  mapEls.state.textContent = cur ? "Locked" : "No lock";
  mapEls.lock.textContent = cur ? "LOCKED" : "no lock";
  mapEls.xy.textContent = cur ? cur.x.toFixed(2) + " / " + cur.y.toFixed(2) : "—";
  mapEls.z.textContent = cur ? cur.z.toFixed(2) + " m" : "—";
  mapEls.hdg.textContent = cur && cur.yawCw != null ? (Math.round(((cur.yawCw % 360) + 360) % 360) % 360) + "°" : "—";
  mapEls.dist.textContent = mapDist.toFixed(1) + " m";
  const cells = (d.outline || []).length;
  mapEls.note.className = "cal-status" + (cur ? "" : " bad");
  mapEls.note.textContent = cur
    ? "Lidar position holding. Map: " + cells + " occupied cells at " + ((d.meters_per_pixel || 0.1) * 100).toFixed(0) + " cm."
    : (cells ? "Map loaded (" + cells + " cells) but the lidar has no position lock yet. The track continues when it locks."
             : "Feed is up but no map has been posted yet by the lidar laptop.");
  drawMap();
}

async function pollMap(host) {
  clearTimeout(mapTimer);
  try {
    const response = await fetch("http://" + host + ":8086/map", { cache: "no-store" });
    if (!response.ok) throw new Error("HTTP " + response.status);
    mapFails = 0;
    renderMap(await response.json());
  } catch {
    mapFails += 1;
    mapEls.state.dataset.state = mapFails > 2 ? "error" : "connecting";
    mapEls.state.textContent = mapFails > 2 ? "Not connected" : "Connecting";
    mapEls.note.className = "cal-status bad";
    mapEls.note.textContent = "Map feed not reachable at " + host + ":8086 (map_feed.py on the Pi).";
  }
  if (normalizeHost(hostInput.value) === host) mapTimer = setTimeout(() => pollMap(host), 500);
}

mapEls.retry.addEventListener("click", () => { const h = normalizeHost(hostInput.value); if (h) pollMap(h); });
mapEls.clear.addEventListener("click", async () => {
  mapTrail = []; mapDist = 0; mapLast = null; drawMap();
  const h = normalizeHost(hostInput.value);
  if (h) { try { await fetch("http://" + h + ":8086/clear", { method: "POST" }); } catch { /* feed unreachable */ } }
});
mapEls.rotate.addEventListener("click", () => {
  mapRot = (mapRot + 1) % 4;
  try { localStorage.setItem("team16.mapRot", String(mapRot)); } catch { /* storage unavailable */ }
  drawMap();
});
window.addEventListener("resize", () => { if (mapDoc) drawMap(); });


const savedHost = localStorage.getItem(storageKey);
const pageHost = ["127.0.0.1", "localhost"].includes(location.hostname)
  ? ""
  : location.hostname;
hostInput.value = normalizeHost(queryHost || savedHost || pageHost);
if (hostInput.value) connectAll();

function updateClock() {
  document.querySelector("#clock").textContent = new Intl.DateTimeFormat([], {
    hour: "2-digit",
    minute: "2-digit",
    second: "2-digit",
    hour12: false,
  }).format(new Date());
}
updateClock();
setInterval(updateClock, 1000);
