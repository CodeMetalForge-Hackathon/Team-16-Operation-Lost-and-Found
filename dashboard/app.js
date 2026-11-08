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
  pollBeacon(host);
  pollSdr(host);
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

// ---------------------------------------------------------------- DF beacon telemetry (:8083)
const beaconEls = {
  state: document.querySelector("#beacon-state"), note: document.querySelector("#beacon-note"),
  link: document.querySelector("#b-link"), age: document.querySelector("#b-age"),
  rssi: document.querySelector("#b-rssi-now"), snr: document.querySelector("#b-snr"),
  loss: document.querySelector("#b-loss"), rate: document.querySelector("#b-rate"),
  uptime: document.querySelector("#b-uptime"), batt: document.querySelector("#b-batt"),
  sdr: document.querySelector("#b-sdr-now"), id: document.querySelector("#b-id"),
  sdrNote: document.querySelector("#b-sdr-note"), pings: document.querySelector("#b-pings"),
  retry: document.querySelector("#beacon-retry"),
};
let beaconTimer = null;
let beaconHost = "";
let beaconFails = 0;

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
  g.font = "10px ui-monospace, monospace";
  g.fillStyle = cssVar("--muted");
  g.strokeStyle = "rgba(148,180,169,0.14)";
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

function renderBeacon(d) {
  const linkText = { strong: "STRONG", ok: "OK", weak: "WEAK", sdr: "SDR DETECTED", lost: "LOST", none: "NO SIGNAL YET" }[d.link] || d.link;
  const linkCls = { strong: "good", ok: "good", weak: "warn", sdr: "good", lost: "bad", none: "bad" }[d.link];
  const live = d.link === "strong" || d.link === "ok" || d.link === "weak" || d.link === "sdr";
  beaconEls.state.dataset.state = live ? "live" : (d.link === "none" ? "idle" : "error");
  beaconEls.state.textContent = d.link === "sdr" ? "Receiving (SDR)" : live ? "Receiving" : (d.link === "none" ? "Waiting" : "Lost");
  setTile(beaconEls.link, linkText, linkCls);
  const heard = d.link === "sdr" && d.sdr ? d.sdr.last_detect_age_s : d.age_s;
  setTile(beaconEls.age, heard == null ? "—" : heard + " s ago" + (d.link === "sdr" ? " (SDR)" : ""), heard != null && heard > 10 ? "bad" : "");
  const L = d.latest, S = d.stats;
  if (L && d.link !== "sdr") setTile(beaconEls.rssi, L.rssi + " dBm", linkCls);
  else if (d.sdr) setTile(beaconEls.rssi, d.sdr.power_dbm + " " + (d.sdr.unit || "dBm") + " (SDR)", linkCls);
  else setTile(beaconEls.rssi, "—", linkCls);
  setTile(beaconEls.snr, L ? L.snr + " dB" : "—", L && L.snr < 0 ? "warn" : "");
  const loss = S.loss_pct_60s;
  setTile(beaconEls.loss, loss == null ? "—" : loss + " %", loss == null ? "" : loss > 25 ? "bad" : loss > 8 ? "warn" : "good");
  setTile(beaconEls.rate, String(S.rate_per_min));
  setTile(beaconEls.uptime, L ? fmtDuration(L.uptime) : "—");
  if (L && L.mv) {
    const pct = Math.max(0, Math.min(100, Math.round((L.mv - 3300) / 9)));
    setTile(beaconEls.batt, (L.mv / 1000).toFixed(2) + " V (" + pct + "%)", pct < 15 ? "bad" : pct < 35 ? "warn" : "good");
  } else {
    setTile(beaconEls.batt, "—");
  }
  setTile(beaconEls.id, L ? L.id : "—");
  if (d.sdr) {
    const u = d.sdr.unit || "dBm";
    setTile(beaconEls.sdr, d.sdr.power_dbm + " " + u, d.sdr.detected ? "good" : "");
    beaconEls.sdrNote.textContent = d.sdr.detected
      ? "beacon detected, +" + d.sdr.over_floor_db + " dB over the noise floor"
      : "noise floor " + d.sdr.noise_floor_dbm + " " + u + ", no beacon seen";
  } else {
    setTile(beaconEls.sdr, "—");
    beaconEls.sdrNote.textContent = "no SDR connected";
  }
  beaconEls.note.className = "cal-status " + (live ? "ok" : "bad");
  beaconEls.note.textContent = "Sources: " + (d.sources.length ? d.sources.join(", ") : "none") +
    " · received " + S.received_total + ", missed " + S.lost_total +
    (S.rssi_best_5min != null ? " · best signal (5 min) " + S.rssi_best_5min + " dBm" : "");
  beaconEls.pings.replaceChildren();
  if (!d.packets.length) {
    const li = document.createElement("li");
    li.className = "muted"; li.textContent = "Waiting for the first ping…";
    beaconEls.pings.append(li);
  }
  for (const p of d.packets) {
    const li = document.createElement("li");
    if (p.lost_before) li.className = "gap";
    const when = new Date(p.t * 1000).toLocaleTimeString([], { hour12: false });
    const left = document.createElement("span");
    left.textContent = when + " #" + p.seq + (p.lost_before ? " (" + p.lost_before + " missed)" : "");
    const right = document.createElement("span");
    right.textContent = p.rssi + " dBm";
    li.append(left, right);
    beaconEls.pings.append(li);
  }
  drawChart(document.querySelector("#c-rssi"), d.history.map((p) => ({ t: p.t, v: p.rssi })),
            { range: [-130, -70], step: 10, color: "--green", gap: 6, dots: true, empty: "no pings yet" });
  const sd = d.sdr ? d.sdr.history.map((p) => ({ t: p.t, v: p.p })) : [];
  const vals = sd.map((p) => p.v);
  const lo = vals.length ? Math.floor((Math.min(...vals) - 3) / 10) * 10 : -120;
  const hi = vals.length ? Math.max(lo + 30, Math.ceil((Math.max(...vals) + 3) / 10) * 10) : -60;
  drawChart(document.querySelector("#c-sdr"), sd,
            { range: [lo, hi], step: 10, color: "--cyan", gap: 3, dots: false, floor: d.sdr ? d.sdr.noise_floor_dbm : null, empty: "no SDR data" });
}

async function pollBeacon(host) {
  clearTimeout(beaconTimer);
  beaconHost = host;
  try {
    const response = await fetch("http://" + host + ":8083/beacon", { cache: "no-store" });
    if (!response.ok) throw new Error("HTTP " + response.status);
    beaconFails = 0;
    renderBeacon(await response.json());
  } catch {
    beaconFails += 1;
    beaconEls.state.dataset.state = beaconFails > 2 ? "error" : "connecting";
    beaconEls.state.textContent = beaconFails > 2 ? "Unavailable" : "Connecting";
    beaconEls.note.className = "cal-status bad";
    beaconEls.note.textContent = "Could not reach the beacon feed on the Pi (port 8083)";
  }
  if (beaconHost === host) beaconTimer = setTimeout(() => pollBeacon(host), 1000);
}

beaconEls.retry.addEventListener("click", () => {
  const host = normalizeHost(hostInput.value);
  if (host) { beaconFails = 0; pollBeacon(host); } else hostInput.focus();
});

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
  g.font = "10px ui-monospace, monospace";
  g.strokeStyle = "rgba(148,180,169,0.14)"; g.fillStyle = cssVar("--muted"); g.lineWidth = 1;
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
    g.strokeStyle = "rgba(255,189,91,0.6)"; g.setLineDash([4, 4]);
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
// ---- Lidar room map (:8086, posted by the lidar laptop). Lidar positions + map only, no fusion with the INS.
const mapEls = {
  state: document.querySelector("#map-state"), note: document.querySelector("#map-note"),
  lock: document.querySelector("#m-lock"), xy: document.querySelector("#m-xy"), z: document.querySelector("#m-z"),
  hdg: document.querySelector("#m-hdg"), dist: document.querySelector("#m-dist"),
  canvas: document.querySelector("#c-map"), retry: document.querySelector("#map-retry"), clear: document.querySelector("#map-clear"),
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
    g.fillStyle = cssVar("--muted"); g.font = "13px ui-monospace, monospace";
    g.fillText("Waiting for the lidar laptop to post a map…", 16, 28);
    return;
  }
  const mpp = d.meters_per_pixel || 0.1, ox = d.origin?.px ?? 0, oy = d.origin?.py ?? 0;
  const cur0 = readPose(d.pose);
  // view = the map grid plus anywhere the track has been, so the position never falls off the plot
  let x0 = -ox * mpp, x1 = (d.width - ox) * mpp, y0 = (oy - d.height) * mpp, y1 = oy * mpp;
  for (const p of cur0 ? [...mapTrail, cur0] : mapTrail) { x0 = Math.min(x0, p.x - 0.5); x1 = Math.max(x1, p.x + 0.5); y0 = Math.min(y0, p.y - 0.5); y1 = Math.max(y1, p.y + 0.5); }
  const pad = 14, scale = Math.min((w - 2 * pad) / (x1 - x0), (h - 2 * pad) / (y1 - y0));   // px per metre
  const left = (w - (x1 - x0) * scale) / 2, top = (h - (y1 - y0) * scale) / 2;
  const mx = (xm) => left + (xm - x0) * scale, my = (ym) => top + (y1 - ym) * scale;
  const cell = mpp * scale;
  g.strokeStyle = "rgba(148,180,169,0.2)"; g.strokeRect(left, top, (x1 - x0) * scale, (y1 - y0) * scale);
  g.strokeStyle = "rgba(148,180,169,0.12)"; g.fillStyle = cssVar("--muted"); g.font = "10px ui-monospace, monospace";
  for (let e = Math.ceil(x0); e <= x1; e++) { g.beginPath(); g.moveTo(mx(e), top); g.lineTo(mx(e), top + (y1 - y0) * scale); g.stroke(); }
  for (let n = Math.ceil(y0); n <= y1; n++) { g.beginPath(); g.moveTo(left, my(n)); g.lineTo(left + (x1 - x0) * scale, my(n)); g.stroke(); }
  g.fillText("1 m grid", left + 4, top + (y1 - y0) * scale - 4);
  g.fillStyle = "rgba(237,247,243,0.85)";
  for (const [col, row] of d.outline || []) {
    const xm = (col - ox) * mpp, ym = (oy - row) * mpp;   // cell's lower-left corner, metres
    g.fillRect(mx(xm), my(ym) - cell, Math.max(1.5, cell), Math.max(1.5, cell));
  }
  g.fillStyle = cssVar("--green"); g.beginPath(); g.arc(mx(0), my(0), 4, 0, 6.3); g.fill();   // map origin
  g.strokeStyle = cssVar("--cyan"); g.lineWidth = 2; g.beginPath();
  mapTrail.forEach((p, i) => (i ? g.lineTo(mx(p.x), my(p.y)) : g.moveTo(mx(p.x), my(p.y))));
  g.stroke();
  const cur = readPose(d.pose);
  if (cur) {
    const X = mx(cur.x), Y = my(cur.y);
    if (cur.yawCw != null) {   // heading is clockwise from +X (map right); canvas Y points down so clockwise is positive
      const a = (cur.yawCw * Math.PI) / 180, len = 16;
      g.strokeStyle = cssVar("--red"); g.lineWidth = 2.5; g.beginPath();
      g.moveTo(X, Y); g.lineTo(X + Math.cos(a) * len, Y + Math.sin(a) * len); g.stroke();
    }
    g.fillStyle = cssVar("--red"); g.beginPath(); g.arc(X, Y, 6, 0, 6.3); g.fill();
  }
}

function renderMap(d) {
  mapDoc = d;
  const cur = d.locked === false ? null : readPose(d.pose);
  if (cur) {
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
  mapEls.hdg.textContent = cur && cur.yawCw != null ? (((cur.yawCw % 360) + 360) % 360).toFixed(0) + "°" : "—";
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
mapEls.clear.addEventListener("click", () => { mapTrail = []; mapDist = 0; mapLast = null; drawMap(); });
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
