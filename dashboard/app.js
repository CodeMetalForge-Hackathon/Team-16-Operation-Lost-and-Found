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
    if (action === "fullscreen") {
      card.querySelector(".viewport").requestFullscreen?.();
    }
  });
});

const queryHost = new URLSearchParams(location.search).get("host");
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
