const recordBtn = document.getElementById("startRecordingBtn");
const stopBtn = document.getElementById("stopRecordingBtn");
const missionBtn = document.getElementById("startMissionBtn");
const abortBtn = document.getElementById("abortMissionBtn");

const recordingStateValue = document.getElementById("recordingState");
const missionStateValue = document.getElementById("missionState");
const lastActionValue = document.getElementById("lastAction");
const apiStateValue = document.getElementById("apiState");
const liveStateText = document.getElementById("liveStateText");
const liveDot = document.getElementById("liveDot");
const logOutput = document.getElementById("logOutput");

const API_BASE_URL =
  (window.UGENT_SAILING_API_BASE_URL || "").trim() ||
  (window.location.protocol === "file:"
    ? "http://127.0.0.1:8000"
    : window.location.origin);

const state = {
  recording: false,
  mission: false,
  apiOnline: false,
  busy: false,
  statusMessage: "Waiting for mission command.",
};

function setBusy(isBusy) {
  state.busy = isBusy;
  recordBtn.disabled = isBusy || state.recording || !state.apiOnline;
  stopBtn.disabled = isBusy || !state.recording || !state.apiOnline;
  missionBtn.disabled = isBusy || state.mission || !state.apiOnline;
  abortBtn.disabled = isBusy || !state.mission || !state.apiOnline;
}

function stamp() {
  return new Date().toLocaleTimeString([], {
    hour: "2-digit",
    minute: "2-digit",
    second: "2-digit",
  });
}

function log(message) {
  const line = document.createElement("p");
  line.textContent = `[${stamp()}] ${message}`;
  logOutput.prepend(line);
}

async function apiRequest(path, options = {}) {
  const response = await fetch(`${API_BASE_URL}${path}`, {
    headers: {
      "Content-Type": "application/json",
      ...(options.headers || {}),
    },
    ...options,
  });

  const contentType = response.headers.get("content-type") || "";
  const payload = contentType.includes("application/json")
    ? await response.json().catch(() => null)
    : await response.text().catch(() => "");

  if (!response.ok) {
    const detail =
      payload && typeof payload === "object" && "detail" in payload
        ? payload.detail
        : typeof payload === "string" && payload
          ? payload
          : `HTTP ${response.status}`;
    throw new Error(detail);
  }

  return payload;
}

function updateFromStatus(payload) {
  if (!payload || typeof payload !== "object") {
    throw new Error("Invalid status response");
  }

  state.apiOnline = true;
  state.recording = Boolean(payload.recording);
  state.mission = Boolean(payload.mission);
  state.statusMessage =
    typeof payload.message === "string" && payload.message
      ? payload.message
      : state.recording || state.mission
        ? "Boat systems are active."
        : "Waiting for mission command.";

  apiStateValue.textContent = `Online at ${API_BASE_URL}`;
  lastActionValue.textContent = payload.last_action || "Status refreshed";

  refreshUI();
}

function refreshUI() {
  recordingStateValue.textContent = state.recording ? "Recording" : "Idle";
  missionStateValue.textContent = state.mission ? "Running" : "Standby";

  if (!state.apiOnline) {
    apiStateValue.textContent = `Offline - trying ${API_BASE_URL}`;
  }

  setBusy(state.busy);

  const active = state.recording || state.mission;
  liveStateText.textContent = state.apiOnline ? state.statusMessage : "Waiting for mission command.";
  liveDot.classList.toggle("on", active);
}

async function runAction(actionName, path, method = "POST") {
  setBusy(true);
  try {
    const payload = await apiRequest(path, { method });
    state.apiOnline = true;
    state.statusMessage =
      payload && typeof payload === "object" && typeof payload.message === "string" && payload.message
        ? payload.message
        : state.recording || state.mission
          ? "Boat systems are active."
          : "Waiting for mission command.";
    if (payload && typeof payload === "object") {
      if (typeof payload.recording === "boolean") {
        state.recording = payload.recording;
      }
      if (typeof payload.mission === "boolean") {
        state.mission = payload.mission;
      }
      if (typeof payload.last_action === "string") {
        lastActionValue.textContent = payload.last_action;
      } else {
        lastActionValue.textContent = actionName;
      }
      if (typeof payload.message === "string" && payload.message) {
        state.statusMessage = payload.message;
      }
    }
    log(`${actionName} succeeded.`);
    apiStateValue.textContent = `Online at ${API_BASE_URL}`;
    refreshUI();
  } catch (error) {
    state.apiOnline = false;
    apiStateValue.textContent = `Offline - ${error.message}`;
    log(`${actionName} failed: ${error.message}`);
    refreshUI();
  } finally {
    setBusy(false);
  }
}

recordBtn.addEventListener("click", () => {
  runAction("Start recording", "/api/recording/start");
});

stopBtn.addEventListener("click", () => {
  runAction("Stop recording", "/api/recording/stop");
});

missionBtn.addEventListener("click", () => {
  runAction("Start mission", "/api/mission/start");
});

abortBtn.addEventListener("click", () => {
  runAction("Abort mission", "/api/mission/abort");
});

refreshUI();
log("UGent Sailing dashboard initialized.");

async function pollStatus() {
  try {
    const payload = await apiRequest("/api/status", { method: "GET" });
    updateFromStatus(payload);
  } catch (error) {
    state.apiOnline = false;
    apiStateValue.textContent = `Offline - ${error.message}`;
    refreshUI();
  }
}

pollStatus();
setInterval(pollStatus, 2000);
