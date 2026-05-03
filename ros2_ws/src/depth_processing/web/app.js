"use strict";

const canvas = document.getElementById("mapCanvas");
const context = canvas.getContext("2d");
const mapStatus = document.getElementById("mapStatus");
const poseStatus = document.getElementById("poseStatus");
const planStatus = document.getElementById("planStatus");
const navStatus = document.getElementById("navStatus");
const readout = document.getElementById("readout");
const navigateButton = document.getElementById("navigateButton");
const stopButton = document.getElementById("stopButton");
const fitButton = document.getElementById("fitButton");
const clearButton = document.getElementById("clearButton");
const headingMode = document.getElementById("headingMode");
const headingInput = document.getElementById("headingInput");
const goalX = document.getElementById("goalX");
const goalY = document.getElementById("goalY");
const goalZ = document.getElementById("goalZ");
const pathCount = document.getElementById("pathCount");
const robotX = document.getElementById("robotX");
const robotY = document.getElementById("robotY");
const robotZ = document.getElementById("robotZ");
const robotYaw = document.getElementById("robotYaw");
const navDistance = document.getElementById("navDistance");
const navTime = document.getElementById("navTime");
const navRecoveries = document.getElementById("navRecoveries");

const mapImage = new Image();

let state = null;
let mapRevision = -1;
let mapReady = false;
let needsFit = true;
let lastPlanRequest = 0;
let selectedGoal = null;
let dragStart = null;
let viewport = {
  scale: 1,
  offsetX: 0,
  offsetY: 0
};

function clamp(value, minimum, maximum) {
  return Math.min(Math.max(value, minimum), maximum);
}

function meters(value) {
  if (!Number.isFinite(value)) {
    return "-";
  }
  return `${value.toFixed(2)} m`;
}

function degrees(rad) {
  if (!Number.isFinite(rad)) {
    return "-";
  }
  return `${(rad * 180 / Math.PI).toFixed(1)} deg`;
}

function seconds(value) {
  if (!Number.isFinite(value)) {
    return "-";
  }
  return `${value.toFixed(1)} s`;
}

function setChip(element, text, className) {
  element.textContent = text;
  element.className = `status-chip ${className || ""}`.trim();
}

function isNavigationActive(status) {
  return status === "sending" || status === "active" || status === "canceling";
}

function resizeCanvas() {
  const rect = canvas.getBoundingClientRect();
  const ratio = window.devicePixelRatio || 1;
  const width = Math.max(1, Math.floor(rect.width * ratio));
  const height = Math.max(1, Math.floor(rect.height * ratio));

  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
    if (needsFit && mapReady) {
      fitMap();
    }
  }

  draw();
}

function canvasSize() {
  const rect = canvas.getBoundingClientRect();
  return {
    width: rect.width,
    height: rect.height
  };
}

function fitMap() {
  if (!mapReady || mapImage.width <= 0 || mapImage.height <= 0) {
    return;
  }

  const size = canvasSize();
  const scaleX = size.width / mapImage.width;
  const scaleY = size.height / mapImage.height;
  viewport.scale = Math.max(0.05, Math.min(scaleX, scaleY) * 0.94);
  viewport.offsetX = (size.width - mapImage.width * viewport.scale) * 0.5;
  viewport.offsetY = (size.height - mapImage.height * viewport.scale) * 0.5;
  needsFit = false;
  draw();
}

function screenToImage(clientX, clientY) {
  const rect = canvas.getBoundingClientRect();
  return {
    x: (clientX - rect.left - viewport.offsetX) / viewport.scale,
    y: (clientY - rect.top - viewport.offsetY) / viewport.scale
  };
}

function imageToScreen(point) {
  return {
    x: viewport.offsetX + point.x * viewport.scale,
    y: viewport.offsetY + point.y * viewport.scale
  };
}

function mapMeta() {
  return state && state.map ? state.map : null;
}

function imagePointInMap(point) {
  const meta = mapMeta();
  return Boolean(
    meta &&
    point.x >= 0 &&
    point.x < meta.width &&
    point.y >= 0 &&
    point.y < meta.height
  );
}

function worldToImage(point) {
  const meta = mapMeta();
  if (!meta) {
    return null;
  }

  const origin = meta.origin;
  const dx = point.x - origin.x;
  const dy = point.y - origin.y;
  const cosYaw = Math.cos(origin.yaw);
  const sinYaw = Math.sin(origin.yaw);
  const gridX = (cosYaw * dx + sinYaw * dy) / meta.resolution;
  const gridY = (-sinYaw * dx + cosYaw * dy) / meta.resolution;
  return {
    x: gridX,
    y: meta.height - gridY
  };
}

function imageToWorld(point) {
  const meta = mapMeta();
  if (!meta) {
    return null;
  }

  const origin = meta.origin;
  const gridX = point.x;
  const gridY = meta.height - point.y;
  const mapX = gridX * meta.resolution;
  const mapY = gridY * meta.resolution;
  const cosYaw = Math.cos(origin.yaw);
  const sinYaw = Math.sin(origin.yaw);
  return {
    x: origin.x + cosYaw * mapX - sinYaw * mapY,
    y: origin.y + sinYaw * mapX + cosYaw * mapY
  };
}

function yawForGoal() {
  if (headingMode.value === "zero") {
    return 0;
  }
  if (headingMode.value === "custom") {
    return Number(headingInput.value || 0) * Math.PI / 180;
  }
  if (state && state.pose && Number.isFinite(state.pose.yaw)) {
    return state.pose.yaw;
  }
  return 0;
}

function drawMapPlaceholder(width, height) {
  context.fillStyle = "#e8edf1";
  context.fillRect(0, 0, width, height);
  context.fillStyle = "#637183";
  context.font = "14px system-ui, sans-serif";
  context.textAlign = "center";
  context.fillText("Waiting for /map", width * 0.5, height * 0.5);
}

function drawPath(points) {
  if (!points || points.length < 2) {
    return;
  }

  context.lineWidth = 3;
  context.lineJoin = "round";
  context.lineCap = "round";
  context.strokeStyle = "#2563eb";
  context.beginPath();

  let started = false;
  for (const point of points) {
    const imagePoint = worldToImage(point);
    if (!imagePoint) {
      continue;
    }
    const screenPoint = imageToScreen(imagePoint);
    if (!started) {
      context.moveTo(screenPoint.x, screenPoint.y);
      started = true;
    } else {
      context.lineTo(screenPoint.x, screenPoint.y);
    }
  }

  if (started) {
    context.stroke();
  }
}

function drawGoal(goal) {
  if (!goal) {
    return;
  }

  const imagePoint = worldToImage(goal);
  if (!imagePoint) {
    return;
  }
  const screenPoint = imageToScreen(imagePoint);

  context.save();
  context.translate(screenPoint.x, screenPoint.y);
  context.strokeStyle = "#d97706";
  context.fillStyle = "#fff7e8";
  context.lineWidth = 2;
  context.beginPath();
  context.arc(0, 0, 8, 0, Math.PI * 2);
  context.fill();
  context.stroke();
  context.beginPath();
  context.moveTo(-12, 0);
  context.lineTo(12, 0);
  context.moveTo(0, -12);
  context.lineTo(0, 12);
  context.stroke();
  context.restore();
}

function drawRobot(pose) {
  if (!pose) {
    return;
  }

  const imagePoint = worldToImage(pose);
  if (!imagePoint) {
    return;
  }
  const screenPoint = imageToScreen(imagePoint);
  const nosePoint = worldToImage({
    x: pose.x + Math.cos(pose.yaw) * 0.45,
    y: pose.y + Math.sin(pose.yaw) * 0.45
  });
  const noseScreen = nosePoint ? imageToScreen(nosePoint) : {
    x: screenPoint.x + 1,
    y: screenPoint.y
  };
  const angle = Math.atan2(
    noseScreen.y - screenPoint.y,
    noseScreen.x - screenPoint.x
  );

  context.save();
  context.translate(screenPoint.x, screenPoint.y);
  context.rotate(angle);
  context.fillStyle = "#0f766e";
  context.strokeStyle = "#ffffff";
  context.lineWidth = 2;
  context.beginPath();
  context.moveTo(14, 0);
  context.lineTo(-10, -9);
  context.lineTo(-6, 0);
  context.lineTo(-10, 9);
  context.closePath();
  context.fill();
  context.stroke();
  context.restore();
}

function draw() {
  const ratio = window.devicePixelRatio || 1;
  const width = canvas.width / ratio;
  const height = canvas.height / ratio;
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  context.clearRect(0, 0, width, height);

  if (!mapReady) {
    drawMapPlaceholder(width, height);
    return;
  }

  context.imageSmoothingEnabled = false;
  context.drawImage(
    mapImage,
    viewport.offsetX,
    viewport.offsetY,
    mapImage.width * viewport.scale,
    mapImage.height * viewport.scale
  );

  const path = state && state.path ? state.path.points : [];
  drawPath(path);
  drawGoal(selectedGoal || (state ? state.goal : null));
  drawRobot(state ? state.pose : null);
}

async function requestPlan(goal) {
  const requestId = Date.now();
  lastPlanRequest = requestId;
  setChip(planStatus, "planning", "warn");

  try {
    const response = await fetch("/api/plan", {
      method: "POST",
      headers: {
        "Content-Type": "application/json"
      },
      body: JSON.stringify(goal)
    });
    const payload = await response.json();
    if (!response.ok || !payload.ok) {
      throw new Error(payload.error || "Planning failed.");
    }
    if (lastPlanRequest === requestId) {
      selectedGoal = payload.goal;
      await refreshState();
    }
  } catch (error) {
    setChip(planStatus, error.message, "bad");
  }
}

async function clearPath() {
  await fetch("/api/clear_path", {
    method: "POST"
  });
  selectedGoal = null;
  await refreshState();
}

async function requestNavigation() {
  const goal = selectedGoal || (state ? state.goal : null);
  if (!goal) {
    setChip(navStatus, "no goal", "warn");
    return;
  }

  setChip(navStatus, "starting", "warn");
  try {
    const response = await fetch("/api/navigate", {
      method: "POST",
      headers: {
        "Content-Type": "application/json"
      },
      body: JSON.stringify(goal)
    });
    const payload = await response.json();
    if (!response.ok || !payload.ok) {
      throw new Error(payload.error || "Navigation failed to start.");
    }
    await refreshState();
  } catch (error) {
    setChip(navStatus, error.message, "bad");
  }
}

async function cancelNavigation() {
  setChip(navStatus, "canceling", "warn");
  try {
    const response = await fetch("/api/cancel_navigation", {
      method: "POST"
    });
    const payload = await response.json();
    if (!response.ok || !payload.ok) {
      throw new Error(payload.error || "Cancel failed.");
    }
    await refreshState();
  } catch (error) {
    setChip(navStatus, error.message, "bad");
  }
}

function updateMapImage(nextState) {
  if (!nextState.map || nextState.map_revision === mapRevision) {
    return;
  }

  mapRevision = nextState.map_revision;
  mapReady = false;
  mapImage.onload = () => {
    mapReady = true;
    fitMap();
    draw();
  };
  mapImage.src = `/api/map.png?rev=${mapRevision}`;
}

function updateMetrics() {
  if (!state) {
    return;
  }

  if (state.map) {
    const text = `${state.map.width}x${state.map.height} @ ${state.map.resolution.toFixed(3)} m`;
    setChip(mapStatus, text, "good");
  } else {
    setChip(mapStatus, "map waiting", "warn");
  }

  if (state.pose) {
    setChip(poseStatus, `pose ${meters(state.pose.x)}, ${meters(state.pose.y)}`, "good");
    robotX.textContent = meters(state.pose.x);
    robotY.textContent = meters(state.pose.y);
    robotZ.textContent = meters(state.pose.z);
    robotYaw.textContent = degrees(state.pose.yaw);
  } else {
    setChip(poseStatus, "pose waiting", "warn");
    robotX.textContent = "-";
    robotY.textContent = "-";
    robotZ.textContent = "-";
    robotYaw.textContent = "-";
  }

  if (state.planner.error) {
    setChip(planStatus, state.planner.error, "bad");
  } else if (state.planner.status === "idle") {
    setChip(planStatus, "planner idle", "");
  } else {
    setChip(planStatus, state.planner.status, "good");
  }

  const goal = selectedGoal || state.goal;
  if (goal) {
    goalX.textContent = meters(goal.x);
    goalY.textContent = meters(goal.y);
    goalZ.textContent = meters(goal.z);
  } else {
    goalX.textContent = "-";
    goalY.textContent = "-";
    goalZ.textContent = "-";
  }

  const points = state.path && state.path.points ? state.path.points.length : 0;
  pathCount.textContent = `${points} poses`;

  const navigation = state.navigation || {};
  const navFeedback = navigation.feedback || {};
  const navState = navigation.status || "idle";
  if (navigation.error) {
    setChip(navStatus, navigation.error, "bad");
  } else if (navState === "idle") {
    setChip(navStatus, "nav idle", "");
  } else if (navState === "active") {
    setChip(navStatus, "nav active", "good");
  } else if (navState === "succeeded") {
    setChip(navStatus, "nav complete", "good");
  } else if (navState === "canceled") {
    setChip(navStatus, "nav canceled", "warn");
  } else if (navState === "failed") {
    setChip(navStatus, "nav failed", "bad");
  } else {
    setChip(navStatus, `nav ${navState}`, "warn");
  }

  navDistance.textContent = meters(navFeedback.distance_remaining);
  navTime.textContent = seconds(navFeedback.navigation_time_sec);
  navRecoveries.textContent = Number.isFinite(navFeedback.number_of_recoveries)
    ? String(navFeedback.number_of_recoveries)
    : "-";

  const canNavigate = Boolean(goal) && points > 0 && !isNavigationActive(navState);
  navigateButton.disabled = !canNavigate;
  stopButton.disabled = !isNavigationActive(navState);
}

async function refreshState() {
  try {
    const response = await fetch("/api/state", {
      cache: "no-store"
    });
    state = await response.json();
    updateMapImage(state);
    updateMetrics();
    draw();
  } catch (error) {
    setChip(mapStatus, "bridge offline", "bad");
  }
}

canvas.addEventListener("pointerdown", (event) => {
  canvas.setPointerCapture(event.pointerId);
  dragStart = {
    x: event.clientX,
    y: event.clientY,
    offsetX: viewport.offsetX,
    offsetY: viewport.offsetY,
    moved: false
  };
});

canvas.addEventListener("pointermove", (event) => {
  const imagePoint = screenToImage(event.clientX, event.clientY);
  const worldPoint = imageToWorld(imagePoint);
  if (worldPoint && imagePointInMap(imagePoint)) {
    readout.textContent = `x ${worldPoint.x.toFixed(2)}, y ${worldPoint.y.toFixed(2)}`;
  } else if (mapReady) {
    readout.textContent = "outside map";
  }

  if (!dragStart) {
    return;
  }

  const dx = event.clientX - dragStart.x;
  const dy = event.clientY - dragStart.y;
  if (Math.hypot(dx, dy) > 4) {
    dragStart.moved = true;
    canvas.classList.add("dragging");
  }
  if (dragStart.moved) {
    viewport.offsetX = dragStart.offsetX + dx;
    viewport.offsetY = dragStart.offsetY + dy;
    draw();
  }
});

canvas.addEventListener("pointerup", (event) => {
  canvas.releasePointerCapture(event.pointerId);
  canvas.classList.remove("dragging");

  if (dragStart && !dragStart.moved && state && state.map) {
    const imagePoint = screenToImage(event.clientX, event.clientY);
    const worldPoint = imagePointInMap(imagePoint) ? imageToWorld(imagePoint) : null;
    if (worldPoint) {
      selectedGoal = {
        x: worldPoint.x,
        y: worldPoint.y,
        yaw: yawForGoal()
      };
      draw();
      requestPlan(selectedGoal);
    } else {
      setChip(planStatus, "goal outside map", "warn");
    }
  }
  dragStart = null;
});

canvas.addEventListener("pointercancel", () => {
  canvas.classList.remove("dragging");
  dragStart = null;
});

canvas.addEventListener("wheel", (event) => {
  if (!mapReady) {
    return;
  }

  event.preventDefault();
  const before = screenToImage(event.clientX, event.clientY);
  const zoomFactor = event.deltaY < 0 ? 1.12 : 0.89;
  viewport.scale = clamp(viewport.scale * zoomFactor, 0.04, 80);
  const rect = canvas.getBoundingClientRect();
  viewport.offsetX = event.clientX - rect.left - before.x * viewport.scale;
  viewport.offsetY = event.clientY - rect.top - before.y * viewport.scale;
  needsFit = false;
  draw();
}, { passive: false });

fitButton.addEventListener("click", () => {
  needsFit = true;
  fitMap();
});

clearButton.addEventListener("click", () => {
  clearPath();
});

navigateButton.addEventListener("click", () => {
  requestNavigation();
});

stopButton.addEventListener("click", () => {
  cancelNavigation();
});

headingMode.addEventListener("change", () => {
  headingInput.disabled = headingMode.value !== "custom";
});

window.addEventListener("resize", resizeCanvas);

headingInput.disabled = true;
resizeCanvas();
refreshState();
setInterval(refreshState, 700);
