"use strict";

const canvas = document.getElementById("mapCanvas");
const context = canvas.getContext("2d");
const mapStatus = document.getElementById("mapStatus");
const poseStatus = document.getElementById("poseStatus");
const planStatus = document.getElementById("planStatus");
const navStatus = document.getElementById("navStatus");
const manualStatus = document.getElementById("manualStatus");
const personStatus = document.getElementById("personStatus");
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
const manualCanvas = document.getElementById("manualCanvas");
const manualContext = manualCanvas.getContext("2d");
const manualCcwButton = document.getElementById("manualCcwButton");
const manualStopButton = document.getElementById("manualStopButton");
const manualCwButton = document.getElementById("manualCwButton");
const manualCommand = document.getElementById("manualCommand");
const manualWheels = document.getElementById("manualWheels");
const personTrackingToggle = document.getElementById("personTrackingToggle");
const personImage = document.getElementById("personImage");
const personImagePlaceholder = document.getElementById("personImagePlaceholder");
const personCount = document.getElementById("personCount");
const personFrameAge = document.getElementById("personFrameAge");

const mapImage = new Image();
const SQRT_3 = Math.sqrt(3);
const GAMEPAD_AXIS_DEADBAND = 0.12;
const GAMEPAD_ACTIVE_EPSILON = 0.01;
const GAMEPAD_BOTTOM_BUTTON = 0;
const GAMEPAD_LEFT_TRIGGER = 6;
const GAMEPAD_RIGHT_TRIGGER = 7;
const MANUAL_SPIN_POWER = 0.75;
const manualWheelDefs = [
  {
    key: "topLeft",
    coord: [-SQRT_3 / 2, 0.5],
    direction: [-0.5, -SQRT_3 / 2]
  },
  {
    key: "topRight",
    coord: [SQRT_3 / 2, 0.5],
    direction: [-0.5, SQRT_3 / 2]
  },
  {
    key: "bottom",
    coord: [0, -1],
    direction: [1, 0]
  }
];

let state = null;
let mapRevision = -1;
let personImageRevision = -1;
let mapReady = false;
let needsFit = true;
let lastPlanRequest = 0;
let personTrackingRequestPending = false;
let selectedGoal = null;
let dragStart = null;
let manualPointerId = null;
let manualRotation = 0;
let manualSendPending = false;
let gamepadIndex = null;
let gamepadManualActive = false;
let gamepadStopLatched = false;
let gamepadStopButtonDown = false;
let manualInputSource = "idle";
let manualState = {
  x: 0,
  y: 0,
  angular: 0,
  wheels: [0, 0, 0]
};
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

function compactNumber(value) {
  if (!Number.isFinite(value)) {
    return "0.00";
  }
  return value.toFixed(2);
}

function statusErrorText(error, fallback) {
  const message = error && error.message ? String(error.message) : "";
  if (!message) {
    return fallback;
  }
  if (
    /expected pattern|failed to fetch|load failed|networkerror|network error/i
      .test(message)
  ) {
    return fallback;
  }
  if (message.length > 34) {
    return fallback;
  }
  return message;
}

function setChip(element, text, className) {
  element.textContent = text;
  element.className = `status-chip ${className || ""}`.trim();
}

function isNavigationActive(status) {
  return status === "sending" || status === "active" || status === "canceling";
}

function manualLinearActive() {
  return Math.hypot(manualState.x, manualState.y) > 0.01;
}

function dot2(a, b) {
  return a[0] * b[0] + a[1] * b[1];
}

function cross2d(a, b) {
  return a[0] * b[1] - a[1] * b[0];
}

function normalIntersection(a, b) {
  const abCross = cross2d(a, b);
  if (Math.abs(abCross) < 1e-6) {
    return a;
  }
  const intersectionBScalar = dot2(a, [b[0] - a[0], b[1] - a[1]]) / abCross;
  return [
    b[0] + intersectionBScalar * -b[1],
    b[1] + intersectionBScalar * b[0]
  ];
}

function manualWheelPowers(x, y, angular) {
  return manualWheelDefs.map((wheel) => dot2([x, y], wheel.direction) + angular);
}

function setManualState(x, y, angular) {
  const magnitude = Math.hypot(x, y);
  let nextX = x;
  let nextY = y;
  if (magnitude > 1) {
    nextX /= magnitude;
    nextY /= magnitude;
  }
  manualState = {
    x: clamp(nextX, -1, 1),
    y: clamp(nextY, -1, 1),
    angular: clamp(angular, -1, 1),
    wheels: manualWheelPowers(
      clamp(nextX, -1, 1),
      clamp(nextY, -1, 1),
      clamp(angular, -1, 1)
    )
  };
  updateManualMetrics();
  drawManualControl();
}

function setManualInputSource(source) {
  manualInputSource = source;
}

function updateManualMetrics() {
  manualCommand.textContent = [
    compactNumber(manualState.x),
    compactNumber(manualState.y),
    compactNumber(manualState.angular)
  ].join(", ");
  manualWheels.textContent = manualState.wheels.map(compactNumber).join(", ");
}

function manualPointToScreen(point, center, scale) {
  return {
    x: center.x + point[0] * scale,
    y: center.y - point[1] * scale
  };
}

function drawManualLine(start, end, color, width = 2) {
  manualContext.save();
  manualContext.strokeStyle = color;
  manualContext.lineWidth = width;
  manualContext.lineCap = "round";
  manualContext.lineJoin = "round";
  manualContext.beginPath();
  manualContext.moveTo(start.x, start.y);
  manualContext.lineTo(end.x, end.y);
  manualContext.stroke();
  manualContext.restore();
}

function drawManualArrow(start, end, color, width = 3, headSize = 8) {
  const dx = end.x - start.x;
  const dy = end.y - start.y;
  const length = Math.hypot(dx, dy);
  if (length < 1) {
    return;
  }

  drawManualLine(start, end, color, width);
  if (length < headSize * 1.8) {
    return;
  }

  const angle = Math.atan2(dy, dx);
  manualContext.save();
  manualContext.fillStyle = color;
  manualContext.beginPath();
  manualContext.moveTo(end.x, end.y);
  manualContext.lineTo(
    end.x - Math.cos(angle - 0.55) * headSize,
    end.y - Math.sin(angle - 0.55) * headSize
  );
  manualContext.lineTo(
    end.x - Math.cos(angle + 0.55) * headSize,
    end.y - Math.sin(angle + 0.55) * headSize
  );
  manualContext.closePath();
  manualContext.fill();
  manualContext.restore();
}

function drawManualCircle(point, radius, fill, stroke, width = 1) {
  manualContext.save();
  if (fill) {
    manualContext.fillStyle = fill;
  }
  if (stroke) {
    manualContext.strokeStyle = stroke;
    manualContext.lineWidth = width;
  }
  manualContext.beginPath();
  manualContext.arc(point.x, point.y, radius, 0, Math.PI * 2);
  if (fill) {
    manualContext.fill();
  }
  if (stroke) {
    manualContext.stroke();
  }
  manualContext.restore();
}

function drawManualBackground(center, scale, width, height) {
  const background = manualContext.createRadialGradient(
    center.x,
    center.y,
    scale * 0.08,
    center.x,
    center.y,
    scale * 1.5
  );
  background.addColorStop(0, "#1c2935");
  background.addColorStop(0.68, "#14202a");
  background.addColorStop(1, "#101720");
  manualContext.fillStyle = background;
  manualContext.fillRect(0, 0, width, height);

  manualContext.save();
  manualContext.strokeStyle = "rgba(148, 163, 184, 0.12)";
  manualContext.lineWidth = 1;
  const spacing = Math.max(18, scale * 0.22);
  for (let x = center.x % spacing; x < width; x += spacing) {
    manualContext.beginPath();
    manualContext.moveTo(x, 0);
    manualContext.lineTo(x, height);
    manualContext.stroke();
  }
  for (let y = center.y % spacing; y < height; y += spacing) {
    manualContext.beginPath();
    manualContext.moveTo(0, y);
    manualContext.lineTo(width, y);
    manualContext.stroke();
  }
  manualContext.restore();

  const base = manualContext.createRadialGradient(
    center.x - scale * 0.18,
    center.y - scale * 0.22,
    scale * 0.05,
    center.x,
    center.y,
    scale
  );
  base.addColorStop(0, "#243241");
  base.addColorStop(1, "#111820");
  drawManualCircle(center, scale, base, "#f8fafc", 2);

  manualContext.save();
  manualContext.strokeStyle = "rgba(248, 250, 252, 0.16)";
  manualContext.lineWidth = 1;
  for (const radius of [scale * 0.33, scale * 0.66]) {
    manualContext.beginPath();
    manualContext.arc(center.x, center.y, radius, 0, Math.PI * 2);
    manualContext.stroke();
  }
  manualContext.restore();

  drawManualLine(
    { x: center.x - scale, y: center.y },
    { x: center.x + scale, y: center.y },
    "rgba(248, 250, 252, 0.18)",
    1
  );
  drawManualLine(
    { x: center.x, y: center.y - scale },
    { x: center.x, y: center.y + scale },
    "rgba(248, 250, 252, 0.18)",
    1
  );
}

function drawManualRotationArc(center, scale) {
  const angular = clamp(manualState.angular, -1, 1);
  if (Math.abs(angular) <= GAMEPAD_ACTIVE_EPSILON) {
    return;
  }

  const radius = scale * 0.78;
  const start = -Math.PI / 2;
  const end = start - angular * Math.PI * 1.35;
  manualContext.save();
  manualContext.strokeStyle = "#f97316";
  manualContext.lineWidth = 5;
  manualContext.lineCap = "round";
  manualContext.shadowColor = "rgba(249, 115, 22, 0.32)";
  manualContext.shadowBlur = 8;
  manualContext.beginPath();
  manualContext.arc(center.x, center.y, radius, start, end, angular > 0);
  manualContext.stroke();
  manualContext.restore();
}

function drawManualDefaultForwardMarker(center, ringRadius) {
  const tip = {
    x: center.x,
    y: center.y - ringRadius - 6
  };
  const left = {
    x: center.x - 7,
    y: center.y - ringRadius + 7
  };
  const right = {
    x: center.x + 7,
    y: center.y - ringRadius + 7
  };

  manualContext.save();
  manualContext.shadowColor = "rgba(249, 115, 22, 0.26)";
  manualContext.shadowBlur = 7;
  manualContext.fillStyle = "#f8fafc";
  manualContext.strokeStyle = "#f97316";
  manualContext.lineWidth = 2;
  manualContext.beginPath();
  manualContext.moveTo(tip.x, tip.y);
  manualContext.lineTo(left.x, left.y);
  manualContext.lineTo(center.x, center.y - ringRadius + 3);
  manualContext.lineTo(right.x, right.y);
  manualContext.closePath();
  manualContext.fill();
  manualContext.stroke();
  manualContext.restore();
}

function drawManualWheelPad(wheel, center, scale, power) {
  const coord = wheel.coord;
  const direction = wheel.direction;
  const padStart = manualPointToScreen(
    [coord[0] - direction[0] * 0.32, coord[1] - direction[1] * 0.32],
    center,
    scale
  );
  const padEnd = manualPointToScreen(
    [coord[0] + direction[0] * 0.32, coord[1] + direction[1] * 0.32],
    center,
    scale
  );
  drawManualLine(padStart, padEnd, "#263443", 12);
  drawManualLine(padStart, padEnd, "rgba(248, 250, 252, 0.38)", 2);

  const displayPower = clamp(power, -1, 1);
  if (Math.abs(displayPower) <= GAMEPAD_ACTIVE_EPSILON) {
    return;
  }

  const wheelCenter = manualPointToScreen(coord, center, scale);
  const arrowEnd = manualPointToScreen(
    [
      coord[0] + direction[0] * displayPower * 0.54,
      coord[1] + direction[1] * displayPower * 0.54
    ],
    center,
    scale
  );
  drawManualArrow(
    wheelCenter,
    arrowEnd,
    displayPower > 0 ? "#22c55e" : "#60a5fa",
    4,
    8
  );
}

function manualOrientationState() {
  return state && state.manual && state.manual.orientation
    ? state.manual.orientation
    : {};
}

function robotForwardVector(orientation) {
  if (!Number.isFinite(orientation.relative_yaw)) {
    return null;
  }
  return [
    -Math.sin(orientation.relative_yaw),
    Math.cos(orientation.relative_yaw)
  ];
}

function drawManualHeadingIndicator(center, scale, width, height) {
  const orientation = manualOrientationState();
  const ringRadius = Math.min(
    scale * 1.24,
    center.x - 8,
    width - center.x - 8,
    center.y - 8,
    height - center.y - 8
  );
  if (ringRadius <= scale + 4) {
    return;
  }

  manualContext.strokeStyle = orientation.enabled ? "#7c8794" : "#3a4652";
  manualContext.lineWidth = 1.5;
  manualContext.beginPath();
  manualContext.arc(center.x, center.y, ringRadius, 0, Math.PI * 2);
  manualContext.stroke();

  for (let index = 0; index < 24; index += 1) {
    const angle = index * Math.PI / 12;
    const major = index % 6 === 0;
    const tickVector = [Math.sin(angle), Math.cos(angle)];
    const innerRadius = ringRadius / scale - (major ? 0.08 : 0.045);
    const outerRadius = ringRadius / scale + 0.02;
    drawManualLine(
      manualPointToScreen(
        [tickVector[0] * innerRadius, tickVector[1] * innerRadius],
        center,
        scale
      ),
      manualPointToScreen(
        [tickVector[0] * outerRadius, tickVector[1] * outerRadius],
        center,
        scale
      ),
      major ? "#f8fafc" : "rgba(248, 250, 252, 0.42)",
      major ? 2 : 1
    );
  }

  drawManualDefaultForwardMarker(center, ringRadius);

  const forward = robotForwardVector(orientation);
  if (!forward) {
    return;
  }

  const markerColor = orientation.fresh ? "#f97316" : "#94a3b8";
  const markerStart = [
    forward[0] * (ringRadius / scale - 0.12),
    forward[1] * (ringRadius / scale - 0.12)
  ];
  const markerEnd = [
    forward[0] * (ringRadius / scale + 0.02),
    forward[1] * (ringRadius / scale + 0.02)
  ];
  drawManualLine(
    manualPointToScreen(markerStart, center, scale),
    manualPointToScreen(markerEnd, center, scale),
    markerColor,
    4
  );
  const markerPoint = manualPointToScreen(markerEnd, center, scale);
  manualContext.fillStyle = markerColor;
  manualContext.strokeStyle = "#121a22";
  manualContext.lineWidth = 2;
  manualContext.beginPath();
  manualContext.arc(markerPoint.x, markerPoint.y, 5, 0, Math.PI * 2);
  manualContext.fill();
  manualContext.stroke();
}

function syncManualCanvasSize() {
  const rect = manualCanvas.getBoundingClientRect();
  const cssSize = Math.max(1, rect.width || manualCanvas.width);
  manualCanvas.style.height = `${cssSize}px`;

  const ratio = window.devicePixelRatio || 1;
  const width = Math.floor(cssSize * ratio);
  const height = Math.floor(cssSize * ratio);

  if (manualCanvas.width !== width || manualCanvas.height !== height) {
    manualCanvas.width = width;
    manualCanvas.height = height;
  }
  manualContext.setTransform(ratio, 0, 0, ratio, 0, 0);

  return {
    width: cssSize,
    height: cssSize
  };
}

function drawManualControl() {
  const { width, height } = syncManualCanvasSize();
  const center = { x: width * 0.5, y: height * 0.52 };
  const scale = Math.min(width, height) * 0.36;
  const velocity = [manualState.x, manualState.y];
  const wheelVectors = manualWheelDefs.map((wheel, index) => [
    clamp(manualState.wheels[index], -1, 1) * wheel.direction[0] * 0.58,
    clamp(manualState.wheels[index], -1, 1) * wheel.direction[1] * 0.58
  ]);
  const linearActive = Math.hypot(velocity[0], velocity[1]) > 0.01;
  const angularActive = Math.abs(manualState.angular) > 0.01;

  manualContext.clearRect(0, 0, width, height);
  drawManualBackground(center, scale, width, height);
  drawManualHeadingIndicator(center, scale, width, height);
  drawManualRotationArc(center, scale);

  manualWheelDefs.forEach((wheel, index) => {
    drawManualWheelPad(wheel, center, scale, manualState.wheels[index]);
  });

  if (linearActive) {
    drawManualArrow(
      center,
      manualPointToScreen(velocity, center, scale),
      "#f97316",
      5,
      11
    );
  }
  drawManualThumb(center, manualPointToScreen(velocity, center, scale));

  if (linearActive || angularActive) {
    const intersections = [
      normalIntersection(wheelVectors[2], wheelVectors[0]),
      normalIntersection(wheelVectors[0], wheelVectors[1]),
      normalIntersection(wheelVectors[2], wheelVectors[1])
    ];
    manualContext.save();
    manualContext.fillStyle = "rgba(125, 211, 252, 0.18)";
    manualContext.strokeStyle = "rgba(125, 211, 252, 0.7)";
    manualContext.lineWidth = 1.5;
    intersections.forEach((point) => {
      if (!Number.isFinite(point[0]) || !Number.isFinite(point[1])) {
        return;
      }
      const screenPoint = manualPointToScreen(point, center, scale);
      manualContext.beginPath();
      manualContext.arc(screenPoint.x, screenPoint.y, 3.5, 0, Math.PI * 2);
      manualContext.fill();
      manualContext.stroke();
    });
    manualContext.restore();
  }
}

function drawManualThumb(center, point) {
  const active = Math.hypot(manualState.x, manualState.y) > 0.01;
  const fill = manualInputSource === "gamepad" ? "#f97316" : "#2563eb";

  drawManualCircle(center, 5, "#f8fafc", "#121a22", 2);
  drawManualCircle(center, 2, "#121a22", null);

  manualContext.save();
  if (active) {
    manualContext.shadowColor = manualInputSource === "gamepad"
      ? "rgba(249, 115, 22, 0.42)"
      : "rgba(37, 99, 235, 0.36)";
    manualContext.shadowBlur = 10;
  }
  drawManualCircle(
    point,
    active ? 11 : 8,
    active ? fill : "#94a3b8",
    "#f8fafc",
    3
  );
  manualContext.restore();
  drawManualCircle(point, active ? 4 : 3, "rgba(255, 255, 255, 0.72)", null);
}

async function sendManualCommand(force = false) {
  if (manualSendPending && !force) {
    return;
  }
  manualSendPending = true;
  try {
    const response = await fetch("/api/manual_cmd", {
      method: "POST",
      headers: {
        "Content-Type": "application/json"
      },
      body: JSON.stringify({
        x: manualState.x,
        y: manualState.y,
        angular: manualState.angular
      })
    });
    const payload = await response.json();
    if (!response.ok || !payload.ok) {
      throw new Error(payload.error || "Manual command failed.");
    }
    setChip(manualStatus, "manual active", "good");
  } catch (error) {
    setChip(manualStatus, statusErrorText(error, "manual offline"), "bad");
  } finally {
    manualSendPending = false;
  }
}

function stopManualCommand() {
  manualPointerId = null;
  manualRotation = 0;
  gamepadManualActive = false;
  gamepadStopLatched = true;
  setManualInputSource("idle");
  setManualState(0, 0, 0);
  sendManualCommand(true);
  setChip(manualStatus, "manual idle", "");
}

function manualCommandFromPointer(event) {
  const rect = manualCanvas.getBoundingClientRect();
  const x = (event.clientX - rect.left) / rect.width;
  const y = (event.clientY - rect.top) / rect.height;
  const centeredX = x * 2 - 1;
  const centeredY = -(y * 2 - 1);
  const deadband = 0.04;
  if (Math.hypot(centeredX, centeredY) < deadband) {
    return [0, 0];
  }
  return [centeredX, centeredY];
}

function updateManualFromPointer(event) {
  const [x, y] = manualCommandFromPointer(event);
  gamepadManualActive = false;
  gamepadStopLatched = false;
  setManualInputSource(Math.hypot(x, y) > 0.01 ? "pointer" : "idle");
  setManualState(x, y, manualRotation);
  sendManualCommand();
}

function setManualRotation(direction) {
  manualRotation = clamp(direction, -1, 1);
  setManualState(manualState.x, manualState.y, manualRotation);
  sendManualCommand(true);
}

function gamepadAxis(value) {
  if (!Number.isFinite(value)) {
    return 0;
  }
  const magnitude = Math.abs(value);
  if (magnitude <= GAMEPAD_AXIS_DEADBAND) {
    return 0;
  }
  const scaled = (magnitude - GAMEPAD_AXIS_DEADBAND) / (1 - GAMEPAD_AXIS_DEADBAND);
  return Math.sign(value) * clamp(scaled, 0, 1);
}

function gamepadHasLeftStick(gamepad) {
  return Boolean(gamepad && gamepad.connected && gamepad.axes.length >= 2);
}

function gamepadButtonPressed(gamepad, index) {
  const button = gamepad.buttons[index];
  return Boolean(button && button.pressed);
}

function gamepadTriggerValue(gamepad, index) {
  const button = gamepad.buttons[index];
  if (!button) {
    return 0;
  }
  const value = Number.isFinite(button.value)
    ? button.value
    : (button.pressed ? 1 : 0);
  if (value <= GAMEPAD_AXIS_DEADBAND) {
    return 0;
  }
  return clamp(
    (value - GAMEPAD_AXIS_DEADBAND) / (1 - GAMEPAD_AXIS_DEADBAND),
    0,
    1
  );
}

function setManualControllerButtonState(ccwPressed, cwPressed, stopPressed) {
  manualCcwButton.classList.toggle("controller-pressed", ccwPressed);
  manualCwButton.classList.toggle("controller-pressed", cwPressed);
  manualStopButton.classList.toggle("controller-pressed", stopPressed);
}

function connectedGamepads() {
  if (!navigator.getGamepads) {
    return [];
  }
  return Array.from(navigator.getGamepads()).filter(gamepadHasLeftStick);
}

function activeGamepad() {
  const pads = connectedGamepads();
  if (gamepadIndex !== null) {
    const selected = pads.find((gamepad) => gamepad.index === gamepadIndex);
    if (selected) {
      return selected;
    }
  }
  const standard = pads.find((gamepad) => gamepad.mapping === "standard");
  const selected = standard || pads[0] || null;
  gamepadIndex = selected ? selected.index : null;
  return selected;
}

function pollGamepadManualControl() {
  if (manualPointerId !== null) {
    return false;
  }

  const gamepad = activeGamepad();
  if (!gamepad) {
    setManualControllerButtonState(false, false, false);
    gamepadStopButtonDown = false;
    if (gamepadManualActive) {
      gamepadManualActive = false;
      manualRotation = 0;
      setManualInputSource("idle");
      setManualState(0, 0, 0);
      sendManualCommand(true);
    }
    return false;
  }

  const x = gamepadAxis(gamepad.axes[0]);
  const y = -gamepadAxis(gamepad.axes[1]);
  const ccwTrigger = gamepadTriggerValue(gamepad, GAMEPAD_LEFT_TRIGGER);
  const cwTrigger = gamepadTriggerValue(gamepad, GAMEPAD_RIGHT_TRIGGER);
  const ccwPressed = ccwTrigger > GAMEPAD_ACTIVE_EPSILON;
  const cwPressed = cwTrigger > GAMEPAD_ACTIVE_EPSILON;
  const stopPressed = gamepadButtonPressed(gamepad, GAMEPAD_BOTTOM_BUTTON);
  setManualControllerButtonState(ccwPressed, cwPressed, stopPressed);
  const rotation = (
    (ccwTrigger - cwTrigger) * MANUAL_SPIN_POWER
  );
  const stickActive = Math.hypot(x, y) > GAMEPAD_ACTIVE_EPSILON;
  const rotationActive = Math.abs(rotation) > GAMEPAD_ACTIVE_EPSILON;
  const controllerActive = stickActive || rotationActive;

  if (stopPressed) {
    if (!gamepadStopButtonDown) {
      stopManualCommand();
    }
    gamepadStopButtonDown = true;
    return false;
  }
  gamepadStopButtonDown = false;

  if (!controllerActive) {
    gamepadStopLatched = false;
  }
  if (gamepadStopLatched) {
    return false;
  }
  if (controllerActive) {
    gamepadManualActive = true;
    setManualInputSource("gamepad");
    manualRotation = rotation;
    setManualState(x, y, manualRotation);
    return true;
  }

  if (gamepadManualActive) {
    gamepadManualActive = false;
    manualRotation = 0;
    setManualInputSource("idle");
    setManualState(0, 0, 0);
    sendManualCommand(true);
  }
  return false;
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
  drawManualControl();
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
  context.fillStyle = "#657384";
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
  context.strokeStyle = "#f97316";
  context.fillStyle = "#fff1e7";
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
  context.fillStyle = "#f97316";
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
    setChip(planStatus, statusErrorText(error, "planner offline"), "bad");
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
    setChip(navStatus, statusErrorText(error, "nav offline"), "bad");
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
    setChip(navStatus, statusErrorText(error, "nav offline"), "bad");
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

function updatePersonTracking(nextState) {
  const person = nextState.person_tracking || {};
  const image = person.image || {};
  const detections = Array.isArray(person.detections) ? person.detections : [];
  const status = person.status || "stopped";
  const enabled = Boolean(person.enabled);
  const isExternal = status === "external";

  personTrackingToggle.checked = enabled;
  personTrackingToggle.disabled = (
    personTrackingRequestPending ||
    !person.control_enabled ||
    isExternal
  );

  if (person.error) {
    setChip(personStatus, person.error, "bad");
  } else if (status === "running") {
    setChip(personStatus, `person ${detections.length}`, "good");
  } else if (status === "external") {
    setChip(personStatus, "person external", "good");
  } else if (status === "starting" || status === "stopping") {
    setChip(personStatus, `person ${status}`, "warn");
  } else if (status.startsWith("exited")) {
    setChip(personStatus, `person ${status}`, "bad");
  } else {
    setChip(personStatus, "person off", "");
  }

  personCount.textContent = String(detections.length);
  personFrameAge.textContent = seconds(image.age_sec);

  if (image.available && image.revision !== personImageRevision) {
    personImageRevision = image.revision;
    personImage.hidden = false;
    personImagePlaceholder.hidden = true;
    personImage.src = `/api/person_tracking/image.jpg?rev=${personImageRevision}`;
  } else if (!image.available) {
    personImageRevision = -1;
    personImage.removeAttribute("src");
    personImage.hidden = true;
    personImagePlaceholder.hidden = false;
  }
}

async function setPersonTrackingEnabled(enabled) {
  personTrackingRequestPending = true;
  personTrackingToggle.disabled = true;
  setChip(personStatus, enabled ? "person starting" : "person stopping", "warn");

  try {
    const response = await fetch("/api/person_tracking", {
      method: "POST",
      headers: {
        "Content-Type": "application/json"
      },
      body: JSON.stringify({ enabled })
    });
    const payload = await response.json();
    if (!response.ok || !payload.ok) {
      throw new Error(payload.error || "Person tracking update failed.");
    }
    await refreshState();
  } catch (error) {
    personTrackingToggle.checked = !enabled;
    setChip(personStatus, statusErrorText(error, "person offline"), "bad");
  } finally {
    personTrackingRequestPending = false;
    updatePersonTracking(state || {});
  }
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

  const manual = state.manual || {};
  const localManualActive = (
    manualLinearActive() ||
    Math.abs(manualState.angular) > 0.01
  );
  const manualOrientation = manual.orientation || {};
  if (!manual.enabled) {
    setChip(manualStatus, "manual disabled", "bad");
  } else if (
    manualOrientation.enabled &&
    manualLinearActive() &&
    !manualOrientation.fresh
  ) {
    setChip(
      manualStatus,
      manualOrientation.available ? "imu stale" : "imu waiting",
      "warn"
    );
  } else if (localManualActive) {
    setChip(manualStatus, "manual active", "good");
  } else {
    setChip(manualStatus, "manual idle", "");
  }

  const canNavigate = Boolean(goal) && points > 0 && !isNavigationActive(navState);
  navigateButton.disabled = !canNavigate;
  stopButton.disabled = !isNavigationActive(navState);
  updatePersonTracking(state);
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
    setChip(mapStatus, statusErrorText(error, "backend offline"), "bad");
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

manualCanvas.addEventListener("pointerdown", (event) => {
  manualPointerId = event.pointerId;
  gamepadManualActive = false;
  gamepadStopLatched = false;
  manualCanvas.setPointerCapture(event.pointerId);
  updateManualFromPointer(event);
});

manualCanvas.addEventListener("pointermove", (event) => {
  if (manualPointerId !== event.pointerId) {
    return;
  }
  updateManualFromPointer(event);
});

manualCanvas.addEventListener("pointerup", (event) => {
  if (manualPointerId !== event.pointerId) {
    return;
  }
  manualCanvas.releasePointerCapture(event.pointerId);
  manualPointerId = null;
  setManualState(0, 0, manualRotation);
  sendManualCommand(true);
});

manualCanvas.addEventListener("pointercancel", () => {
  stopManualCommand();
});

manualCcwButton.addEventListener("pointerdown", () => {
  setManualRotation(MANUAL_SPIN_POWER);
});

manualCwButton.addEventListener("pointerdown", () => {
  setManualRotation(-MANUAL_SPIN_POWER);
});

for (const button of [manualCcwButton, manualCwButton]) {
  button.addEventListener("pointerup", () => {
    setManualRotation(0);
  });
  button.addEventListener("pointercancel", () => {
    setManualRotation(0);
  });
  button.addEventListener("pointerleave", (event) => {
    if (event.buttons === 0) {
      setManualRotation(0);
    }
  });
}

manualStopButton.addEventListener("click", () => {
  stopManualCommand();
});

personTrackingToggle.addEventListener("change", () => {
  setPersonTrackingEnabled(personTrackingToggle.checked);
});

personImage.addEventListener("error", () => {
  personImage.hidden = true;
  personImagePlaceholder.hidden = false;
});

headingMode.addEventListener("change", () => {
  headingInput.disabled = headingMode.value !== "custom";
});

window.addEventListener("resize", resizeCanvas);
window.addEventListener("gamepadconnected", (event) => {
  if (gamepadHasLeftStick(event.gamepad)) {
    gamepadIndex = event.gamepad.index;
  }
});
window.addEventListener("gamepaddisconnected", (event) => {
  if (event.gamepad.index !== gamepadIndex) {
    return;
  }
  gamepadIndex = null;
  setManualControllerButtonState(false, false, false);
  if (gamepadManualActive) {
    gamepadManualActive = false;
    gamepadStopButtonDown = false;
    manualRotation = 0;
    setManualInputSource("idle");
    setManualState(0, 0, 0);
    sendManualCommand(true);
  }
});

headingInput.disabled = true;
resizeCanvas();
refreshState();
setInterval(refreshState, 700);
setInterval(() => {
  const gamepadActive = pollGamepadManualControl();
  const active = (
    gamepadActive ||
    manualPointerId !== null ||
    manualRotation !== 0 ||
    Math.hypot(manualState.x, manualState.y) > 0.01 ||
    Math.abs(manualState.angular) > 0.01
  );
  if (active) {
    sendManualCommand();
  }
}, 80);
