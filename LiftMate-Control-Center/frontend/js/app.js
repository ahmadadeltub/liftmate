/* LiftMate Control & Diagnostic Center - Frontend Controller */
"use strict";

let appState = null;
let ws = null;
let holdInterval = null;
let activeHoldType = null;
let wheelSliders = { 1: 50, 2: 50, 3: 50, 4: 50 };
let wheelCardsInitialized = false;
let activeKey = null;

function showToast(message, level = "info") {
  const banner = document.getElementById("toastBanner");
  const textEl = document.getElementById("toastMessage");
  if (!banner || !textEl) return;
  textEl.textContent = message;
  banner.className = "toast-banner";
  if (level === "error") banner.classList.add("error");
  else if (level === "warn") banner.classList.add("warn");
}

async function apiPost(path, body = {}) {
  try {
    const resp = await fetch(path, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    });
    const data = await resp.json();
    if (data.message) {
      showToast(data.message, data.ok === false ? "error" : "info");
    }
    if (data.state) {
      renderState(data.state);
    }
    return data;
  } catch (err) {
    showToast(`API Error (${path}): ${err.message}`, "error");
    return { ok: false };
  }
}

/* ==================== NAVIGATION ==================== */
function switchTab(tabId, btnEl) {
  document.querySelectorAll(".tab-panel").forEach((el) => el.classList.remove("active"));
  document.querySelectorAll(".nav-btn").forEach((el) => el.classList.remove("active"));
  const target = document.getElementById(tabId);
  if (target) target.classList.add("active");
  if (btnEl) btnEl.classList.add("active");
  if (tabId === "tab-diagnostics") {
    fetchDiagnosticsEvents();
  }
}

function toggleShowAllPanels(checked) {
  if (checked) {
    document.body.classList.add("show-all-panels");
  } else {
    document.body.classList.remove("show-all-panels");
  }
}

/* ==================== SAFETY & EMERGENCY CONTROLS ==================== */
async function armSystem() {
  await apiPost("/api/safety/arm");
}

async function disarmSystem() {
  stopActiveHold();
  await apiPost("/api/safety/disarm");
}

async function stopAllNow() {
  stopActiveHold();
  await apiPost("/api/safety/stop_all");
}

async function triggerEmergencyStop() {
  stopActiveHold();
  await apiPost("/api/safety/estop", { reason: "Operator pressed E-STOP LATCH button" });
}

async function resetEmergencyStop() {
  stopActiveHold();
  await apiPost("/api/safety/reset_estop");
}

async function clearFaults() {
  await apiPost("/api/safety/clear_fault");
  fetchDiagnosticsEvents();
}

/* ==================== SERIAL & DIAGNOSTICS ==================== */
async function connectSerial() {
  const port = document.getElementById("serialPortSelect").value || "auto";
  await apiPost("/api/serial/connect", { port, baud_rate: 115200 });
}

async function disconnectSerial() {
  stopActiveHold();
  await apiPost("/api/serial/disconnect");
}

async function pingArduino() {
  await apiPost("/api/serial/ping");
}

async function toggleSimulationMode() {
  const currentSim = appState && appState.serial ? appState.serial.simulation_mode : false;
  await apiPost("/api/serial/simulation", { enabled: !currentSim });
}

async function sendRawCommand() {
  const cmd = document.getElementById("rawCommandInput").value.trim();
  if (!cmd) return;
  const res = await apiPost("/api/serial/raw", { command: cmd });
  if (res.response) {
    showToast(`Raw Command Reply: ${res.response}`, res.ok ? "info" : "warn");
  }
  fetchDiagnosticsEvents();
}

async function fetchDiagnosticsEvents() {
  try {
    const resp = await fetch("/api/diagnostics/events?limit=80");
    const data = await resp.json();
    const tbody = document.getElementById("diagEventsTableBody");
    if (!tbody || !data.events) return;
    tbody.innerHTML = data.events
      .map(
        (ev) => `
      <tr>
        <td class="mono">${ev.timestamp.split("T")[1] || ev.timestamp}</td>
        <td><strong>${ev.level}</strong></td>
        <td>${ev.subsystem}</td>
        <td class="mono">${ev.event}</td>
        <td>${ev.details || ""}</td>
      </tr>`
      )
      .join("");
  } catch (_) {}
}

/* ==================== 4 WHEEL TEST CARDS & CALIBRATION ==================== */
function initWheelCards() {
  const container = document.getElementById("wheelCardsContainer");
  if (!container || wheelCardsInitialized) return;
  wheelCardsInitialized = true;

  const html = [1, 2, 3, 4]
    .map(
      (id) => `
    <div class="card" id="wheelCard_${id}">
      <div class="card-title">
        <span>MOTOR ${id}</span>
        <span class="pill" id="wPosBadge_${id}">--</span>
      </div>
      <div class="kv-row"><span>Arduino Pins</span><span id="wPins_${id}">PWM -- / IN --,--</span></div>
      <div class="kv-row"><span>Commanded PWM</span><span id="wCmdPwm_${id}">0 PWM</span></div>
      <div class="kv-row"><span>Measured Speed</span><span style="color:var(--text-muted);">Open-Loop</span></div>
      <div class="kv-row"><span>Comm / Result</span><span id="wStatus_${id}">OFFLINE / UNTESTED</span></div>

      <div class="control-group">
        <label>
          <span>Test PWM Speed</span>
          <strong id="wSliderLabel_${id}">50 PWM</strong>
        </label>
        <input type="range" id="wSlider_${id}" min="0" max="255" value="50"
               oninput="onWheelSliderChange(${id}, this.value)" />
      </div>

      <div class="btn-row">
        <button class="btn btn-hold" id="wHoldFwd_${id}">HOLD FWD</button>
        <button class="btn btn-hold" id="wHoldRev_${id}">HOLD REV</button>
      </div>
      <div class="btn-row">
        <button class="btn btn-cyan" style="flex:1;" onclick="jogSingleMotor(${id}, 'FORWARD')">JOG FWD</button>
        <button class="btn btn-cyan" style="flex:1;" onclick="jogSingleMotor(${id}, 'REVERSE')">JOG REV</button>
        <button class="btn btn-stop-all" style="padding:6px 10px;" onclick="stopSingleMotor(${id})">STOP</button>
      </div>
      <div class="btn-row" style="justify-content:space-between;margin-top:2px;">
        <span style="font-size:11px;color:var(--text-secondary);">Mark Test:</span>
        <div class="btn-row">
          <button class="btn" style="padding:3px 8px;font-size:11px;" onclick="markMotorTest(${id}, 'PASS')">PASS</button>
          <button class="btn" style="padding:3px 8px;font-size:11px;" onclick="markMotorTest(${id}, 'FAIL')">FAIL</button>
        </div>
      </div>
    </div>
  `
    )
    .join("");
  container.innerHTML = html;

  [1, 2, 3, 4].forEach((id) => {
    bindHoldButton(
      document.getElementById(`wHoldFwd_${id}`),
      () => sendSingleMotorHold(id, "FORWARD"),
      () => stopSingleMotor(id)
    );
    bindHoldButton(
      document.getElementById(`wHoldRev_${id}`),
      () => sendSingleMotorHold(id, "REVERSE"),
      () => stopSingleMotor(id)
    );
  });
}

function onWheelSliderChange(id, val) {
  wheelSliders[id] = parseInt(val, 10);
  const lbl = document.getElementById(`wSliderLabel_${id}`);
  if (lbl) lbl.textContent = `${val} PWM`;
}

async function sendSingleMotorHold(motorId, direction) {
  const pwm = wheelSliders[motorId] || 50;
  await apiPost("/api/motors/single", { motor_id: motorId, pwm, direction });
}

async function jogSingleMotor(motorId, direction) {
  const pwm = wheelSliders[motorId] || 50;
  const dur = parseInt(document.getElementById("wheelJogMs").value || "350", 10);
  await apiPost("/api/motors/single", {
    motor_id: motorId,
    pwm,
    direction,
    duration_ms: Math.min(500, Math.max(200, dur)),
  });
}

async function stopSingleMotor(motorId) {
  await apiPost("/api/motors/single", { motor_id: motorId, pwm: 0, direction: "STOP" });
}

async function stopAllWheels() {
  stopActiveHold();
  await apiPost("/api/motors/stop");
}

async function markMotorTest(motorId, result) {
  await apiPost("/api/motors/test_result", { motor_id: motorId, result });
}

async function saveWheelCalibration() {
  const positions = {
    "1": document.getElementById("calPos1").value,
    "2": document.getElementById("calPos2").value,
    "3": document.getElementById("calPos3").value,
    "4": document.getElementById("calPos4").value,
  };
  const inverted = {
    "1": document.getElementById("calInv1").checked,
    "2": document.getElementById("calInv2").checked,
    "3": document.getElementById("calInv3").checked,
    "4": document.getElementById("calInv4").checked,
  };
  const max_pwm_limit = parseInt(document.getElementById("calMaxPwm").value || "200", 10);
  await apiPost("/api/motors/calibration", { positions, inverted, max_pwm_limit });
}

/* ==================== PRESS-AND-HOLD ENGINE ==================== */
function stopActiveHold() {
  if (holdInterval) {
    clearInterval(holdInterval);
    holdInterval = null;
  }
  document.querySelectorAll(".holding").forEach((el) => el.classList.remove("holding"));
  if (activeHoldType === "DRIVE") {
    fetch("/api/motors/stop", { method: "POST" }).catch(() => {});
  } else if (activeHoldType === "ACTUATOR") {
    fetch("/api/actuators/stop", { method: "POST" }).catch(() => {});
  }
  activeHoldType = null;
}

function bindHoldButton(btn, onTick, onRelease) {
  if (!btn) return;
  const start = (ev) => {
    ev.preventDefault();
    stopActiveHold();
    btn.classList.add("holding");
    onTick();
    holdInterval = setInterval(onTick, 150);
  };
  const end = (ev) => {
    if (!btn.classList.contains("holding") && !holdInterval) return;
    ev.preventDefault();
    if (holdInterval) {
      clearInterval(holdInterval);
      holdInterval = null;
    }
    btn.classList.remove("holding");
    activeHoldType = null;
    onRelease();
  };
  btn.addEventListener("mousedown", start);
  btn.addEventListener("touchstart", start, { passive: false });
  btn.addEventListener("mouseup", end);
  btn.addEventListener("mouseleave", end);
  btn.addEventListener("touchend", end);
  btn.addEventListener("touchcancel", end);
}

function initMovementControls() {
  document.querySelectorAll(".dpad-btn[data-dir]").forEach((btn) => {
    const dir = btn.getAttribute("data-dir");
    bindHoldButton(
      btn,
      () => {
        activeHoldType = "DRIVE";
        const speed = parseInt(document.getElementById("driveSpeedSlider").value || "75", 10);
        apiPost("/api/motors/drive", { direction: dir, speed_pwm: speed });
      },
      () => {
        stopAllWheels();
      }
    );
  });

  // Actuator hold buttons
  bindHoldButton(
    document.getElementById("btnHoldUpperExtend"),
    () => {
      activeHoldType = "ACTUATOR";
      const pwm = parseInt(document.getElementById("upperPwmSlider").value || "120", 10);
      commandActuator("UPPER", "EXTEND", pwm);
    },
    () => commandActuator("UPPER", "STOP", 0)
  );
  bindHoldButton(
    document.getElementById("btnHoldUpperRetract"),
    () => {
      activeHoldType = "ACTUATOR";
      const pwm = parseInt(document.getElementById("upperPwmSlider").value || "120", 10);
      commandActuator("UPPER", "RETRACT", pwm);
    },
    () => commandActuator("UPPER", "STOP", 0)
  );
  bindHoldButton(
    document.getElementById("btnHoldLowerExtend"),
    () => {
      activeHoldType = "ACTUATOR";
      const pwm = parseInt(document.getElementById("lowerPwmSlider").value || "120", 10);
      commandActuator("LOWER", "EXTEND", pwm);
    },
    () => commandActuator("LOWER", "STOP", 0)
  );
  bindHoldButton(
    document.getElementById("btnHoldLowerRetract"),
    () => {
      activeHoldType = "ACTUATOR";
      const pwm = parseInt(document.getElementById("lowerPwmSlider").value || "120", 10);
      commandActuator("LOWER", "RETRACT", pwm);
    },
    () => commandActuator("LOWER", "STOP", 0)
  );

  // Keyboard W/A/S/D/Q/E/SPACE hold support
  const keyMap = {
    KeyW: "FORWARD",
    KeyS: "BACKWARD",
    KeyA: "LEFT_STRAFE",
    KeyD: "RIGHT_STRAFE",
    KeyQ: "ROTATE_LEFT",
    KeyE: "ROTATE_RIGHT",
  };

  window.addEventListener("keydown", (ev) => {
    const kbEnabled = document.getElementById("chkKeyboardDrive")?.checked;
    if (!kbEnabled) return;
    if (["INPUT", "SELECT", "TEXTAREA"].includes(document.activeElement?.tagName)) return;

    if (ev.code === "Space") {
      ev.preventDefault();
      stopAllNow();
      return;
    }

    const dir = keyMap[ev.code];
    if (dir && activeKey !== ev.code) {
      ev.preventDefault();
      activeKey = ev.code;
      stopActiveHold();
      activeHoldType = "DRIVE";
      const sendDrive = () => {
        const speed = parseInt(document.getElementById("driveSpeedSlider").value || "75", 10);
        apiPost("/api/motors/drive", { direction: dir, speed_pwm: speed });
      };
      sendDrive();
      holdInterval = setInterval(sendDrive, 150);
    }
  });

  window.addEventListener("keyup", (ev) => {
    if (activeKey && ev.code === activeKey) {
      activeKey = null;
      stopAllWheels();
    }
  });

  // Safety requirement: stop immediately if browser tab loses focus or visibility
  window.addEventListener("blur", () => {
    if (holdInterval || activeKey) {
      activeKey = null;
      stopAllNow();
    }
  });
  document.addEventListener("visibilitychange", () => {
    if (document.hidden) {
      activeKey = null;
      stopAllNow();
    }
  });
}

/* ==================== ACTUATOR CONTROLS ==================== */
async function commandActuator(group, action, pwm) {
  await apiPost("/api/actuators/command", { group, action, pwm });
}

async function pulseActuator(group, action) {
  const isUpper = group === "UPPER";
  const pwm = parseInt(
    document.getElementById(isUpper ? "upperPwmSlider" : "lowerPwmSlider").value || "120",
    10
  );
  const dur = parseInt(
    document.getElementById(isUpper ? "upperPulseMs" : "lowerPulseMs").value || "300",
    10
  );
  await apiPost("/api/actuators/command", { group, action, pwm, duration_ms: dur });
}

async function syncDriverVerified(checked) {
  document.getElementById("chkDriverVerifiedUpper").checked = checked;
  document.getElementById("chkDriverVerifiedLower").checked = checked;
  await updateActuatorSettings();
}

async function updateActuatorSettings() {
  const verified = document.getElementById("chkDriverVerifiedUpper").checked;
  const dual = document.getElementById("chkAllowDualJacks").checked;
  const reqImu = document.getElementById("chkRequireImuLifting").checked;
  const uInv = document.getElementById("chkUpperInvert").checked;
  const lInv = document.getElementById("chkLowerInvert").checked;
  await apiPost("/api/actuators/settings", {
    driver_electrical_verified: verified,
    allow_simultaneous_groups: dual,
    require_imu_for_lifting: reqImu,
    upper_inverted: uInv,
    lower_inverted: lInv,
  });
}

/* ==================== CAMERA CONTROLS ==================== */
async function startCamera() {
  const source = document.getElementById("cameraSourceSelect").value;
  const res = document.getElementById("cameraResSelect").value.split("x");
  const width = parseInt(res[0], 10);
  const height = parseInt(res[1], 10);
  const data = await apiPost("/api/camera/start", { source, width, height });
  if (data.ok) {
    const img = document.getElementById("cameraStreamImg");
    img.src = `/api/camera/stream.mjpg?t=${Date.now()}`;
    img.style.display = "block";
    document.getElementById("cameraPlaceholder").style.display = "none";
  }
}

async function stopCamera() {
  await apiPost("/api/camera/stop");
  const img = document.getElementById("cameraStreamImg");
  img.src = "";
  img.style.display = "none";
  document.getElementById("cameraPlaceholder").style.display = "block";
}

async function capturePhoto() {
  await apiPost("/api/camera/capture");
}

/* ==================== SERVO CONTROLS ==================== */
async function toggleServoEnable(enabled) {
  await apiPost("/api/servo/enable", { enabled });
}

async function setServoAngle(angle) {
  const slider = document.getElementById("servoAngleSlider");
  const label = document.getElementById("servoSliderVal");
  if (slider) slider.value = angle;
  if (label) label.textContent = `${angle}°`;
  await apiPost("/api/servo/angle", { angle_deg: parseInt(angle, 10) });
}

async function centerServo() {
  await apiPost("/api/servo/center");
}

async function saveServoLimits() {
  const safe_min_deg = parseInt(document.getElementById("servoSafeMin").value || "10", 10);
  const safe_max_deg = parseInt(document.getElementById("servoSafeMax").value || "170", 10);
  const inverted = document.getElementById("chkServoInvert").checked;
  await apiPost("/api/servo/limits", { safe_min_deg, safe_max_deg, inverted });
}

/* ==================== IMU CONTROLS & GRAPH ==================== */
async function requestImuSample() {
  await apiPost("/api/imu/request_sample");
}

async function saveImuThresholds() {
  const warning_deg = parseFloat(document.getElementById("imuWarnInput").value || "8.0");
  const critical_deg = parseFloat(document.getElementById("imuCritInput").value || "15.0");
  await apiPost("/api/imu/thresholds", { warning_deg, critical_deg });
}

function drawImuGraph(imu) {
  const canvas = document.getElementById("imuGraphCanvas");
  if (!canvas) return;
  const ctx = canvas.getContext("2d");
  const W = canvas.width;
  const H = canvas.height;
  ctx.clearRect(0, 0, W, H);

  const midY = H / 2;
  const scale = (H / 2) / 25.0; // +/- 25 deg full scale

  // Center zero line
  ctx.strokeStyle = "rgba(255,255,255,0.12)";
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(0, midY);
  ctx.lineTo(W, midY);
  ctx.stroke();

  // Warning threshold lines
  const warnY = (imu.tilt_warning_deg || 8.0) * scale;
  ctx.strokeStyle = "rgba(245, 158, 11, 0.35)";
  ctx.setLineDash([4, 4]);
  ctx.beginPath();
  ctx.moveTo(0, midY - warnY);
  ctx.lineTo(W, midY - warnY);
  ctx.moveTo(0, midY + warnY);
  ctx.lineTo(W, midY + warnY);
  ctx.stroke();
  ctx.setLineDash([]);

  const hist = imu.history || [];
  if (hist.length < 2) return;

  // Pitch line (Cyan)
  ctx.strokeStyle = "#00d4ff";
  ctx.lineWidth = 2;
  ctx.beginPath();
  hist.forEach((pt, idx) => {
    const x = (idx / Math.max(1, hist.length - 1)) * (W - 20) + 10;
    const y = Math.max(6, Math.min(H - 6, midY - pt.pitch * scale));
    if (idx === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  });
  ctx.stroke();

  // Roll line (Green)
  ctx.strokeStyle = "#10b981";
  ctx.lineWidth = 2;
  ctx.beginPath();
  hist.forEach((pt, idx) => {
    const x = (idx / Math.max(1, hist.length - 1)) * (W - 20) + 10;
    const y = Math.max(6, Math.min(H - 6, midY - pt.roll * scale));
    if (idx === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  });
  ctx.stroke();
}

/* ==================== RENDER TELEMETRY STATE ==================== */
function renderState(state) {
  appState = state;
  initWheelCards();

  const { safety, serial, motors, actuators, servo, imu, camera, diagnostics } = state;

  // 1. Header Badges
  const modeBadge = document.getElementById("modeBadge");
  if (modeBadge) {
    modeBadge.textContent = serial.simulation_mode ? "SIMULATION MODE" : "HARDWARE MODE";
    modeBadge.className = serial.simulation_mode ? "pill amber" : "pill cyan";
  }

  const safetyPill = document.getElementById("safetyStatePill");
  document.getElementById("headerSafetyState").textContent = safety.state;
  document.getElementById("safetyPanelState").textContent = safety.state;
  if (safety.state === "EMERGENCY_STOP" || safety.state === "FAULT") {
    safetyPill.className = "pill red";
  } else if (safety.armed) {
    safetyPill.className = "pill green";
  } else {
    safetyPill.className = "pill amber";
  }

  document.getElementById("btnResetEstop").style.display = safety.estop_latched ? "inline-flex" : "none";

  const serialPill = document.getElementById("serialStatusPill");
  if (serial.connected) {
    serialPill.className = "pill green";
    document.getElementById("headerSerialText").textContent = serial.simulation_mode
      ? "MEGA: SIMULATED"
      : `MEGA: ${serial.port.replace("/dev/", "")}`;
  } else {
    serialPill.className = "pill red";
    document.getElementById("headerSerialText").textContent = "MEGA: OFFLINE";
  }

  const imuPill = document.getElementById("imuStatusPill");
  if (imu.connected) {
    imuPill.className = imu.tilt_critical ? "pill red" : imu.tilt_warning ? "pill amber" : "pill green";
    document.getElementById("headerImuText").textContent = `IMU: P ${imu.pitch}° R ${imu.roll}°`;
  } else {
    imuPill.className = "pill amber";
    document.getElementById("headerImuText").textContent = "IMU: DISCONNECTED";
  }

  const camPill = document.getElementById("camStatusPill");
  if (camera.streaming && camera.connected) {
    camPill.className = "pill green";
    document.getElementById("headerCamText").textContent = `CAM: ${camera.measured_fps} FPS`;
  } else {
    camPill.className = "pill";
    document.getElementById("headerCamText").textContent = "CAM: STOPPED";
  }

  // 2. Overview Cards
  document.getElementById("ovMegaBadge").textContent = serial.connected ? "ONLINE" : "OFFLINE";
  document.getElementById("ovMegaBadge").className = serial.connected ? "pill green" : "pill red";
  document.getElementById("ovPort").textContent = serial.port;
  document.getElementById("ovBaud").textContent = serial.baud_rate;
  document.getElementById("ovReady").textContent = serial.arduino_ready ? "YES (#,READY,#)" : "NO";
  document.getElementById("ovLatency").textContent = `${diagnostics.latency_ms} ms`;

  document.getElementById("ovCalBadge").textContent = motors.calibrated ? "CALIBRATED" : "DEFAULT MAP";
  document.getElementById("ovCalBadge").className = motors.calibrated ? "pill green" : "pill amber";
  motors.motors.forEach((m) => {
    const el = document.getElementById(`ovM${m.motor_id}`);
    if (el) el.textContent = `${m.commanded_pwm} PWM (${m.calibrated_position})`;
  });

  document.getElementById("ovDriverBadge").textContent = actuators.driver_electrical_verified
    ? "DRIVER VERIFIED"
    : "VERIFY DRIVER";
  document.getElementById("ovDriverBadge").className = actuators.driver_electrical_verified
    ? "pill green"
    : "pill amber";
  document.getElementById("ovUpper").textContent = `${actuators.upper.direction} (${actuators.upper.commanded_pwm})`;
  document.getElementById("ovLower").textContent = `${actuators.lower.direction} (${actuators.lower.commanded_pwm})`;
  document.getElementById("ovServo").textContent = `${servo.enabled ? "ENABLED" : "DISABLED"} (${servo.commanded_angle}°)`;

  document.getElementById("ovImuBadge").textContent = imu.connected
    ? imu.simulated
      ? "SIMULATED"
      : "ONLINE"
    : "OFFLINE";
  document.getElementById("ovImuBadge").className = imu.connected ? "pill green" : "pill amber";
  document.getElementById("ovHeading").textContent = imu.connected ? `${imu.heading}°` : "Disconnected";
  document.getElementById("ovPitchRoll").textContent = imu.connected
    ? `${imu.pitch}° / ${imu.roll}°`
    : "Disconnected";
  document.getElementById("ovCamBackend").textContent = camera.active_backend;
  document.getElementById("ovCamFps").textContent = `${camera.measured_fps} FPS`;

  // 3. Wheel Cards & Mecanum Visualizer
  const posLabels = {
    front_left: "Front Left (FL)",
    front_right: "Front Right (FR)",
    rear_left: "Rear Left (RL)",
    rear_right: "Rear Right (RR)",
  };
  const posShort = {
    front_left: "FL",
    front_right: "FR",
    rear_left: "RL",
    rear_right: "RR",
  };

  motors.motors.forEach((m) => {
    const id = m.motor_id;
    document.getElementById(`wPosBadge_${id}`).textContent = posLabels[m.calibrated_position] || m.calibrated_position;
    document.getElementById(`wPins_${id}`).textContent = `PWM ${m.pins.pwm} / IN ${m.pins.in1},${m.pins.in2}`;
    document.getElementById(`wCmdPwm_${id}`).textContent = `${m.commanded_pwm} PWM`;
    document.getElementById(`wStatus_${id}`).textContent = `${m.comm_status} / ${m.test_result}`;

    const shortKey = posShort[m.calibrated_position];
    if (shortKey) {
      const pEl = document.getElementById(`kin${shortKey}`);
      const mEl = document.getElementById(`kin${shortKey}Motor`);
      if (pEl) pEl.textContent = `${m.commanded_pwm} PWM`;
      if (mEl) mEl.textContent = `Motor ${id} (PWM ${m.pins.pwm})`;
    }
  });

  document.getElementById("activeDriveBadge").textContent = motors.last_drive_direction;
  document.getElementById("calWizardBadge").textContent = motors.calibrated
    ? "Calibration Saved"
    : "Default Mapping Active";
  document.getElementById("calWizardBadge").className = motors.calibrated ? "pill green" : "pill amber";

  // 4. Actuators
  document.getElementById("upperStateBadge").textContent = actuators.upper.direction;
  document.getElementById("upperCmdText").textContent = `${actuators.upper.commanded_pwm} PWM (${actuators.upper.direction})`;
  document.getElementById("upperDurationText").textContent = `${actuators.upper.active_duration_s} s (Max: 10.0 s)`;
  document.getElementById("chkDriverVerifiedUpper").checked = actuators.driver_electrical_verified;
  document.getElementById("chkDriverVerifiedLower").checked = actuators.driver_electrical_verified;
  document.getElementById("chkAllowDualJacks").checked = actuators.allow_simultaneous_groups;
  document.getElementById("chkRequireImuLifting").checked = actuators.require_imu_for_lifting;

  const uHist = document.getElementById("upperHistoryTable");
  if (uHist) {
    uHist.innerHTML = (actuators.upper.history || [])
      .map((h) => `<tr><td class="mono">${h.ts}</td><td>${h.action}</td><td>${h.pwm}</td><td>${h.pulse_ms || "HOLD"}</td></tr>`)
      .join("");
  }

  document.getElementById("lowerStateBadge").textContent = actuators.lower.direction;
  document.getElementById("lowerCmdText").textContent = `${actuators.lower.commanded_pwm} PWM (${actuators.lower.direction})`;
  document.getElementById("lowerDurationText").textContent = `${actuators.lower.active_duration_s} s (Max: 5.0 s)`;
  const lHist = document.getElementById("lowerHistoryTable");
  if (lHist) {
    lHist.innerHTML = (actuators.lower.history || [])
      .map((h) => `<tr><td class="mono">${h.ts}</td><td>${h.action}</td><td>${h.pwm}</td><td>${h.pulse_ms || "HOLD"}</td></tr>`)
      .join("");
  }

  // 5. Camera
  document.getElementById("camFeedBadge").textContent = camera.streaming ? "LIVE" : "STOPPED";
  document.getElementById("camFeedBadge").className = camera.streaming ? "pill green" : "pill";
  document.getElementById("camDiagBackend").textContent = camera.active_backend;
  document.getElementById("camDiagFps").textContent = `${camera.measured_fps} FPS`;
  document.getElementById("camDiagFrames").textContent = camera.frame_count;
  document.getElementById("camDiagPicam").textContent = camera.picamera2_installed ? "YES" : "NO (Remote/OpenCV)";
  document.getElementById("camSavedList").textContent =
    camera.saved_photos && camera.saved_photos.length > 0 ? camera.saved_photos[0] : "None";
  if (!camera.streaming && camera.error_message) {
    document.getElementById("camErrorTitle").textContent = "Camera Unavailable";
    document.getElementById("camErrorDetail").textContent = camera.error_message;
  }

  // 6. Servo
  document.getElementById("chkServoEnable").checked = servo.enabled;
  document.getElementById("servoStateBadge").textContent = servo.enabled ? "ENABLED" : "DISABLED";
  document.getElementById("servoStateBadge").className = servo.enabled ? "pill green" : "pill amber";
  document.getElementById("servoCmdAngle").textContent = `${servo.commanded_angle}°`;
  document.getElementById("servoWindowText").textContent = `${servo.safe_min_deg}° – ${servo.safe_max_deg}°`;

  // 7. IMU
  document.getElementById("imuHeadingVal").textContent = imu.connected ? `${imu.heading}°` : "OFFLINE";
  document.getElementById("imuPitchVal").textContent = imu.connected ? `${imu.pitch}°` : "OFFLINE";
  document.getElementById("imuRollVal").textContent = imu.connected ? `${imu.roll}°` : "OFFLINE";
  document.getElementById("imuConnText").textContent = imu.connected
    ? imu.simulated
      ? "SIMULATED TELEMETRY"
      : "CONNECTED (0x29)"
    : `DISCONNECTED ${imu.sensor_error ? "(" + imu.sensor_error + ")" : ""}`;
  document.getElementById("imuCalText").textContent = imu.calibration_state;
  document.getElementById("imuAgeText").textContent =
    imu.last_update_age_ms !== null ? `${imu.last_update_age_ms} ms` : "N/A";
  document.getElementById("imuMaxTiltText").textContent =
    imu.max_tilt_deg !== null ? `${imu.max_tilt_deg}°` : "Unavailable";
  document.getElementById("imuSimBadge").textContent = imu.simulated ? "SIMULATED DATA" : "HARDWARE 0x29";
  document.getElementById("imuSimBadge").className = imu.simulated ? "pill amber" : "pill green";
  drawImuGraph(imu);

  // 8. Diagnostics & Serial Port Dropdown
  const portSel = document.getElementById("serialPortSelect");
  if (portSel && serial.available_ports && portSel.options.length <= 1) {
    serial.available_ports.forEach((p) => {
      const opt = document.createElement("option");
      opt.value = p.device;
      opt.textContent = `${p.device} (${p.description})`;
      portSel.appendChild(opt);
    });
  }
  document.getElementById("diagLastCmd").textContent = diagnostics.last_command;
  document.getElementById("diagLastResp").textContent = diagnostics.last_response;
  document.getElementById("diagLatency").textContent = `${diagnostics.latency_ms} ms`;
  document.getElementById("diagErrCount").textContent = diagnostics.error_count;
  document.getElementById("diagTimeoutCount").textContent = diagnostics.timeout_count;
  document.getElementById("diagStopCount").textContent = diagnostics.stop_event_count;
  document.getElementById("diagLastFault").textContent = diagnostics.last_fault;
}

/* ==================== WEBSOCKET TELEMETRY CLIENT ==================== */
function connectWebSocket() {
  const proto = window.location.protocol === "https:" ? "wss" : "ws";
  ws = new WebSocket(`${proto}://${window.location.host}/ws/telemetry`);
  const ind = document.getElementById("wsIndicator");

  ws.onopen = () => {
    if (ind) {
      ind.textContent = "WS: LIVE (10Hz)";
      ind.style.color = "var(--status-green)";
    }
  };

  ws.onmessage = (ev) => {
    try {
      const msg = JSON.parse(ev.data);
      if (msg.type === "STATE_UPDATE" && msg.data) {
        renderState(msg.data);
      }
    } catch (_) {}
  };

  ws.onclose = () => {
    if (ind) {
      ind.textContent = "WS: RECONNECTING...";
      ind.style.color = "var(--status-amber)";
    }
    setTimeout(connectWebSocket, 1200);
  };
}

window.addEventListener("DOMContentLoaded", () => {
  initWheelCards();
  initMovementControls();
  connectWebSocket();
  fetch("/api/state")
    .then((r) => r.json())
    .then((s) => renderState(s))
    .catch(() => {});
});
