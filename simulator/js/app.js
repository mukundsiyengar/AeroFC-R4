// app.js - the simulator / ground-station page: connects the flight controller (software or real board),
// runs the physics in real time, reads keyboard / gamepad, draws the 3D view, the OLED and the live plots.
import { SimEngine } from "./sim_engine.js";
import { Pilot, CODES } from "./pilot_input.js";
import { View3D, drawHorizon } from "./view3d.js";
import { StripChart } from "./plots.js";
import { SerialLink } from "./fc_webserial.js";
import { Parser, T, decodeStatus, STATES, REASONS } from "./protocol.js";
import { BTN, EV } from "./fc_wasm.js";

const $ = (id) => document.getElementById(id);
const pilot = new Pilot();
pilot.attach(window);
const view = new View3D($("view"));

let engine = null;        // SimEngine (software FC or real board through Web Serial)
let mode = "none";        // "software" | "arduino" | "telemetry" | "gazebo"
let serial = null;
let status = null;        // last decoded STATUS telemetry
let statusTime = 0;
let wall0 = 0, sim0 = 0, speed = 1;
let lastFb = null;
let gazebo = null;        // { es: EventSource, url }
let lastPilotSend = 0;

// ------------------------------------------------------------------ live plots
const C = { r: "#e11d48", p: "#2563eb", y: "#16a34a", t: "#d97706", g: "#7c3aed", k: "#64748b" };
const charts = {
  rc: new StripChart($("plotRc"), { title: "RC input", min: -1.05, max: 1.05, series: [
    { name: "roll", color: C.r }, { name: "pitch", color: C.p }, { name: "yaw", color: C.y }, { name: "climb/thr", color: C.t }] }),
  att: new StripChart($("plotAtt"), { title: "Attitude", unit: "deg", series: [
    { name: "roll", color: C.r }, { name: "roll true", color: C.r, dash: true }, { name: "pitch", color: C.p },
    { name: "pitch true", color: C.p, dash: true }, { name: "roll sp", color: C.k, dash: true }] }),
  yaw: new StripChart($("plotYaw"), { title: "Heading", unit: "deg", series: [
    { name: "yaw", color: C.y }, { name: "true (mag)", color: C.y, dash: true }, { name: "target", color: C.k, dash: true }] }),
  alt: new StripChart($("plotAlt"), { title: "Altitude", unit: "m", series: [
    { name: "estimate", color: C.g }, { name: "true", color: C.g, dash: true }, { name: "target", color: C.k, dash: true }, { name: "climb m/s", color: C.t }] }),
  mot: new StripChart($("plotMot"), { title: "Motors", min: 0, max: 1, series: [
    { name: "FR", color: C.r }, { name: "RL", color: C.p }, { name: "FL", color: C.y }, { name: "RR", color: C.t }] }),
};

// ------------------------------------------------------------------ logging
let rec = null;
function logRow(t) {
  if (!rec) return;
  const s = status || {};
  const tr = engine ? engine.phys.truth() : {};
  rec.push([t.toFixed(3), s.stateName, s.modeName, s.armed ? 1 : 0, s.rcThr, s.rcRoll, s.rcPitch, s.rcYaw,
    s.roll, s.pitch, s.yaw, s.alt, s.altTarget, s.vz, tr.roll, tr.pitch, tr.yawMag, tr.alt, tr.north, tr.east,
    ...(s.motors || [NaN, NaN, NaN, NaN])].map((v) => (typeof v === "number" ? +v.toFixed(4) : v ?? "")).join(","));
}
$("recBtn").onclick = () => {
  if (!rec) {
    rec = ["t,state,mode,armed,rc_thr,rc_roll,rc_pitch,rc_yaw,roll,pitch,yaw,alt,alt_target,vz,roll_true,pitch_true,yaw_true_mag,alt_true,north_true,east_true,m1,m2,m3,m4"];
    $("recBtn").textContent = "Stop & download log";
    $("recBtn").classList.add("rec");
  } else {
    const blob = new Blob([rec.join("\n") + "\n"], { type: "text/csv" });
    const a = document.createElement("a");
    a.href = URL.createObjectURL(blob);
    a.download = `aerofc_flight_${new Date().toISOString().replace(/[:.]/g, "-")}.csv`;
    a.click();
    rec = null;
    $("recBtn").textContent = "Record log (CSV)";
    $("recBtn").classList.remove("rec");
  }
};

// ------------------------------------------------------------------ log panel
function addLog(line) {
  const el = $("log");
  el.textContent += line + "\n";
  if (el.textContent.length > 8000) el.textContent = el.textContent.slice(-6000);
  el.scrollTop = el.scrollHeight;
}

// ------------------------------------------------------------------ environment controls
function envFromUi() {
  const v = (id) => parseFloat($(id).value) || 0;
  return {
    wind: [v("envWind"), 0, 0], gust: v("envGust"),
    gyroNoise: v("envGyroN") * Math.PI / 180, accNoise: v("envAccN"), baroNoise: v("envBaroN"),
    gyroBias: [v("envGyroB") * Math.PI / 180, -v("envGyroB") * Math.PI / 180 * 0.7, v("envGyroB") * Math.PI / 180 * 0.4],
    magNoise: v("envMagN") * 1e-6,
  };
}
function applyEnv() {
  if (!engine) return;
  Object.assign(engine.phys.env, envFromUi());
  engine.opts.upLatencyMs = 2 + (parseFloat($("envLat").value) || 0);
  engine.bridge.opts.burstPeriod = 1 / Math.max(10, parseFloat($("envRate").value) || 50);
  engine.pilotMode = $("pilotProto").value;
  for (const id of ["envWind", "envGust", "envGyroN", "envAccN", "envBaroN", "envGyroB", "envMagN", "envLat", "envRate"]) {
    const out = $(id + "Val");
    if (out) out.textContent = $(id).value;
  }
}
document.querySelectorAll("#envPanel input, #pilotProto").forEach((el) => el.addEventListener("input", applyEnv));
$("simSpeed").onchange = () => { speed = parseFloat($("simSpeed").value); resync(); };

// ------------------------------------------------------------------ connections
function resync() { if (engine) { wall0 = performance.now(); sim0 = engine.t; } }

async function stopAll() {
  if (serial) { await serial.close(); serial = null; }
  if (gazebo) { gazebo.es.close(); gazebo = null; }
  engine = null;
  status = null;
  lastFb = null;
  view.trail = [];
  Object.values(charts).forEach((c) => c.clear());
}

function setMode(m, label) {
  mode = m;
  $("modeLabel").textContent = label;
  document.body.dataset.mode = m;
  $("oledCard").style.display = m === "software" ? "" : "none";
  $("realOledNote").style.display = m === "arduino" ? "" : "none";
}

async function startSoftware() {
  await stopAll();
  $("log").textContent = "";
  try {
    engine = await SimEngine.software({ storageKey: "aerofc_r4_settings", env: envFromUi() });
  } catch (e) {
    addLog("Could not load fc.wasm: " + e.message + "  (start the page with START_SIMULATOR.bat)");
    return;
  }
  engine.bridge.onLog = addLog;
  engine.bridge.onStatus = (s) => { status = s; statusTime = performance.now(); };
  applyEnv();
  engine.bridge.start();
  setMode("software", "Software flight controller (same code as the Arduino)");
  resync();
}

async function connectArduino() {
  if (!SerialLink.supported()) { alert("This browser has no Web Serial. Please use Microsoft Edge or Google Chrome."); return; }
  await stopAll();
  $("log").textContent = "";
  serial = new SerialLink();
  try { await serial.open(115200); } catch (e) { addLog("Port not opened: " + e.message); serial = null; return; }
  engine = new SimEngine({ env: envFromUi() });
  engine.attachSerial(serial);
  serial.onData = (b) => engine && engine.serialReceived(b);
  serial.onClose = () => addLog("Serial port closed (board unplugged?)");
  engine.bridge.onLog = addLog;
  engine.bridge.onStatus = (s) => { status = s; statusTime = performance.now(); };
  applyEnv();
  speed = 1; $("simSpeed").value = "1";
  addLog("Port open. Sending HELLO to the Arduino...");
  engine.bridge.start();
  setMode("arduino", "Arduino UNO R4 over USB (hardware in the loop)");
  resync();
}

async function connectTelemetry() {
  if (!SerialLink.supported()) { alert("This browser has no Web Serial. Please use Microsoft Edge or Google Chrome."); return; }
  await stopAll();
  serial = new SerialLink();
  try { await serial.open(115200); } catch (e) { addLog("Port not opened: " + e.message); serial = null; return; }
  const parser = new Parser();
  serial.onData = (b) => {
    for (const pk of parser.push(b)) if (pk.type === T.STATUS) { const s = decodeStatus(pk.payload); if (s) { status = s; statusTime = performance.now(); } }
  };
  setMode("telemetry", "Telemetry only (USB-TTL adapter on pins D0/D1)");
  addLog("Listening for STATUS telemetry at 115200 baud...");
}

function connectGazebo() {
  const base = $("relayUrl").value.replace(/\/$/, "");
  stopAll().then(() => {
    const es = new EventSource(base + "/events");
    es.onmessage = (ev) => { try { const s = JSON.parse(ev.data); if (s && s.stateName) { status = s; statusTime = performance.now(); } } catch (_) { } };
    es.onerror = () => addLog("No connection to " + base + " (is bridge_plus.py / relay.py running in WSL?)");
    gazebo = { es, url: base };
    setMode("gazebo", "Gazebo in WSL (pilot via relay.py, plots via bridge_plus.py)");
    addLog("Gazebo mode: sending pilot input to " + base + "/control");
  });
}

$("btnSoftware").onclick = startSoftware;
$("btnArduino").onclick = connectArduino;
$("btnTelemetry").onclick = connectTelemetry;
$("btnGazebo").onclick = connectGazebo;
$("btnStop").onclick = async () => { await stopAll(); setMode("none", "Stopped"); };
$("camBtn").onclick = () => { view.camMode = view.camMode === "chase" ? "ground" : "chase"; $("camBtn").textContent = view.camMode === "chase" ? "Camera: chase" : "Camera: ground"; };

const cmd = (code) => {
  if (engine) engine.command(code);
  else if (gazebo) sendGazebo({ thr: code, roll: 0, pitch: 0, yaw: 0 }, true);
};
$("cmdArm").onclick = () => cmd(status && status.armed ? CODES.DISARM : CODES.ARM);
$("cmdTakeoff").onclick = () => cmd(CODES.TAKEOFF);
$("cmdLand").onclick = () => cmd(CODES.LAND);
$("cmdKill").onclick = () => cmd(CODES.KILL);
$("cmdManual").onclick = () => pilot.toggleManual();
$("cmdStop").onclick = () => engine && engine.bridge.sendStop();

// ------------------------------------------------------------------ virtual OLED buttons
function setupOledButtons() {
  const map = { oledUp: BTN.UP, oledDown: BTN.DOWN, oledSel: BTN.SELECT, oledBack: BTN.BACK };
  for (const [id, b] of Object.entries(map)) {
    const el = $(id);
    let downAt = 0, repeatTimer = null, longSent = false, longTimer = null;
    const send = (ev) => engine && engine.fc && engine.fc.button(b, ev, engine.nowUs());
    const down = (e) => {
      e.preventDefault();
      downAt = performance.now();
      longSent = false;
      if (b === BTN.UP || b === BTN.DOWN) {
        send(EV.SHORT);
        repeatTimer = setTimeout(function rep() { send(EV.REPEAT); repeatTimer = setTimeout(rep, 110); }, 450);
      } else {
        longTimer = setTimeout(() => { longSent = true; send(EV.LONG); }, 800);
      }
    };
    const up = () => {
      clearTimeout(repeatTimer); clearTimeout(longTimer);
      if ((b === BTN.SELECT || b === BTN.BACK) && !longSent && downAt) send(EV.SHORT);
      downAt = 0;
    };
    el.addEventListener("pointerdown", down);
    el.addEventListener("pointerup", up);
    el.addEventListener("pointerleave", () => { if (downAt) up(); });
  }
}
setupOledButtons();

function drawOled(fb) {
  const cv = $("oled");
  const g = cv.getContext("2d");
  const img = g.createImageData(128, 64);
  for (let page = 0; page < 8; page++)
    for (let x = 0; x < 128; x++) {
      const byte = fb[page * 128 + x];
      for (let bit = 0; bit < 8; bit++) {
        const on = (byte >> bit) & 1;
        const i = ((page * 8 + bit) * 128 + x) * 4;
        img.data[i] = on ? 120 : 6; img.data[i + 1] = on ? 220 : 10; img.data[i + 2] = on ? 255 : 20; img.data[i + 3] = 255;
      }
    }
  g.putImageData(img, 0, 0);
}

// ------------------------------------------------------------------ Gazebo relay (WSL)
function sendGazebo(o, isCode = false) {
  if (!gazebo) return;
  fetch(gazebo.url + "/control", { method: "POST", body: `${o.thr},${o.roll},${o.pitch},${o.yaw}\n`, headers: { "Content-Type": "text/plain" } }).catch(() => {});
  if (isCode) { setTimeout(() => sendGazebo(o), 40); setTimeout(() => sendGazebo(o), 80); }
}

// ------------------------------------------------------------------ simulation clock (worker metronome)
const metronome = new Worker(new URL("./metronome.worker.js", import.meta.url));
let nextLog = 0;
metronome.onmessage = () => {
  if (!engine) return;
  const target = sim0 + (performance.now() - wall0) / 1000 * speed;
  let n = 0;
  while (engine.t < target && n < 120) {
    engine.step();
    n++;
    if (engine.t >= nextLog) { nextLog = engine.t + 0.02; logRow(engine.t); }
  }
  if (target - engine.t > 0.25) resync();   // computer too slow: do not try to catch up
};
metronome.postMessage("start");

// ------------------------------------------------------------------ frame loop
let lastFrame = performance.now(), lastPlot = 0;
function frame() {
  const now = performance.now();
  const dt = Math.min(0.1, (now - lastFrame) / 1000);
  lastFrame = now;
  pilot.armed = !!(status && status.armed);
  const out = pilot.update(dt);
  const codes = pilot.takeCommands();
  if (engine) {
    engine.setPilot(out);
    codes.forEach((c) => engine.command(c));
    if (engine.fc) {
      engine.fc.serviceStorage();
      const fb = engine.fc.render(engine.nowUs());
      if (fb) lastFb = fb;
      if (lastFb) drawOled(lastFb);
    }
  } else if (gazebo) {
    codes.forEach((c) => sendGazebo({ thr: c, roll: out.roll, pitch: out.pitch, yaw: out.yaw }, true));
    if (now - lastPilotSend > 40) { lastPilotSend = now; sendGazebo(out); }
  }

  // drone pose for the 3D view: truth from the physics, or the FC's own estimate in telemetry-only modes
  const s = status;
  let phys = engine ? engine.phys : null;
  if (!phys && s) phys = poseFromEstimate(s);
  const tr = engine ? engine.phys.truth() : null;
  const hud = [];
  if (s) {
    hud.push(`${s.stateName}  ${s.armed ? s.modeName : ""}  ${s.failsafe ? "FAILSAFE: " + s.reasonName : ""}${s.crashed ? " CRASH" : ""}`);
    hud.push(`alt ${s.alt.toFixed(2)} m  climb ${s.vz.toFixed(2)} m/s  target ${s.altTarget.toFixed(2)}`);
  }
  if (tr) hud.push(`speed ${tr.speed.toFixed(1)} m/s  N ${tr.north.toFixed(1)}  E ${tr.east.toFixed(1)}`);
  if (phys) view.draw(phys, s ? s.motors : (engine ? engine.bridge.motors : null), hud);
  if (s) drawHorizon($("horizon"), s.roll, s.pitch, s.yaw);

  // plots at ~30 Hz
  if (s && now - lastPlot > 33) {
    lastPlot = now;
    const t = engine ? engine.t : now / 1000;
    const thrPlot = s.rcThr >= 1.5 && s.rcThr <= 3.5 ? (s.rcThr - 2.5) * 2 : (s.rcThr <= 1 ? s.rcThr : NaN);
    charts.rc.push(t, [s.rcRoll, s.rcPitch, s.rcYaw, thrPlot]);
    charts.att.push(t, [s.roll, tr ? tr.roll : NaN, s.pitch, tr ? tr.pitch : NaN, s.rollSp]);
    charts.yaw.push(t, [s.yaw, tr ? tr.yawMag : NaN, s.headingTarget]);
    charts.alt.push(t, [s.alt, tr ? tr.alt : NaN, s.altTarget, s.vz]);
    charts.mot.push(t, s.motors);
  }
  Object.values(charts).forEach((c) => c.draw());
  updatePanels(s);
  requestAnimationFrame(frame);
}

function poseFromEstimate(s) {
  const d = Math.PI / 180, r = s.roll * d, p = s.pitch * d, y = s.yaw * d;
  const cr = Math.cos(r), sr = Math.sin(r), cp = Math.cos(p), sp = Math.sin(p), cy = Math.cos(y), sy = Math.sin(y);
  const Rn = [[cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy],
              [cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy],
              [-sp, sr * cp, cr * cp]];
  // R_enu_flu = T * R_ned_frd * D   (T swaps x/y and flips z, D = diag(1,-1,-1))
  const R = [[Rn[1][0], -Rn[1][1], -Rn[1][2]], [Rn[0][0], -Rn[0][1], -Rn[0][2]], [-Rn[2][0], Rn[2][1], Rn[2][2]]];
  return { p: [0, 0, Math.max(0.055, s.alt + 0.055)], R };
}

function updatePanels(s) {
  const age = s ? (performance.now() - statusTime) / 1000 : 99;
  $("stState").textContent = s ? s.stateName + (s.armed ? " (armed)" : "") : "-";
  $("stState").className = s && s.armed ? "armed" : "";
  $("stMode").textContent = s ? s.modeName : "-";
  $("stReason").textContent = s ? s.reasonName : "-";
  $("stLink").textContent = mode === "none" ? "not started" : age < 0.5 ? "telemetry OK" : "no telemetry";
  $("stRates").textContent = s ? `${s.imuHz} Hz sensors, ${s.loopUs} us step` : "-";
  $("stEst").textContent = s ? `${s.compass ? "compass" : "gyro yaw"}${s.headingLock ? ", heading lock" : ""}` : "-";
  $("stHover").textContent = s ? s.hoverCmd.toFixed(3) : "-";
  if (engine) {
    const b = engine.bridge;
    $("stBridge").textContent = `${b.state}  motors ${b.motors.map((m) => m.toFixed(2)).join(" ")}`;
    if (engine.fc) {
      const i = engine.info();
      $("stEst").textContent += i.ekfInit ? `, GPS aid ${engine.fc.info()[50] > 0.5 ? "on" : "off"}` : ", EKF starting";
    }
  } else $("stBridge").textContent = mode === "gazebo" ? "official bridge (Gazebo)" : "-";
  $("stPilot").textContent = `${pilot.source}${pilot.gamepadName ? ": " + pilot.gamepadName : ""}${pilot.manual ? "  MANUAL " + pilot.manualCmd.toFixed(2) : ""}`;
  const o = pilot.output();
  $("stSticks").textContent = `roll ${o.roll.toFixed(2)}  pitch ${o.pitch.toFixed(2)}  yaw ${o.yaw.toFixed(2)}  thr ${o.thr.toFixed(2)}`;
  $("cmdArm").textContent = s && s.armed ? "Disarm (Enter)" : "Arm (Enter)";
}

window.addEventListener("resize", () => Object.values(charts).forEach((c) => c.draw()));
setMode("none", "Choose how to fly (left panel)");
requestAnimationFrame(frame);
if (new URLSearchParams(location.search).get("autostart") === "software") startSoftware();
window.__app = { get engine() { return engine; }, get status() { return status; }, pilot, startSoftware };
