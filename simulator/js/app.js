// app.js - the simulator / ground-station page: connects the flight controller (software or real board),
// runs the physics in real time, reads keyboard / gamepad, draws the 3D view, the OLED and the live plots.
import { SimEngine } from "./sim_engine.js";
import { Pilot, CODES, PAD_DEADZONE } from "./pilot_input.js";
import { View3D, drawHorizon } from "./view3d.js";
import { StripChart } from "./plots.js";
import { SerialLink } from "./fc_webserial.js";
import { Parser, T, decodeStatus, STATES, REASONS } from "./protocol.js";
import { BTN, EV } from "./fc_wasm.js";
import { linkStatus, throttleView, faultList } from "./page_status.js";
import { NavPanel, navText, MAX_WAYPOINTS } from "./nav_panel.js";

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
let gazebo = null;        // { es: EventSource, url, since, inFlight, postStatus, postAt }
let bridgeSeen = false;   // the Gazebo bridge answers, while this page is doing something else
let lastPilotSend = 0;
let rcCutAt = 0;          // performance.now() when the RC link test cut the pilot stream (0 = link on)
let keysMsgAt = 0;        // performance.now() of the web server's last message about the keys (0 = not heard)
let keysActive = false;   // a Gazebo window is the active window, so the server is reading the keys

// keys pressed in the Gazebo window: this page can't see them itself, so the web server reads them (tools/serve.py)
new EventSource("/keys").onmessage = (e) => {
  const m = JSON.parse(e.data);
  keysMsgAt = performance.now();
  keysActive = m.active;
  pilot.setRemoteKeys(m.keys);
};

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
  pos: new StripChart($("plotPos"), { title: "Position hold", unit: "m, m/s", series: [
    { name: "from hold point", color: C.g }, { name: "true", color: C.g, dash: true }, { name: "speed", color: C.t }] }),
};

// ------------------------------------------------------------------ logging
let rec = null;
// how far the flight controller's estimate is from the simulator's truth
function estimateError(s, tr) {
  return { pos: Math.hypot(tr.north - s.posN, tr.east - s.posE), att: Math.hypot(tr.roll - s.roll, tr.pitch - s.pitch) };
}
function logRow(t) {
  if (!rec) return;
  const s = status || {};
  const tr = engine ? engine.phys.truth() : {};
  const err = status && engine ? estimateError(s, tr) : { pos: NaN, att: NaN };
  const raw = pilot.padRaw || [NaN, NaN, NaN, NaN];
  rec.push([t.toFixed(3), s.stateName, s.modeName, s.armed ? 1 : 0, s.rcThr, s.rcRoll, s.rcPitch, s.rcYaw,
    s.roll, s.pitch, s.yaw, s.alt, s.altTarget, s.vz, tr.roll, tr.pitch, tr.yaw, tr.alt, tr.north, tr.east,
    ...(s.motors || [NaN, NaN, NaN, NaN]), s.speedModeName, s.posStateName, s.posN, s.posE, s.velN, s.velE, s.holdDist,
    rcCutAt ? 1 : 0, s.yawOffset, err.pos, err.att, engine ? engine.bridge.controlDelayMs : NaN, pilot.padRest, ...raw].map((v) => (typeof v === "number" ? +v.toFixed(4) : v ?? "")).join(","));
}
$("recBtn").onclick = () => {
  if (!rec) {
    rec = ["t,state,mode,armed,rc_thr,rc_roll,rc_pitch,rc_yaw,roll,pitch,yaw,alt,alt_target,vz,roll_true,pitch_true,yaw_true_mag,alt_true,north_true,east_true,m1,m2,m3,m4,speed_mode,pos_state,pos_n,pos_e,vel_n,vel_e,hold_dist,rc_cut,heading_offset,pos_err,att_err,control_delay_ms,pad_rest,pad0,pad1,pad2,pad3"];
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
    gpsNoise: v("envGpsN"), gpsFreeze: $("envGpsOff").checked,
    magError: v("envDecl"),   // compass error in deg (the FC is set up for Gazebo's compass)
  };
}
function applyEnv() {
  if (!engine) return;
  Object.assign(engine.phys.env, envFromUi());
  engine.opts.upLatencyMs = 2 + (parseFloat($("envLat").value) || 0);
  engine.bridge.opts.burstPeriod = 1 / Math.max(10, parseFloat($("envRate").value) || 50);
  engine.pilotMode = $("pilotProto").value;
  for (const id of ["envWind", "envGust", "envGyroN", "envAccN", "envBaroN", "envGyroB", "envMagN", "envGpsN", "envDecl", "envLat", "envRate"]) {
    const out = $(id + "Val");
    if (out) out.textContent = $(id).value;
  }
}
document.querySelectorAll("#envPanel input, #pilotProto").forEach((el) => el.addEventListener("input", applyEnv));
$("envGpsOff").addEventListener("change", applyEnv);
$("simSpeed").onchange = () => { speed = parseFloat($("simSpeed").value); resync(); };

// ------------------------------------------------------------------ connections
function resync() { if (engine) { wall0 = performance.now(); sim0 = engine.t; } }

async function stopAll() {
  setRcCut(false);
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
  $("psWindRow").style.display = m === "software" ? "" : "none";
  // the RC-lost action is a flight-controller setting: the page can only change it on the software FC
  // (on the real board: OLED MENU > Failsafe > RC lost)
  $("rcAction").disabled = m !== "software";
  if (m === "software" && engine && engine.fc) $("rcAction").value = String(Math.round(engine.fc.get("RC lost")));
}

async function startSoftware() {
  await stopAll();
  $("log").textContent = "";
  try {
    engine = await SimEngine.software({ storageKey: "aerofc_q_settings", env: envFromUi() });
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
  engine.lockstepS = 0.07;
  serial.onData = (b) => engine && engine.serialReceived(b);
  serial.onClose = () => addLog("Serial port closed (board unplugged?)");
  engine.bridge.onLog = addLog;
  engine.bridge.onStatus = (s) => { status = s; statusTime = performance.now(); };
  applyEnv();
  speed = 1; $("simSpeed").value = "1";
  addLog("Port open. Sending HELLO to the Arduino...");
  engine.bridge.start();
  setMode("arduino", "Arduino UNO R4 Minima over USB (hardware in the loop)");
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

// "localhost" tries IPv6 first, and Ubuntu's bridge only listens on IPv4: that wait put 320 ms on every pilot command
// (measured; 127.0.0.1 takes 12 ms), because each command is a new connection
function relayBase() { return $("relayUrl").value.trim().replace(/\/$/, "").replace("//localhost", "//127.0.0.1"); }

function openEvents(base) {
  const es = new EventSource(base + "/events");
  es.onmessage = (ev) => { try { const s = JSON.parse(ev.data); if (s && s.stateName) { status = s; statusTime = performance.now(); } } catch (_) { } };
  es.onerror = () => addLog("No connection to " + base + " (is bridge_plus.py / relay.py running in WSL?)");
  return es;
}

function connectGazebo() {
  const base = relayBase();
  stopAll().then(() => {
    gazebo = { es: openEvents(base), url: base, since: performance.now(), inFlight: 0, postStatus: 0, postAt: 0 };
    setMode("gazebo", "Flying in Gazebo (commands and plots through the bridge)");
    addLog("Gazebo mode: sending pilot input to " + base + "/control");
  });
}

// Gazebo running while this page does something else is an easy mistake to miss, so the page looks for the bridge
async function watchBridge() {
  if (gazebo) {
    if (gazebo.es.readyState === EventSource.CLOSED) gazebo.es = openEvents(gazebo.url);
  } else if (mode === "telemetry") {
    bridgeSeen = false;
  } else {
    try {
      const r = await fetch(relayBase() + "/", { signal: AbortSignal.timeout(1500) });
      bridgeSeen = r.ok;
    } catch (_) { bridgeSeen = false; }
  }
  setTimeout(watchBridge, 3000);
}

$("btnSoftware").onclick = startSoftware;
$("btnArduino").onclick = connectArduino;
$("btnTelemetry").onclick = connectTelemetry;
$("btnGazebo").onclick = connectGazebo;
$("linkAction").onclick = connectGazebo;
$("btnStop").onclick = async () => { await stopAll(); setMode("none", "Stopped"); };
$("camBtn").onclick = () => { view.camMode = view.camMode === "chase" ? "ground" : "chase"; $("camBtn").textContent = view.camMode === "chase" ? "Camera: chase" : "Camera: ground"; };

const cmd = (code) => {
  if (engine) engine.command(code);
  else if (gazebo) pilot.commands.push(code);          // sent with the keys' commands, see servicePilotInput
};
$("cmdArm").onclick = () => cmd(status && status.armed ? CODES.DISARM : CODES.ARM);
$("cmdTakeoff").onclick = () => cmd(CODES.TAKEOFF);
$("cmdLand").onclick = () => cmd(CODES.LAND);
$("cmdKill").onclick = () => cmd(CODES.KILL);
$("cmdManual").onclick = () => pilot.toggleManual();
$("cmdStop").onclick = () => engine && engine.bridge.sendStop();
document.querySelectorAll("#speedSeg button").forEach((b) => { b.onclick = () => cmd(+b.dataset.code); });
$("cmdPosHold").onclick = () => cmd(CODES.POSHOLD);
$("cmdStick").onclick = () => cmd(CODES.STICK);

// ------------------------------------------------------------------ flying by itself
// a command that carries numbers: the flight controller takes them as a target, not as sticks
function sendNumbers(thr, roll, pitch, yaw) {
  if (engine) engine.pilotCodes.push([thr, roll, pitch, yaw]);
  else if (gazebo) for (let i = 0; i < 3; i++) codeQueue.push({ thr, roll, pitch, yaw });
}
const navHeight = () => Math.max(0.5, +$("navHeight").value || 3);
const sendGoto = (north, east) => {
  sendNumbers(CODES.GOTO, north, east, navHeight());
  addLog(`Go to ${north.toFixed(1)} m north, ${east.toFixed(1)} m east, ${navHeight().toFixed(1)} m up`);
};
function showPoints() { $("navPoints").textContent = String(nav.points.length); }
const nav = new NavPanel($("navMap"), (p) => {
  if ($("navAdd").checked) {
    if (nav.points.length < MAX_WAYPOINTS) nav.points.push({ ...p, height: navHeight() });
    showPoints();
  } else {
    nav.target = p;
    $("navClicked").textContent = `${p.north.toFixed(1)} m N, ${p.east.toFixed(1)} m E`;
  }
});
$("navGo").onclick = () => { if (nav.target) sendGoto(nav.target.north, nav.target.east); };
$("navHome").onclick = () => cmd(CODES.RTH);
$("navCancel").onclick = () => cmd(CODES.NAV_CANCEL);
$("navLandMarker").onclick = () => {
  if (status && status.markerSeen) cmd(CODES.MARKER_LAND);
  else addLog("The camera doesn't see the landing pad: fly over it first (the camera world: START_ALL camera)");
};
$("navStart").onclick = () => {
  if (!nav.points.length) return;
  cmd(CODES.MISSION_CLEAR);
  nav.points.forEach((p, i) => sendNumbers(CODES.WAYPOINT, p.north, p.east, p.height + 1000 * i));
  cmd(CODES.MISSION_START);
  addLog(`Mission: ${nav.points.length} points`);
};
$("navClear").onclick = () => { nav.points = []; nav.trail = []; showPoints(); };
// the red pole in Gazebo's world: the bridge asks Gazebo where it is now
$("navMarker").onclick = async () => {
  if (!gazebo) { addLog("The Gazebo marker only exists when flying in Gazebo"); return; }
  try {
    const m = await (await fetch(gazebo.url + "/marker")).json();
    if (!status || status.homeN === undefined) return;
    sendGoto(m.north - status.homeN, m.east - status.homeE);
  } catch (e) { addLog("Could not ask Gazebo where the marker is: " + e.message); }
};

// ------------------------------------------------------------------ the laptop's cameras (optional, off by default)
let visionModule = null;
const panels = {};
async function setCamera(kind, on) {
  const box = $(kind + "On");
  try {
    visionModule = visionModule || await import("./vision.js");
    if (kind === "gesture") {
      panels.gesture = panels.gesture || new visionModule.GesturePanel($("gestureVideo"), $("gestureOverlay"), cmd, (t) => { $("gestureState").textContent = t; }, addLog);
    } else {
      panels.object = panels.object || new visionModule.ObjectPanel($("objectVideo"), $("objectOverlay"), (t) => { $("objectState").textContent = t; }, addLog);
    }
    $(kind + "View").style.display = on ? "" : "none";
    if (on) await panels[kind].start();
    else panels[kind].stop();
  } catch (e) {
    addLog(`Camera problem: ${e.message}`);
    $(kind + "State").textContent = "camera problem: " + e.message;
    $(kind + "View").style.display = "none";
    box.checked = false;
  }
}
$("gestureOn").onchange = () => setCamera("gesture", $("gestureOn").checked);
$("objectOn").onchange = () => setCamera("object", $("objectOn").checked);

const faultWas = {};
function updateFaults(s, age) {
  const rows = faultList(s, age);
  const box = $("faultList");
  if (box.children.length !== rows.length) box.innerHTML = rows.map(() => '<div class="fault"><span></span><b></b></div>').join("");
  rows.forEach((r, i) => {
    const el = box.children[i];
    el.className = "fault" + (r.bad ? " bad" : "");
    el.children[0].textContent = r.name;
    el.children[1].textContent = r.text;
    if (faultWas[r.name] !== undefined && faultWas[r.name] !== r.bad) addLog(`${r.bad ? "FAULT" : "cleared"}: ${r.name}: ${r.text}`);
    faultWas[r.name] = r.bad;
  });
}

// ------------------------------------------------------------------ RC link test
function setRcCut(on) {
  const was = !!rcCutAt;
  rcCutAt = on ? performance.now() : 0;
  if (engine) engine.rcCut = on;
  $("rcCutBtn").textContent = on ? "Restore RC link" : "Cut RC link";
  $("rcCutBtn").classList.toggle("cut", on);
  if (on) addLog("RC LINK CUT: no more pilot packets (simulated transmitter failure)");
  else if (was) addLog("RC link restored");
}
$("rcCutBtn").onclick = () => setRcCut(!rcCutAt);
$("rcAction").onchange = () => {
  if (engine && engine.fc) engine.fc.set("RC lost", +$("rcAction").value);
};

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

const OLED_BLANK = new Uint8Array(1024);
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
function postControl(o) {
  const g = gazebo;
  if (!g) return;
  const done = (code) => { g.postStatus = code; g.postAt = performance.now(); g.inFlight--; };
  g.inFlight++;
  fetch(g.url + "/control", { method: "POST", body: `${o.thr},${o.roll},${o.pitch},${o.yaw}\n`, headers: { "Content-Type": "text/plain" } })
    .then((r) => done(r.status), () => done(0));
}
function sendGazebo(o) {
  if (gazebo && gazebo.inFlight < 3) postControl(o);    // bridge busy: the next stick packet is only 40 ms away, don't pile up
}

// ------------------------------------------------------------------ simulation clock (worker metronome)
const metronome = new Worker(new URL("./metronome.worker.js", import.meta.url));
let nextLog = 0;
metronome.onmessage = () => {
  servicePilotInput(performance.now());
  if (!engine) return;
  const target = sim0 + (performance.now() - wall0) / 1000 * speed;
  let n = 0;
  while (engine.t < target && n < 120) {
    if (engine.waitingForBoard(performance.now())) break;
    engine.step();
    n++;
    if (engine.t >= nextLog) { nextLog = engine.t + 0.02; logRow(engine.t); }
  }
  if (target - engine.t > 0.25) resync();   // computer too slow: do not try to catch up
};
metronome.postMessage("start");

// ------------------------------------------------------------------ pilot input, 50 times a second
// Runs on the metronome, not in the animation loop: animation frames stop when the page is hidden, and the last stick
// values would keep flying the drone. Hidden page = nobody is flying = neutral sticks.
let lastPilotRead = performance.now();
const codeQueue = [];     // command packets waiting to go to Gazebo
let codeWaitFrom = 0;
function servicePilotInput(now) {
  if (now - lastPilotRead < 19) return;
  const dt = Math.min(0.1, (now - lastPilotRead) / 1000);
  lastPilotRead = now;
  // the server stopped, so nobody sees a held key go up: it must not keep flying the drone
  if (keysMsgAt && now - keysMsgAt > 600) { keysMsgAt = 0; keysActive = false; pilot.setRemoteKeys([]); }
  pilot.armed = !!(status && status.armed);
  if (status && status.version === 2) pilot.speedMode = status.speedMode;
  const out = pilot.update(dt, document.hidden);
  const codes = pilot.takeCommands();
  if (engine) {
    engine.setPilot(out);
    codes.forEach((c) => engine.command(c));
  } else if (gazebo && !rcCutAt) {
    // A command goes out three times in case one is lost, with no stick packet between the copies: the flight controller
    // takes the copies as one command only if nothing comes between them, or P and K would switch two or three times.
    codes.forEach((c) => { for (let i = 0; i < 3; i++) codeQueue.push({ thr: c, roll: out.roll, pitch: out.pitch, yaw: out.yaw }); });
    if (codeQueue.length) {
      // a stick packet still on its way could arrive between the copies, so let it land first (but never wait long)
      if (!gazebo.inFlight || now - codeWaitFrom > 100) { postControl(codeQueue.shift()); codeWaitFrom = now; }
    } else {
      codeWaitFrom = now;
      if (now - lastPilotSend > 35) { lastPilotSend = now; sendGazebo(out); }
    }
  }
}

// ------------------------------------------------------------------ frame loop
let lastPlot = 0, lastView = 0, lastChartDraw = 0, lastPanels = 0;
function frame() {
  const now = performance.now();
  if (engine && engine.fc) {
    engine.fc.serviceStorage();
    const fb = engine.fc.render(engine.nowUs());
    if (fb) lastFb = fb;
    if (!engine.fc.displayOn()) drawOled(OLED_BLANK);   // the display-off timer: a real panel goes dark too
    else if (lastFb) drawOled(lastFb);
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
    if (s.version === 2) {
      const ph = s.posActive ? s.posStateName : s.posOn ? "waiting (" + s.posStateName + ")" : "OFF";
      hud.push(`${s.speedModeName}  position hold ${ph}${s.atCeiling ? "  AT HEIGHT LIMIT" : ""}${rcCutAt ? "  RC LINK CUT" : ""}`);
    }
  }
  if (tr) hud.push(`speed ${tr.speed.toFixed(1)} m/s  N ${tr.north.toFixed(1)}  E ${tr.east.toFixed(1)}`);
  // drawing is rationed: while the page draws, it can't answer the board (3D 30 times a second, plots 15, panels 10)
  if (now - lastView > 32) {
    lastView = now;
    if (phys) view.draw(phys, s ? s.motors : (engine ? engine.bridge.motors : null), hud);
    if (s) drawHorizon($("horizon"), s.roll, s.pitch, s.yaw);
  }

  // plots at ~30 Hz
  if (s && now - lastPlot > 33) {
    lastPlot = now;
    const t = engine ? engine.t : now / 1000;
    const thrPlot = s.rcThr >= 1.5 && s.rcThr <= 3.5 ? (s.rcThr - 2.5) * 2 : (s.rcThr <= 1 ? s.rcThr : NaN);
    charts.rc.push(t, [s.rcRoll, s.rcPitch, s.rcYaw, thrPlot]);
    charts.att.push(t, [s.roll, tr ? tr.roll : NaN, s.pitch, tr ? tr.pitch : NaN, s.rollSp]);
    charts.yaw.push(t, [s.yaw, tr ? tr.yaw : NaN, s.headingTarget]);
    charts.alt.push(t, [s.alt, tr ? tr.alt : NaN, s.altTarget, s.vz]);
    charts.mot.push(t, s.motors);
    const i = engine && engine.fc ? engine.info() : null;
    charts.pos.push(t, [s.posActive ? s.holdDist : NaN,
      i && s.posActive && tr ? Math.hypot(tr.north - i.holdN, tr.east - i.holdE) : NaN,
      tr ? tr.speed : Math.hypot(s.velN, s.velE)]);
  }
  if (now - lastChartDraw > 65) {
    lastChartDraw = now;
    Object.values(charts).forEach((c) => c.draw());
  }
  if (now - lastPanels > 99) {
    lastPanels = now;
    updatePanels(s);
  }
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
  updatePosPanel(s);
  updateRcPanel(s);
  nav.update(s);
  $("navState").textContent = navText(s);
  $("navPadSeen").textContent = s && s.markerSeen ? "sees the marker" : "no marker in view";
  updateFaults(s, age);
  updateDiagnostics(s, o);
  updateLinkBanner(s);
  updateThrottleCard(s);
}

function updateLinkBanner(s) {
  const now = performance.now(), g = gazebo;
  const st = linkStatus({
    mode, bridgeSeen, relay: g ? g.url : "",
    connectedFor: g ? (now - g.since) / 1000 : 0,
    postStatus: g && now - g.postAt < 3000 ? g.postStatus : 0,
    telemetryAge: s ? (now - statusTime) / 1000 : 99,
    pilotHz: s ? s.pilotHz : 0,
    rcCut: !!rcCutAt, hidden: document.hidden, focused: document.hasFocus(), pad: !!pilot.gamepadName,
    keysHelper: !!keysMsgAt, keysActive, oldFirmware: !!(s && s.version === 2 && s.navKind === undefined),
  });
  $("linkBanner").style.display = st ? "" : "none";
  if (!st) return;
  $("linkBanner").className = "banner " + st.level;
  $("linkText").textContent = st.text;
  $("linkAction").style.display = st.action ? "" : "none";
}

function updateThrottleCard(s) {
  const v = throttleView(s);
  $("thrStick").textContent = v.stickText;
  // a bar that grows from the centre: right = up, left = down
  $("thrFill").style.left = `${50 + Math.min(v.stick, 0) * 50}%`;
  $("thrFill").style.width = `${Math.abs(v.stick) * 50}%`;
  $("thrPower").textContent = v.powerText;
  $("thrClimb").textContent = s ? `${s.vz >= 0 ? "+" : ""}${s.vz.toFixed(2)} m/s` : "-";
  [["FR", 0], ["RL", 1], ["FL", 2], ["RR", 3]].forEach(([n, k]) => {      // telemetry order: FR RL FL RR
    const m = s && s.motors ? s.motors[k] : 0;
    $("m" + n).style.width = `${m * 100}%`;
    $("m" + n + "v").textContent = s ? `${Math.round(m * 100)} %` : "-";
  });
}

// Answers "why is it drifting?" without guessing: is a stick being read as a push, is the link slow, is the estimate off
let pushSince = 0, speedWall = 0, speedSim = 0, simSpeed = 1;
function updateDiagnostics(s, o) {
  $("padRawRow").style.display = pilot.padRaw ? "" : "none";
  if (pilot.padRaw) $("stPadRaw").textContent = "pad raw  " + pilot.padRaw.map((v) => v.toFixed(2)).join("  ");
  let warn = "";
  if (pilot.padRest > PAD_DEADZONE) {
    warn = `Stick not centred (rests at ${pilot.padRest.toFixed(2)}): the drone will move. Use the keyboard or unplug the gamepad.`;
  } else if (s && s.posActive && s.posState === 2 && !o.roll && !o.pitch) {       // MOVING, but the page sends no stick
    if (!pushSince) pushSince = performance.now();
    if (performance.now() - pushSince > 2000) warn = `Flight controller sees a stick push (roll ${s.rcRoll.toFixed(2)}, pitch ${s.rcPitch.toFixed(2)}) but the page sends none.`;
  } else {
    pushSince = 0;
  }
  $("stickWarn").textContent = warn;
  $("stickWarn").style.display = warn ? "" : "none";
  $("usbRow").style.display = mode === "arduino" ? "" : "none";
  if (engine && mode === "arduino") {
    const b = engine.bridge, wall = performance.now();
    if (wall - speedWall > 1000) {          // the simulation slows down instead of letting the drone wobble
      simSpeed = (engine.t - speedSim) / ((wall - speedWall) / 1000);
      speedWall = wall;
      speedSim = engine.t;
    }
    $("stUsb").textContent = `${b.controlDelayMs.toFixed(0)} ms (recent max ${b.controlDelayMax.toFixed(0)}), sim speed ${simSpeed.toFixed(2)}x`;
  }
  if (engine && s && s.version === 2) {
    const e = estimateError(s, engine.phys.truth());
    $("stEstErr").textContent = `${e.pos.toFixed(2)} m, ${e.att.toFixed(1)} deg`;
  } else $("stEstErr").textContent = "-";
  $("stHeadOff").textContent = s && Number.isFinite(s.yawOffset) ? `${s.yawOffset.toFixed(1)} deg` : "-";
}

function updatePosPanel(s) {
  const v2 = !!(s && s.version === 2);
  document.querySelectorAll("#speedSeg button").forEach((b) => b.classList.toggle("on", v2 && +b.dataset.code === 17 + s.speedMode));
  $("cmdPosHold").classList.toggle("on", v2 && s.posOn);
  $("cmdPosHold").textContent = `Position hold: ${v2 ? (s.posOn ? "ON" : "OFF") : "-"} (P)`;
  $("cmdStick").textContent = `Sticks: ${v2 ? (s.stickAngle ? "ANGLE" : "SPEED") : "-"} (K)`;
  if (!v2) {
    for (const id of ["psOn", "psState", "psDist", "psSpeed", "psNE", "psMode", "psCeil", "psWind"]) $(id).textContent = "-";
    return;
  }
  $("psOn").textContent = s.posActive ? "ACTIVE" : s.posOn ? "ON (not flying it now)" : "OFF";
  $("psOn").className = s.posActive ? "armed" : "";
  $("psState").textContent = s.posStateName;
  $("psDist").textContent = s.posActive ? `${s.holdDist.toFixed(2)} m` : "-";
  $("psSpeed").textContent = `${Math.hypot(s.velN, s.velE).toFixed(2)} m/s`;
  $("psNE").textContent = `${s.posN.toFixed(1)} / ${s.posE.toFixed(1)} m`;
  $("psMode").textContent = `${s.speedModeName} / ${s.stickAngle ? "ANGLE" : "SPEED"}`;
  $("psCeil").textContent = s.altLimit ? `${s.maxAlt} m above take-off${s.atCeiling ? " - AT LIMIT" : ""}` : "OFF (up to 1000 m)";
  if (engine && engine.fc) {
    const i = engine.info();
    $("psWind").textContent = `${Math.hypot(i.windN, i.windE).toFixed(2)} m/s2 of push`;
  }
}

function updateRcPanel(s) {
  const timeout = engine && engine.fc ? engine.fc.get("Link t/o") : 2.0;
  if (!rcCutAt) $("rcState").textContent = mode === "none" ? "-" : "connected";
  else {
    const el = (performance.now() - rcCutAt) / 1000;
    $("rcState").textContent = el < timeout ? `CUT - failsafe in ${(timeout - el).toFixed(1)} s` : `CUT for ${el.toFixed(0)} s`;
  }
  let fs = "-";
  if (s) {
    if (s.navKindName === "RTH" && s.reasonName === "pilot link") fs = `going home: ${s.navStageName}`;
    else if (s.failsafe) fs = `${s.stateName} (${s.reasonName})${s.posActive ? ", holding position" : ""}`;
    else if (s.reasonName === "pilot link" && !s.armed) fs = "landed and disarmed (pilot link)";
    else if (s.linkLost) fs = "link lost (RC lost action NONE)";
  }
  $("rcFs").textContent = fs;
  $("rcFs").className = s && s.failsafe ? "armed" : "";
}

window.addEventListener("resize", () => Object.values(charts).forEach((c) => c.draw()));
setMode("none", "Choose how to fly (left panel)");
requestAnimationFrame(frame);
const qs = new URLSearchParams(location.search);
if (qs.get("relay")) $("relayUrl").value = qs.get("relay");
if (qs.get("autostart") === "software" || qs.get("mode") === "software") startSoftware();
else if (qs.get("mode") === "gazebo") connectGazebo();
watchBridge();
window.__app = { get engine() { return engine; }, get status() { return status; }, pilot, startSoftware };
