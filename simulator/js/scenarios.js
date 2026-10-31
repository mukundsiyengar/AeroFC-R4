// scenarios.js - automated closed-loop tests: the real firmware (wasm) flies the X3 replica faster than real time.
import { SimEngine, CODES } from "./sim_engine.js";
import { crc32, buildPacket, Parser, T } from "./protocol.js";
import { Pilot, PAD_DEADZONE } from "./pilot_input.js";
import { linkStatus, throttleView } from "./page_status.js";

const wrap180 = (a) => { while (a > 180) a -= 360; while (a < -180) a += 360; return a; };

class Check {
  constructor() { this.items = []; this.metrics = {}; }
  ok(cond, msg) { this.items.push({ pass: !!cond, msg }); return !!cond; }
  metric(name, v, unit = "") { this.metrics[name] = (typeof v === "number" ? +v.toFixed(3) : v) + (unit ? " " + unit : ""); }
  get pass() { return this.items.length > 0 && this.items.every((i) => i.pass); }
}

async function engine(opts = {}) {
  const e = await SimEngine.software(opts);
  if (opts.params) for (const [k, v] of Object.entries(opts.params)) e.fc.set(k, v);
  e.bridge.start();
  return e;
}

// start the bridge, wait for auto take-off to finish and the drone to hover
async function airborne(opts = {}) {
  const e = await engine(opts);
  const tt = e.until((s) => { const i = s.info(); return i.state === 3 && Math.abs(i.alt - 1.5) < 0.1; }, 15);
  e.takeoffTime = tt;
  e.run(2.0);
  return e;
}

// a pilot that keeps moving the sticks: hands off, flying around, turning (same seed = same flight)
function randomPilot(seed) {
  let r = seed, until = 0;
  const rnd = () => { r = (r * 1664525 + 1013904223) >>> 0; return r / 4294967296; };
  return (e, sec) => {
    if (sec < until) return;
    const a = rnd();
    if (a < 0.45) { e.setPilot({ pitch: 0, roll: 0, yaw: 0 }); until = sec + 3 + Math.floor(rnd() * 12); }
    else if (a < 0.65) { e.setPilot({ pitch: (rnd() - 0.5) * 2, roll: (rnd() - 0.5) * 2, yaw: 0 }); until = sec + 1 + Math.floor(rnd() * 5); }
    else if (a < 0.85) { e.setPilot({ pitch: rnd() - 0.5, roll: rnd() - 0.5, yaw: (rnd() - 0.5) * 1.5 }); until = sec + 1 + Math.floor(rnd() * 6); }
    else { e.setPilot({ pitch: 0, roll: 0, yaw: (rnd() - 0.5) * 2 }); until = sec + 1 + Math.floor(rnd() * 4); }
  };
}

// lets go of the sticks, waits until it has stopped, then returns the largest distance from the hold point
function letGoAndHold(e, seconds = 30) {
  e.setPilot({ pitch: 0, roll: 0, yaw: 0, thr: 2.5 });
  e.until((s) => s.info().posState === 4 && s.phys.truth().speed < 0.3, 15);
  e.run(3);
  let worst = 0;
  e.run(seconds, (s) => { const i = s.info(), tr = s.phys.truth(); worst = Math.max(worst, Math.hypot(tr.north - i.holdN, tr.east - i.holdE)); }, 0.1);
  return worst;
}

// The real setup in virtual time: the same firmware behind a USB link with a delay, and the page's event loop
// (metronome ticks, drawing that blocks everything for frameMs).  This is how the page flies a real board.
export async function boardRig({ usbMs = 5, frameMs = 10, env = {}, lockstep = 0.07 } = {}) {
  const e = await SimEngine.software({ env });
  const fc = e.fc;
  let wall = 0, wall0 = 0, sim0 = 0, nextTick = 0, nextFrame = 0, busyUntil = 0, unplugged = false;
  const rx = [], tx = [];
  e.attachSerial({ write: (b) => rx.push({ at: wall + usbMs / 2, b }) });
  e.bridge.wallNow = () => wall;
  e.lockstepS = lockstep;
  e.bridge.start();
  const run = (seconds, every = null) => {
    const end = wall + seconds * 1000;
    while (wall < end) {
      wall += 1;
      const us = (wall * 1000) >>> 0;
      while (rx.length && rx[0].at <= wall) fc.feed(rx.shift().b, us);    // the board never waits for the page
      fc.tick(us);
      const out = fc.poll(us);
      if (out && !unplugged) tx.push({ at: wall + usbMs / 2, b: out });
      if (wall < busyUntil) continue;                                     // the page is drawing
      while (tx.length && tx[0].at <= wall) e.serialReceived(tx.shift().b);
      if (wall >= nextTick) {
        nextTick = wall + 4;
        const target = sim0 + (wall - wall0) / 1000;
        for (let n = 0; e.t < target && n < 120 && !e.waitingForBoard(wall); n++) e.step();
        if (target - e.t > 0.25) { wall0 = wall; sim0 = e.t; }
        if (every) every();
      }
      if (wall >= nextFrame) { nextFrame = wall + 16.7; busyUntil = wall + frameMs; }
    }
  };
  return { e, fc, run, info: () => e.info(fc), wall: () => wall, unplug: () => { unplugged = true; } };
}

// holds the climb stick down until the motors stop, and reports how the landing went
function stickLanding(e, thr = 2.0, limit = 90) {
  const a = e.phys.truth(), t0 = e.t;
  const r = { time: -1, touchSpeed: 0, fastest: 0, side: 0, bounce: 0, minMotor: 1 };
  let touched = false;
  e.setPilot({ thr });
  e.run(limit, (s) => {
    const tr = s.phys.truth(), vz = s.phys.v[2];
    r.side = Math.max(r.side, Math.hypot(tr.north - a.north, tr.east - a.east));
    r.fastest = Math.max(r.fastest, -vz);
    if (!touched && tr.alt > 0.1) r.minMotor = Math.min(r.minMotor, ...s.info().motors);
    if (!touched && tr.alt < 0.03) { touched = true; r.touchSpeed = Math.abs(vz); }
    if (touched) r.bounce = Math.max(r.bounce, tr.alt);
    if (s.info().state !== 0) return true;
    r.time = s.t - t0;
    return false;
  }, 0.01);
  return r;
}

function pct(arr, q) {
  if (!arr.length) return NaN;
  const a = arr.slice().sort((x, y) => x - y);
  return a[Math.min(a.length - 1, Math.floor(q * a.length))];
}

function stats(arr) {
  const n = arr.length || 1;
  const mean = arr.reduce((a, b) => a + b, 0) / n;
  const sd = Math.sqrt(arr.reduce((a, b) => a + (b - mean) * (b - mean), 0) / n);
  return { mean, sd, min: Math.min(...arr), max: Math.max(...arr), rms: Math.sqrt(arr.reduce((a, b) => a + b * b, 0) / n) };
}

// step response metrics on samples [{t, y}] for a step from y0 to y1 starting at t0
function stepMetrics(samples, y0, y1, t0) {
  const span = y1 - y0;
  let t10 = null, t90 = null, peak = y0, settle = null;
  for (const s of samples) {
    const f = (s.y - y0) / span;
    if (t10 === null && f >= 0.1) t10 = s.t;
    if (t90 === null && f >= 0.9) t90 = s.t;
    if ((span > 0 && s.y > peak) || (span < 0 && s.y < peak)) peak = s.y;
  }
  for (let i = samples.length - 1; i >= 0; i--) {
    if (Math.abs(samples[i].y - y1) > Math.abs(span) * 0.1) { settle = (i + 1 < samples.length ? samples[i + 1].t : samples[i].t) - t0; break; }
  }
  if (settle === null) settle = 0;
  return { rise: t10 !== null && t90 !== null ? t90 - t10 : NaN, overshoot: Math.abs(span) > 0 ? Math.max(0, (peak - y1) / span) * 100 : 0, settle };
}

export const SCENARIOS = [
  {
    name: "Protocol: CRC32 + framing match zlib / official bridge",
    async run(c) {
      const golden = await fetch(new URL("../../tests/golden_vectors.json", import.meta.url)).then((r) => r.json()).catch(() => null);
      c.ok(crc32(new TextEncoder().encode("123456789")) === 0xcbf43926, "CRC32('123456789') = CBF43926");
      if (golden) {
        let all = true;
        for (const g of golden.packets) {
          const pl = Uint8Array.from(g.payload_hex.match(/../g) || [], (h) => parseInt(h, 16));
          const pk = buildPacket(g.type, g.seq, BigInt(g.ts), pl);
          const hex = Array.from(pk, (b) => b.toString(16).padStart(2, "0")).join("");
          if (hex !== g.packet_hex) { all = false; c.ok(false, "golden packet type " + g.type + " differs"); }
        }
        c.ok(all, `${golden.packets.length} packets byte-identical to the official hil_protocol.py`);
      } else c.ok(true, "(golden vectors file not found - skipped)");
      // corrupted stream: garbage + bit flip + good packet -> exactly one good packet
      const good = buildPacket(T.HEARTBEAT, 7, 123n);
      const bad = buildPacket(T.HEARTBEAT, 8, 124n); bad[10] ^= 0x40;
      const p = new Parser();
      const got = p.push(new Uint8Array([1, 2, 0x48, 0x48, 0x49, 9, ...bad, ...good]));
      c.ok(got.length === 1 && got[0].seq === 7 && p.crcErrors >= 1, "resync after garbage and CRC error");
      // the FIRMWARE parser: feed it the same corrupted stream, then a HELLO, it must answer HELLO_ACK
      const e = await SimEngine.software();
      e.fc.feed(new Uint8Array([0x48, 0x49, 0x01, 0xff, ...bad]), 1000);
      e.fc.feed(buildPacket(T.HELLO, 1, 5n, new TextEncoder().encode("UAV_HIL_V1")), 2000);
      const out = e.fc.poll(3000);
      const pk = out ? new Parser().push(out) : [];
      c.ok(pk.length >= 1 && pk[0].type === T.HELLO_ACK && new TextDecoder().decode(pk[0].payload) === "UAV_HIL_V1",
        "firmware answers HELLO with HELLO_ACK 'UAV_HIL_V1'");
      c.metric("fw CRC errors counted", e.info().crcErrors);
    },
  },
  {
    name: "Handshake, motor packet per burst, heartbeat 5 Hz",
    async run(c) {
      const e = await engine({ params: { "Auto TO": 0 } });
      e.run(3.0);
      const b = e.bridge;
      c.ok(b.state === "RUNNING", "HELLO -> HELLO_ACK -> START, session RUNNING");
      c.ok(b.log.some((l) => l.includes("HIL session started")), "bridge log: 'HIL session started successfully'");
      const ratio = b.counts.actuator / b.counts.sensorBursts;
      c.metric("actuator packets / sensor bursts", ratio);
      c.ok(ratio > 0.95 && ratio < 1.05, "one motor command per sensor burst");
      c.metric("max heartbeat gap", b.heartbeatGapMax, "s");
      c.ok(b.counts.heartbeat >= 13 && b.heartbeatGapMax < 0.25, "heartbeat every 200 ms");
      c.ok(b.counts.status > 40, "STATUS telemetry arrives (" + b.counts.status + ")");
      const i = e.info();
      c.ok(i.ekfInit && i.state === 0, "zero packets at start handled, EKF initialised, still disarmed (auto take-off off)");
      c.ok(b.motors.every((m) => m === 0), "motors 0 while disarmed");
    },
  },
  {
    name: "Auto take-off to 1.5 m after START, then hover",
    async run(c) {
      const e = await engine();
      const samples = [];
      const tt = e.until((s) => s.info().state === 3, 15, (s) => samples.push({ t: s.t, y: s.phys.truth().alt }));
      c.metric("take-off time", tt, "s");
      e.run(4, (s) => samples.push({ t: s.t, y: s.phys.truth().alt }));
      const m = stepMetrics(samples, 0, 1.5, 0);
      c.metric("altitude overshoot", m.overshoot, "%");
      c.ok(tt > 0 && tt < 8, "reaches FLYING in under 8 s");
      const peak = Math.max(...samples.map((s) => s.y));
      c.metric("peak altitude", peak, "m");
      c.ok(peak < 1.75, "overshoot < 0.25 m");
      const alts = [], tilts = [];
      e.run(5, (s) => { const tr = s.phys.truth(); alts.push(tr.alt); tilts.push(tr.tilt); });
      const a = stats(alts), tl = stats(tilts);
      c.metric("hover altitude mean", a.mean, "m");
      c.metric("hover altitude std", a.sd, "m");
      c.metric("hover tilt max", tl.max, "deg");
      c.ok(Math.abs(a.mean - 1.5) < 0.05 && a.sd < 0.03, "hover 1.5 m +-5 cm");
      c.ok(tl.max < 1.0, "hover level within 1 deg");
      c.metric("learned hover cmd", e.bridge.status ? e.bridge.status.hoverCmd : NaN);
    },
  },
  {
    name: "Directions: roll + right, pitch - forward, yaw + clockwise (mixer sign test)",
    async run(c) {
      const e = await airborne({ params: { "Pos hold": 0 } });
      // the drone spawns facing East (heading 90): forward = East, right = South
      let tr0 = e.phys.truth();
      e.setPilot({ roll: 0.4 });
      e.run(1.5);
      let tr = e.phys.truth();
      c.metric("roll after +0.4 stick", tr.roll, "deg");
      c.ok(tr.roll > 8 && tr.roll < 11.5, "roll stick +0.4 -> right side down ~10 deg (25 deg at full stick)");
      c.ok(tr.north < tr0.north - 0.1, "and the drone moves to its right (south)");
      e.setPilot({ roll: 0 }); e.run(3);
      tr0 = e.phys.truth();
      e.setPilot({ pitch: -0.4 });
      e.run(1.5);
      tr = e.phys.truth();
      c.metric("pitch after -0.4 stick", tr.pitch, "deg");
      c.ok(tr.pitch < -8 && tr.pitch > -11.5, "pitch stick -0.4 -> nose down ~10 deg");
      c.ok(tr.east > tr0.east + 0.1, "and the drone moves forward (east)");
      e.setPilot({ pitch: 0 }); e.run(3);
      const h0 = e.phys.truth().yaw;
      e.setPilot({ yaw: 0.5 });
      e.run(2.0);
      const dh = wrap180(e.phys.truth().yaw - h0);
      c.metric("heading change, yaw +0.5 for 2 s", dh, "deg");
      c.ok(dh > 60 && dh < 110, "yaw stick + -> turns clockwise ~45 deg/s");
      e.setPilot({ yaw: 0 });
      e.run(0.5);
      const hRelease = e.phys.truth().yaw;
      e.run(4);
      const drift = wrap180(e.phys.truth().yaw - hRelease);
      c.metric("heading drift 4 s after release", drift, "deg");
      c.ok(Math.abs(drift) < 4, "heading hold after the stick is released");
    },
  },
  {
    name: "Attitude step response + estimator accuracy",
    async run(c) {
      const e = await airborne({ params: { "Pos hold": 0 } });
      const samples = [], errR = [], errP = [];
      const t0 = e.t;
      e.setPilot({ roll: 0.6 });   // ~13 deg after deadband + expo
      e.run(0.1);
      const target = e.info().rollSp;
      c.metric("roll setpoint", target, "deg");
      e.run(2.4, (s) => {
        const tr = s.phys.truth(), i = s.info();
        samples.push({ t: s.t - t0, y: tr.roll });
        errR.push(i.roll - tr.roll); errP.push(i.pitch - tr.pitch);
      });
      const m = stepMetrics(samples, 0, target, 0);
      c.metric("roll rise time 10-90%", m.rise, "s");
      c.metric("roll overshoot", m.overshoot, "%");
      c.metric("roll settle (10%)", m.settle, "s");
      c.ok(m.rise < 0.5 && m.overshoot < 25 && m.settle < 1.5, "fast, well damped roll response");
      e.setPilot({ roll: 0 });
      e.run(2);
      // sustained 15 deg pitch for 5 s: the accelerometer is misleading during the acceleration, the EKF must hold
      e.setPilot({ pitch: -0.6 });
      e.run(5, (s) => { const tr = s.phys.truth(), i = s.info(); errR.push(i.roll - tr.roll); errP.push(i.pitch - tr.pitch); });
      e.setPilot({ pitch: 0.6 }); e.run(3.0); e.setPilot({ pitch: 0 }); e.run(2);
      const er = stats(errR), ep = stats(errP);
      c.metric("EKF roll error RMS", er.rms, "deg");
      c.metric("EKF pitch error RMS / max", `${ep.rms.toFixed(2)} / ${Math.max(Math.abs(ep.min), Math.abs(ep.max)).toFixed(2)}`, "deg");
      c.ok(er.rms < 1 && Math.max(Math.abs(ep.min), Math.abs(ep.max)) < 2, "attitude estimate within 2 deg during a 5 s 15-deg tilt");
    },
  },
  {
    name: "Climb / descend stick in altitude hold",
    async run(c) {
      const e = await airborne();
      const a0 = e.phys.truth().alt;
      e.setPilot({ thr: 3.0 });     // full climb
      e.run(2);
      e.setPilot({ thr: 2.5 });
      e.run(0.5);
      const aHold = e.phys.truth().alt;
      c.metric("climbed", aHold - a0, "m");
      c.ok(aHold - a0 > 2.8 && aHold - a0 < 4.6, "climbs ~2 m/s (NORMAL speed mode)");
      e.run(3);
      c.metric("drift 3 s after release", e.phys.truth().alt - aHold, "m");
      c.ok(Math.abs(e.phys.truth().alt - aHold) < 0.15, "holds the new altitude");
      e.setPilot({ thr: 2.0 }); e.run(2); e.setPilot({ thr: 2.5 }); e.run(3);
      c.metric("after descend", e.phys.truth().alt, "m");
      c.ok(e.phys.truth().alt < aHold - 1.5, "descends with the stick");
    },
  },
  {
    name: "Stock control_input.py semantics (throttle 0 = hold, 0.66 = manual, sticky +-0.1)",
    async run(c) {
      const e = await airborne({ params: { "Pos hold": 0 } });
      e.pilotMode = "stock";
      e.setPilot({ thr: 0, roll: 0, pitch: 0, yaw: 0 });
      e.run(1);
      e.setPilot({ thr: 0.66 });
      e.run(2);
      let i = e.info();
      c.ok(i.mode === 1, "throttle 0.66 -> MANUAL mode");
      c.metric("altitude after 2 s manual 0.66", e.phys.truth().alt, "m");
      c.ok(Math.abs(e.phys.truth().alt - 1.5) < 0.8, "0.66 roughly hovers");
      e.setPilot({ thr: 0 });
      e.run(0.3);
      const aCap = e.phys.truth().alt;
      e.run(3);
      i = e.info();
      c.ok(i.mode === 0, "throttle 0 -> back to ALT HOLD");
      c.ok(Math.abs(e.phys.truth().alt - aCap) < 0.25, "holds the altitude where it was switched");
      e.setPilot({ roll: 0.1 });
      e.run(2);
      c.metric("roll with sticky +0.1", e.phys.truth().roll, "deg");
      c.ok(e.phys.truth().roll > 1.5 && e.phys.truth().roll < 3.5, "+0.1 roll = ~2 deg (sticky step of the stock tool)");
      e.setPilot({ roll: 0 }); e.run(2);
      c.ok(e.phys.truth().tilt < 1.5 && e.info().state === 3, "no pilot-link failsafe with the stock (on-change) tool");
    },
  },
  {
    name: "Land command: descent, touch-down detection, auto-disarm",
    async run(c) {
      const e = await airborne();
      e.command(CODES.LAND);
      let maxImpact = 0;
      e.phys.maxImpact = 0;
      const t = e.until((s) => s.info().state === 0, 20);
      maxImpact = e.phys.maxImpact / 9.8;
      e.run(0.1);
      const i = e.info();
      c.metric("landing time from 1.5 m", t, "s");
      c.metric("touch-down peak accel", maxImpact, "g");
      c.ok(t > 0 && t < 10, "lands and disarms in under 10 s");
      c.ok(i.reason === 9, "disarm reason = landed");
      c.ok(e.phys.truth().alt < 0.05, "on the ground");
      c.ok(e.bridge.motors.every((m) => m === 0), "motors stopped");
      // take off again with the TAKEOFF command
      e.run(1);
      e.command(CODES.TAKEOFF);
      const t2 = e.until((s) => s.info().state === 3, 12);
      c.ok(t2 > 0, "TAKEOFF command flies again");
    },
  },
  {
    name: "Failsafes: FC silent -> bridge stop + HELLO recovery; STOP; KILL; pilot link loss",
    async run(c) {
      // 1) FC output blocked for 2 s (like a frozen USB link)
      let e = await airborne();
      const fc = e.fc;
      e.fc = null;   // the FC is "frozen": nothing in, nothing out
      e.run(2.0);
      c.ok(e.bridge.state === "FAILSAFE" && e.bridge.motors.every((m) => m === 0), "bridge: 1.5 s without FC packets -> motors 0");
      e.fc = fc;
      e.toFc = [];
      e.run(3.0);
      c.ok(e.bridge.log.some((l) => l.includes("Arduino sent HELLO during session")), "FC sends HELLO, bridge re-acknowledges");
      c.ok(e.bridge.state === "RUNNING", "session running again");
      // 2) STOP while flying -> motors off immediately
      e = await airborne();
      e.bridge.sendStop();
      e.run(0.2);
      c.ok(e.info().state === 0 && e.info().reason === 6, "STOP -> disarm (reason STOP)");
      // 3) KILL
      e = await airborne();
      e.command(CODES.KILL);
      e.run(0.3);
      c.ok(e.info().state === 0 && e.info().reason === 7, "KILL code -> motors off");
      // 4) pilot link loss: our 25 Hz stream stops for > 2 s -> automatic landing
      e = await airborne();
      e.run(1.5);
      c.ok(e.info().streamSeen, "pilot stream detected");
      e.pilotMode = "off";
      const t = e.until((s) => s.info().state === 0, 20);
      c.metric("link loss -> landed", t, "s");
      c.ok(t > 2 && e.info().reason === 2, "pilot link lost -> failsafe landing, disarmed (reason pilot link)");
    },
  },
  {
    name: "Crash detection ON / OFF",
    async run(c) {
      let e = await airborne();
      e.phys.w = [25, 0, 0];     // violent hit: flips the drone
      e.run(1.5);
      let i = e.info();
      c.ok(i.state === 0 && i.reason === 5 && i.crashed, "crash detected -> motors off (reason CRASH)");
      e = await airborne({ params: { "Crash det": 0 } });
      e.phys.w = [25, 0, 0];
      e.run(0.4);
      i = e.info();
      c.ok(i.reason !== 5, "with crash detection OFF it does not cut the motors");
    },
  },
  {
    name: "Robustness: latency, 40 Hz bursts, inertia +-30 %, noise + bias, wind, 800 rad/s world",
    async run(c) {
      const cases = [
        { name: "+30 ms latency", opts: { upLatencyMs: 32 } },
        { name: "40 Hz bursts + jitter", opts: { bridge: { burstPeriod: 0.025, burstJitter: 0.003 } } },
        { name: "inertia x1.3", opts: { env: { inertiaScale: 1.3 } } },
        { name: "inertia x0.7", opts: { env: { inertiaScale: 0.7 } } },
        { name: "mass +15 %", opts: { env: { massScale: 1.15 } } },
        { name: "sensor noise + gyro bias", opts: { env: { gyroNoise: 0.01, accNoise: 0.3, baroNoise: 0.1, magNoise: 1e-6, gyroBias: [0.02, -0.015, 0.01] } } },
        { name: "wind 3 m/s + gusts", opts: { env: { wind: [3, 0, 0], gust: 1.0 } } },
        { name: "800 rad/s world, motor max 0.8", opts: { env: { maxRotVelocity: 800 }, params: { "Motor max": 0.8 } } },
        { name: "slow motors x2", opts: { env: { motorTauScale: 2 } } },
      ];
      for (const k of cases) {
        const e = await airborne({ ...k.opts, params: { "Pos hold": 0, ...(k.opts.params || {}) } });
        const tilts = [], alts = [];
        e.setPilot({ roll: 0.5 }); e.run(1.5, (s) => tilts.push(s.phys.truth().tilt));
        e.setPilot({ roll: 0, pitch: -0.5 }); e.run(1.5, (s) => tilts.push(s.phys.truth().tilt));
        e.setPilot({ pitch: 0, yaw: 0.6 }); e.run(1.5);
        e.setPilot({ yaw: 0 }); e.run(3, (s) => { tilts.push(s.phys.truth().tilt); alts.push(s.phys.truth().alt); });
        const ok = e.info().state === 3 && Math.max(...tilts) < 22 && Math.abs(stats(alts).mean - 1.5) < 0.3;
        c.metric(k.name, `take-off ${e.takeoffTime.toFixed(1)} s, max tilt ${Math.max(...tilts).toFixed(1)}, alt ${stats(alts).mean.toFixed(2)}`);
        c.ok(ok && e.takeoffTime > 0, k.name + ": stable");
      }
    },
  },
  {
    name: "Settings, OLED menu and buttons (virtual board)",
    async run(c) {
      const e = await SimEngine.software();
      const fb = e.fc.render(1000);
      c.ok(fb && fb.some((b) => b !== 0), "OLED dashboard draws");
      // open MENU -> Rate PID (item 3) -> edit Roll P -> UP x2 -> done
      const B = (b, ev = 1) => e.fc.button(b, ev, 2000);
      const p0 = e.fc.get("Roll P");
      B(2); B(1); B(1); B(1); B(1); B(1); B(2); B(2); B(0); B(0); B(2);   // MENU > Rate PID > Roll P
      const p1 = e.fc.get("Roll P");
      c.metric("Roll P before / after", `${p0} / ${p1}`);
      c.ok(Math.abs(p1 - p0 - 0.01) < 1e-6, "menu edit changes the parameter (2 steps of 0.005)");
      // settings blob round trip into a fresh board
      const n = e.fc.x.fc_settings_pack();
      const blob = e.fc.mem().slice(e.fc.setPtr, e.fc.setPtr + n);
      const e2 = await SimEngine.software();
      e2.fc.mem().set(blob, e2.fc.setPtr);
      c.ok(e2.fc.x.fc_settings_unpack(n) === 1 && Math.abs(e2.fc.get("Roll P") - p1) < 1e-6, `settings blob (${n} B) restores on another board`);
      blob[20] ^= 1;
      e2.fc.mem().set(blob, e2.fc.setPtr);
      c.ok(e2.fc.x.fc_settings_unpack(n) === 0, "corrupted settings are rejected (CRC)");
      // arm by holding SELECT on the home page is refused without sensors
      const e3 = await SimEngine.software();
      e3.fc.button(2, 2, 1000);
      c.ok(e3.info().state === 0, "hold SELECT without simulator data: arming refused");
    },
  },
  // ===================================================================== v2: position hold, speed modes, ceiling ...
  {
    name: "Position hold: calm hover, then wind steps 5 and 8 m/s",
    async run(c) {
      const e = await airborne();
      c.ok(e.info().posActive, "position hold active after take-off (GPS + compass ready)");
      const errs = [], tilts = [];
      e.run(20, (s) => { const i = s.info(), tr = s.phys.truth(); errs.push(Math.hypot(tr.north - i.holdN, tr.east - i.holdE)); tilts.push(tr.tilt); });
      c.metric("calm: 95 % position error", pct(errs, 0.95), "m");
      c.metric("calm: tilt RMS", stats(tilts).rms, "deg");
      c.ok(pct(errs, 0.95) < 0.3 && stats(tilts).rms < 1, "calm hover: 95 % within 0.3 m, tilt RMS < 1 deg");
      for (const [w, lim] of [[5, 1.5], [8, 2.5]]) {
        e.phys.env.wind = [w, 0, 0];
        e.phys.env.gust = w > 6 ? 1.5 : 0;
        let peak = 0;
        const tail = [];
        e.run(18, (s) => {
          const i = s.info(), tr = s.phys.truth();
          const d = Math.hypot(tr.north - i.holdN, tr.east - i.holdE);
          peak = Math.max(peak, d);
          if (s.t > 0) tail.push(d);
        });
        const last = tail.slice(-150);
        c.metric(`wind ${w} m/s: peak / final error`, `${peak.toFixed(2)} / ${stats(last).mean.toFixed(2)} m`);
        c.metric(`wind ${w} m/s: learned wind push`, Math.hypot(e.info().windN, e.info().windE), "m/s2");
        c.ok(peak < lim && stats(last).mean < (w > 6 ? 0.5 : 0.3), `wind ${w} m/s: peak < ${lim} m, back on the spot`);
      }
    },
  },
  {
    name: "SPEED sticks: 5 m/s forward, release -> brakes and holds",
    async run(c) {
      const e = await airborne();
      e.setPilot({ pitch: -1 });
      e.run(6);
      const v = e.phys.truth().speed;
      c.metric("speed with full forward stick (NORMAL)", v, "m/s");
      c.ok(Math.abs(v - 5) < 0.6, "full stick = 5 m/s in NORMAL");
      c.ok(e.phys.v[0] > 4, "moves forward (the nose points east)");
      const rel = e.phys.truth();
      e.setPilot({ pitch: 0 });
      const ts = e.until((s) => s.phys.truth().speed < 0.3, 10);
      const stop = e.phys.truth();
      c.metric("time to stop", ts, "s");
      c.metric("stopping distance", Math.hypot(stop.north - rel.north, stop.east - rel.east), "m");
      let over = 0;
      e.run(6, (s) => { const i = s.info(), tr = s.phys.truth(); over = Math.max(over, Math.hypot(tr.north - i.holdN, tr.east - i.holdE)); });
      c.metric("max distance from the hold point after stopping", over, "m");
      c.ok(ts > 0 && ts < 5 && over < 0.6, "stops within 5 s, then stays within 0.6 m of its spot");
    },
  },
  {
    name: "ANGLE sticks with position hold: tilt like v1, release -> brakes and holds",
    async run(c) {
      const e = await airborne({ params: { "Stick": 1 } });
      e.setPilot({ pitch: -0.6 });
      let maxTilt = 0;
      e.run(2.5, (s) => { maxTilt = Math.max(maxTilt, -s.phys.truth().pitch); });
      c.metric("nose-down tilt with 0.6 stick", maxTilt, "deg");
      c.ok(maxTilt > 13 && maxTilt < 17, "0.6 stick = ~15 deg tilt (25 deg max in NORMAL)");
      c.metric("speed at release", e.phys.truth().speed, "m/s");
      e.setPilot({ pitch: 0 });
      const ts = e.until((s) => s.phys.truth().speed < 0.3, 12);
      let over = 0;
      e.run(6, (s) => { const i = s.info(), tr = s.phys.truth(); over = Math.max(over, Math.hypot(tr.north - i.holdN, tr.east - i.holdE)); });
      c.metric("time to stop / distance from hold point", `${ts.toFixed(2)} s / ${over.toFixed(2)} m`);
      c.ok(ts > 0 && over < 0.6, "brakes, then holds within 0.6 m");
    },
  },
  {
    name: "Speed modes ECO / NORMAL / SPORT",
    async run(c) {
      const want = { 17: [2, 15, "ECO"], 18: [5, 25, "NORMAL"], 19: [10, 35, "SPORT"] };
      for (const code of [17, 18, 19]) {
        const [vmax, tilt, nm] = want[code];
        let e = await airborne();
        e.command(code);
        e.run(0.5);
        e.setPilot({ pitch: -1 });
        e.run(code === 19 ? 8 : 6);
        const v = e.phys.truth().speed;
        c.metric(`${nm}: full-stick speed`, v, "m/s");
        c.ok(Math.abs(v - vmax) < 0.15 * vmax, `${nm}: ${vmax} m/s`);
        e = await airborne({ params: { "Pos hold": 0 } });
        e.command(code);
        e.run(0.5);
        e.setPilot({ roll: 1 });
        e.run(1.5);
        const r = e.phys.truth().roll;
        c.metric(`${nm}: full-stick tilt (angle mode)`, r, "deg");
        c.ok(Math.abs(r - tilt) < 2.5, `${nm}: ${tilt} deg`);
      }
      const e = await airborne();
      let jump = 0, last = null;
      e.command(17); e.run(1.5);
      e.command(19); e.run(1.5, (s) => { const i = s.info(); const t = Math.hypot(i.rollSp, i.pitchSp); if (last !== null) jump = Math.max(jump, Math.abs(t - last)); last = t; });
      c.metric("largest set-point step while switching modes in a hover", jump, "deg");
      c.ok(jump < 5 && e.info().speedMode === 2, "mode switch in flight without a jump");
    },
  },
  {
    name: "Compass error -30 .. +30 deg in wind, runaway guard at 60 deg",
    async run(c) {
      for (const d of [-30, -15, 0, 14.6, 30]) {
        const e = await airborne({ env: { magError: d, wind: [3, 0, 0] } });
        const errs = [];
        e.run(15, (s) => { const i = s.info(), tr = s.phys.truth(); errs.push(Math.hypot(tr.north - i.holdN, tr.east - i.holdE)); });
        const late = errs.slice(-500);
        c.metric(`compass error ${d} deg: max error (last 10 s)`, Math.max(...late), "m");
        c.ok(Math.max(...late) < 1.0 && e.info().posActive, `frame error ${d} deg: holds within 1 m`);
      }
      const e = await airborne({ env: { magError: 60, wind: [3, 0, 0] } });
      let maxErr = 0;
      e.run(20, (s) => { const i = s.info(), tr = s.phys.truth(); maxErr = Math.max(maxErr, Math.hypot(tr.north - i.holdN, tr.east - i.holdE)); });
      const i = e.info();
      c.metric("60 deg: max error / switched off", `${maxErr.toFixed(2)} m / ${i.posFailed}`);
      c.ok(maxErr < 2.5 || i.posFailed, "60 deg: still bounded, or the runaway guard switched position hold off");
      c.ok(e.info().state === 3 && e.phys.truth().tilt < 30, "drone stays upright and flying");
    },
  },
  {
    name: "Big compass error (-76 deg, 180 deg): learned on the first move, then holds",
    async run(c) {
      // -76 deg = what a Gazebo version that uses the world file's magnetic field would give; 180 = compass reversed
      for (const d of [-76, 180]) {
        const e = await airborne({ env: { magError: d } });
        e.setPilot({ pitch: -0.6 });
        e.run(3);
        e.setPilot({ pitch: 0 });
        e.run(8);
        let maxErr = 0;
        e.run(6, (s) => { const i = s.info(), tr = s.phys.truth(); maxErr = Math.max(maxErr, Math.hypot(tr.north - i.holdN, tr.east - i.holdE)); });
        const i = e.info();
        const learned = Math.abs(Math.abs(wrap180(i.yawOffset)) - Math.abs(wrap180(d)));
        c.metric(`compass error ${d} deg: learned offset / max error while holding`, `${i.yawOffset.toFixed(1)} deg / ${maxErr.toFixed(2)} m`);
        c.ok(learned < 5 && maxErr < 1.0 && i.posActive && e.info().state === 3, `compass error ${d} deg: learned, holds within 1 m`);
      }
    },
  },
  {
    name: "Far from home, then a full 360 deg turn while holding",
    async run(c) {
      const e = await airborne();
      e.setPilot({ pitch: -1 });
      e.run(10);
      e.setPilot({ pitch: 0 });
      e.run(6);
      const tr0 = e.phys.truth();
      c.metric("distance from home", Math.hypot(tr0.north, tr0.east), "m");
      const h0 = [e.info().holdN, e.info().holdE];
      let maxErr = 0;
      e.setPilot({ yaw: 0.5 });
      e.run(8, (s) => { const i = s.info(), tr = s.phys.truth(); maxErr = Math.max(maxErr, Math.hypot(tr.north - i.holdN, tr.east - i.holdE)); });
      e.setPilot({ yaw: 0 });
      e.run(5, (s) => { const i = s.info(), tr = s.phys.truth(); maxErr = Math.max(maxErr, Math.hypot(tr.north - i.holdN, tr.east - i.holdE)); });
      const i = e.info();
      const moved = Math.hypot(i.holdN - h0[0], i.holdE - h0[1]);
      c.metric("max error during the turn", maxErr, "m");
      c.metric("hold point moved", moved, "m");
      c.ok(Math.hypot(tr0.north, tr0.east) > 40 && maxErr < 1 && moved < 0.5, "50 m from home: holds within 1 m through a full turn");
    },
  },
  {
    name: "Take-off and landing in 5 m/s wind stay on the spot",
    async run(c) {
      const e = await engine({ env: { wind: [5, 0, 0] } });
      e.until((s) => s.info().state === 3, 15);
      const tr = e.phys.truth();
      c.metric("drift during take-off", Math.hypot(tr.north, tr.east), "m");
      c.ok(Math.hypot(tr.north, tr.east) < 1.0, "take-off drifts less than 1 m");
      e.run(3);
      const i0 = e.info();
      e.command(CODES.LAND);
      const t = e.until((s) => s.info().state === 0, 20);
      e.run(0.2);
      const t1 = e.phys.truth();
      const d = Math.hypot(t1.north - i0.holdN, t1.east - i0.holdE);
      c.metric("landing time / touch-down distance from the hold point", `${t.toFixed(1)} s / ${d.toFixed(2)} m`);
      c.ok(t > 0 && d < 0.7 && e.info().reason === 9 && t1.tilt < 10, "lands upright within 0.7 m and disarms");
    },
  },
  {
    name: "RC link lost: LAND lands on the spot, HOLD hovers, link back = control back",
    async run(c) {
      let e = await airborne({ env: { wind: [3, 0, 0] } });
      e.run(2);
      const p0 = e.phys.truth();
      e.rcCut = true;
      const tf = e.until((s) => s.info().state === 5, 5);
      c.metric("failsafe starts after", tf, "s");
      const t = e.until((s) => s.info().state === 0, 25);
      e.run(0.2);
      const p1 = e.phys.truth();
      const d = Math.hypot(p1.north - p0.north, p1.east - p0.east);
      c.metric("time to land / distance from where the link was lost", `${t.toFixed(1)} s / ${d.toFixed(2)} m`);
      c.ok(tf > 1.5 && tf < 3 && t > 0 && d < 1.0 && e.info().reason === 2, "LAND: lands within 1 m of the spot, disarms (reason pilot link)");
      e = await airborne({ env: { wind: [3, 0, 0] }, params: { "RC lost": 1 } });
      e.run(2);
      const h0 = e.phys.truth();
      e.rcCut = true;
      let maxErr = 0;
      e.run(10, (s) => { const tr = s.phys.truth(); maxErr = Math.max(maxErr, Math.hypot(tr.north - h0.north, tr.east - h0.east)); });
      c.metric("HOLD: max drift in 10 s / altitude", `${maxErr.toFixed(2)} m / ${e.phys.truth().alt.toFixed(2)} m`);
      c.ok(e.info().state === 5 && e.info().reason === 2 && maxErr < 0.6 && Math.abs(e.phys.truth().alt - 1.5) < 0.2, "HOLD: failsafe hover in place");
      e.rcCut = false;
      let jump = 0, last = null;
      e.run(2, (s) => { const i = s.info(); const tt = Math.hypot(i.rollSp, i.pitchSp); if (last !== null) jump = Math.max(jump, Math.abs(tt - last)); last = tt; });
      c.ok(e.info().state === 3 && jump < 5, "link restored: back to FLYING without a jump");
    },
  },
  {
    name: "GPS stuck / compass off / noisy, delayed GPS",
    async run(c) {
      let e = await airborne();
      e.setPilot({ pitch: -0.5 });
      e.run(2);
      e.phys.env.gpsFreeze = true;
      let jump = 0, last = null;
      const tOff = e.until((s) => { const i = s.info(); const tt = Math.hypot(i.rollSp, i.pitchSp); if (last !== null) jump = Math.max(jump, Math.abs(tt - last)); last = tt; return !i.posActive; }, 3);
      c.metric("GPS stuck while moving: falls back after", tOff, "s");
      c.ok(tOff > 0 && tOff < 1.5 && jump < 5, "falls back to angle flying within 1.5 s, no jump");
      e.setPilot({ pitch: 0 });
      e.phys.env.gpsFreeze = false;
      const tOn = e.until((s) => s.info().posActive, 6);
      c.ok(tOn > 0, "GPS back: position hold back on");
      e = await airborne({ params: { "Compass": 0 } });
      c.ok(!e.info().posReady && !e.info().posActive && e.info().state === 3, "compass OFF: position hold not ready, flies in angle mode");
      // Gazebo's GPS is clean; a real GPS is noisy (and late).  A GPS drone without a camera holds to about 1 m.
      for (const [env, lim, nm] of [[{ gpsNoise: 0.3 }, 1.0, "noisy GPS (0.3 m)"],
                                    [{ gpsNoise: 0.3, gpsDelay: 0.15 }, 2.5, "noisy (0.3 m) + delayed (150 ms) GPS"]]) {
        e = await airborne({ env });
        const errs = [];
        e.run(20, (s) => { const i = s.info(), tr = s.phys.truth(); errs.push(Math.hypot(tr.north - i.holdN, tr.east - i.holdE)); });
        c.metric(`${nm}: 95 % error`, pct(errs, 0.95), "m");
        c.ok(pct(errs, 0.95) < lim && e.info().posActive && !e.info().posFailed, `${nm}: holds within ${lim} m`);
      }
    },
  },
  {
    name: "Switching position hold / stick style in flight is smooth",
    async run(c) {
      const e = await airborne();
      let jump = 0, last = null;
      const watch = (s) => { const i = s.info(); const tt = Math.hypot(i.rollSp, i.pitchSp); if (last !== null) jump = Math.max(jump, Math.abs(tt - last)); last = tt; };
      e.phys.env.wind = [3, 0, 0];
      e.run(6);
      e.command(CODES.POSHOLD); e.run(2, watch);
      c.ok(!e.info().posActive, "code 20: position hold off");
      e.command(CODES.POSHOLD); e.run(3, watch);
      c.ok(e.info().posActive, "code 20 again: on");
      e.command(CODES.STICK); e.run(1, watch);
      c.ok(e.fc.get("Stick") === 1, "code 21: stick style ANGLE");
      c.metric("largest set-point step", jump, "deg");
      c.ok(jump < 5, "no jumps (< 5 deg)");
    },
  },
  {
    name: "Height limit ON stops at 20 m; OFF climbs to 1000 m (telemetry v2)",
    async run(c) {
      let e = await airborne({ params: { "Alt limit": 1, "Max alt": 20 } });
      e.command(CODES.SPORT);
      e.setPilot({ thr: 3.0 });
      let peak = 0;
      e.run(15, (s) => { peak = Math.max(peak, s.phys.truth().alt); });
      c.metric("limit 20 m: highest point", peak, "m");
      c.ok(peak < 20.3 && peak > 19.0, "stops at the 20 m ceiling");
      c.ok(e.bridge.status && e.bridge.status.atCeiling && e.bridge.status.maxAlt === 20, "telemetry: at ceiling, limit 20 m");
      e.setPilot({ thr: 2.5 });
      e.fc.set("Max alt", 10);
      e.run(12);
      c.metric("after lowering the limit to 10 m", e.phys.truth().alt, "m");
      c.ok(Math.abs(e.phys.truth().alt - 10) < 0.5, "lowering the limit brings it down");
      e = await airborne();
      e.command(CODES.SPORT);
      e.setPilot({ thr: 3.0 });
      const t = e.until((s) => s.phys.truth().alt > 1000, 320);
      e.setPilot({ thr: 2.5 });
      e.run(3);
      const s = e.bridge.status;
      c.metric("time to climb to 1000 m (SPORT, limit OFF)", t, "s");
      c.metric("telemetry altitude at 1 km", s ? s.alt : NaN, "m");
      c.ok(t > 0 && s && s.alt > 995 && Math.abs(s.alt - e.phys.truth().alt) < 1.5, "limit OFF: reaches 1000 m, telemetry shows it correctly");
    },
  },
  {
    name: "Gamepad: a resting stick holds, a real push moves, keys override, hidden page = neutral",
    async run(c) {
      const pad = { connected: true, axes: [0, 0, 0, 0], buttons: [], id: "test pad", mapping: "standard" };
      const pilot = new Pilot();
      pilot.getPads = () => [pad];
      const fly = (e, secs) => e.run(secs, (s) => { s.setPilot(pilot.update(0.02)); }, 0.02);
      let e = await airborne();
      // the stick that used to cause the drift: rests at 0.22 after flying
      pad.axes = [0, 0, 0.22, 0];
      fly(e, 60);
      let i = e.info(), tr = e.phys.truth();
      const flown = Math.hypot(tr.north, tr.east);
      c.metric("stick resting at 0.22: flown in 60 s", flown, "m");
      c.ok(pilot.output().roll === 0 && i.posState === 4 && flown < 0.3, "resting at 0.22: sends 0, HOLDING, stays within 0.3 m");
      // worse than the deadzone: it moves, but the page can warn about it
      pad.axes = [0, 0, 0.3, 0];
      fly(e, 5);
      c.metric("stick resting at 0.30: reported rest value", pilot.padRest);
      c.ok(pilot.padRest > PAD_DEADZONE, "a stick resting beyond the deadzone is reported (page warning)");
      // a real push still flies
      pad.axes = [0, 0, 0.6, 0];
      fly(e, 3);
      c.metric("stick at 0.6: speed", e.phys.truth().speed, "m/s");
      c.ok(e.phys.truth().speed > 1 && pilot.padRest === 0, "a real push moves the drone, and is not called a resting stick");
      // full stick is still full stick
      pad.axes = [0, 0, 1, 0];
      fly(e, 0.2);
      c.ok(Math.abs(pilot.output().roll - 1) < 1e-6, "full stick still gives full deflection");
      // a key overrides the pad on its axis, and the pad comes back when it is released
      pad.axes = [0, 0, 0.6, 0];
      pilot.keys.add("a");
      fly(e, 0.5);
      c.ok(Math.abs(pilot.output().roll + pilot.kbStrength) < 1e-6, "keyboard overrides the gamepad (A = left while the pad pushes right)");
      pilot.keys.delete("a");
      fly(e, 0.5);
      c.ok(pilot.output().roll > 0.3, "key released: the gamepad is back");
      // page hidden: nobody is flying
      pilot.keys.add("w");
      const out = pilot.update(0.02, true);
      c.ok(out.roll === 0 && out.pitch === 0 && pilot.keys.size === 0, "hidden page: sticks neutral, keys released");
      e.setPilot(out);
      const worst = letGoAndHold(e, 10);
      c.ok(worst < 0.3, "hidden page: brakes and holds");
    },
  },
  {
    name: "Long flight: 10 minutes of flying in wind and gusts, then it still holds its spot",
    async run(c) {
      for (const [nm, env, seed, mode] of [["NORMAL", { wind: [3, 0, 0], gust: 1 }, 21, CODES.NORMAL],
                                           ["SPORT", { wind: [3, 0, 0], gust: 1 }, 22, CODES.SPORT],
                                           ["NORMAL, sensor noise + gyro bias", { wind: [3, 0, 0], gyroNoise: 0.01, accNoise: 0.3, baroNoise: 0.1, magNoise: 1e-6, gyroBias: [0.02, -0.015, 0.01] }, 23, CODES.NORMAL]]) {
        const e = await airborne({ env });
        e.command(mode);
        e.run(0.5);
        const fly = randomPilot(seed);
        let maxOffset = 0, failed = false;
        for (let sec = 0; sec < 600; sec++) {
          fly(e, sec);
          e.run(1);
          const i = e.info();
          if (sec > 30) maxOffset = Math.max(maxOffset, Math.abs(i.yawOffset));
          failed = failed || i.posFailed;
        }
        const worst = letGoAndHold(e);
        c.metric(`${nm}: largest heading offset / hold after 10 min`, `${maxOffset.toFixed(1)} deg / ${worst.toFixed(2)} m`);
        c.ok(worst < 0.75 && maxOffset < 5 && !failed && e.info().state === 3, `${nm}: after 10 min holds within 0.75 m, heading offset below 5 deg, never FAILED`);
      }
    },
  },
  {
    name: "Long flight with 70 ms delay + jitter (no waiting, like a busy Gazebo bridge), then it still holds its spot",
    async run(c) {
      const e = await SimEngine.software({ env: { wind: [3, 0, 0] } });
      e.opts.upLatencyMs = 70;
      e.bridge.opts.burstJitter = 0.004;
      e.bridge.start();
      e.until((s) => { const i = s.info(); return i.state === 3 && Math.abs(i.alt - 1.5) < 0.15; }, 15);
      e.run(3);
      const fly = randomPilot(24);
      for (let sec = 0; sec < 300; sec++) { fly(e, sec); e.run(1); }
      const worst = letGoAndHold(e);
      c.metric("after 5 min with 70 ms delay: hold / heading offset", `${worst.toFixed(2)} m / ${e.info().yawOffset.toFixed(1)} deg`);
      c.ok(worst < 0.5 && e.info().state === 3 && !e.info().posFailed, "70 ms delay: holds within 0.5 m after 5 min of flying");
    },
  },
  {
    name: "Real board link: slow USB and a busy page don't shake the drone (the physics waits for the board)",
    async run(c) {
      const fly = async (o) => {
        const r = await boardRig({ env: { wind: [3, 0, 0] }, ...o });
        let up = false;
        for (let k = 0; k < 900 && !up; k++) { r.run(0.1); const i = r.info(); up = i.state === 3 && Math.abs(i.alt - 1.5) < 0.15; }
        if (!up) return { up, r, tilt: 180, hold: 99, speed: 0 };
        r.run(3);
        const sim0 = r.e.t, wall0 = r.wall();
        let tilt = 0, hold = 0;
        const watch = (secs) => { for (let k = 0; k < secs * 10; k++) { r.run(0.1); const i = r.info(), tr = r.e.phys.truth(); tilt = Math.max(tilt, tr.tilt); hold = Math.max(hold, Math.hypot(tr.north - i.holdN, tr.east - i.holdE)); } };
        watch(20);
        const speed = (r.e.t - sim0) / ((r.wall() - wall0) / 1000);
        r.e.setPilot({ pitch: -0.6 });
        r.run(6);
        r.e.setPilot({ pitch: 0 });
        r.run(12);
        watch(20);
        return { up, r, tilt, hold, speed };
      };
      // without waiting: the cliff, for the report
      const bad = await fly({ usbMs: 100, frameMs: 10, lockstep: 0 });
      c.metric("USB 100 ms, physics does not wait: largest tilt / hold", `${bad.tilt.toFixed(0)} deg / ${bad.hold.toFixed(2)} m`);
      for (const [u, f, minSpeed] of [[5, 10, 0.98], [5, 60, 0.98], [50, 30, 0.98], [100, 10, 0.3], [150, 30, 0.2], [300, 30, 0.1]]) {
        const k = await fly({ usbMs: u, frameMs: f });
        c.metric(`USB ${u} ms, drawing ${f} ms: tilt / hold / simulation speed`, `${k.tilt.toFixed(1)} deg / ${k.hold.toFixed(2)} m / ${k.speed.toFixed(2)}x`);
        c.ok(k.up && k.tilt < 5 && k.hold < 0.5 && k.speed > minSpeed, `USB ${u} ms, drawing ${f} ms: steady (tilt < 5 deg, within 0.5 m)`);
      }
      // the control delay the page shows is exact, also with several packets on their way
      const m = await boardRig({ usbMs: 50, frameMs: 1, lockstep: 0 });
      m.run(15);
      c.metric("USB 50 ms: measured control delay", m.e.bridge.controlDelayMs, "ms");
      c.ok(m.e.bridge.controlDelayMs > 40 && m.e.bridge.controlDelayMs < 80, "control delay is measured correctly");
      // board unplugged in flight: time must go on, or the simulator's failsafe would never fire
      const p = await boardRig({ usbMs: 5, frameMs: 10 });
      p.run(10);
      p.unplug();
      const t0 = p.e.t;
      p.run(10);
      c.metric("board unplugged: simulator time went on", p.e.t - t0, "s");
      c.ok(p.e.bridge.state === "FAILSAFE" && p.e.bridge.motors.every((v) => v === 0), "board unplugged: the simulator's failsafe stops the motors");
    },
  },
  {
    name: "Position hold recovers: FAILED tries again by itself; gives up after 3 in a row",
    async run(c) {
      // wind no drone can fight: position hold gives up
      let e = await airborne();
      e.phys.env.wind = [40, 0, 0];
      const tFail = e.until((s) => s.info().posFailed, 30);
      c.metric("wind 40 m/s: FAILED after", tFail, "s");
      c.ok(tFail > 0 && e.bridge.status !== null, "wind far beyond its limit trips the safety check");
      e.run(0.2);
      c.ok(e.info().posState === 5, "the state shows FAILED while it waits");
      // the wind stops: it must not slide away for ever
      e.phys.env.wind = [0, 0, 0];
      const tBack = e.until((s) => !s.info().posFailed, 7);
      c.metric("FAILED -> tries again after", tBack, "s");
      c.ok(tBack > 0, "tries again by itself (within 5 s of failing)");
      const worst = letGoAndHold(e);
      c.metric("then holds within", worst, "m");
      c.ok(worst < 0.5 && e.info().posActive, "brakes, then holds its new spot");
      // the wind stays: stop pushing after 3 failures in a row
      e = await airborne();
      e.phys.env.wind = [40, 0, 0];
      let fails = 0, last = false;
      e.run(120, (s) => { const f = s.info().posFailed; if (f && !last) fails++; last = f; }, 0.1);
      c.metric("wind stays for 2 min: failures", fails);
      c.ok(fails === 3 && e.info().posFailed, "gives up after 3 failures in a row");
      e.command(CODES.KILL);
      e.run(1);
      e.phys.env.wind = [0, 0, 0];
      e.run(10);
      e.command(CODES.TAKEOFF);
      const tUp = e.until((s) => { const i = s.info(); return i.state === 3 && Math.abs(i.alt - 1.5) < 0.1; }, 20);
      e.run(3);
      c.ok(tUp > 0 && e.info().posActive && !e.info().posFailed, "next arm: position hold is back");
    },
  },
  {
    name: "Heading offset stays right after hard flying (and telemetry reports it)",
    async run(c) {
      const hard = await airborne({ params: { "Stick": 1 } });
      hard.command(CODES.SPORT);
      hard.run(0.5);
      hard.setPilot({ pitch: -1 });
      hard.run(10);
      c.metric("speed after 10 s of full stick, ANGLE + SPORT", hard.phys.truth().speed, "m/s");
      hard.setPilot({ pitch: 0 });
      let worst = 0;
      hard.run(10, (s) => { worst = Math.max(worst, Math.abs(s.info().yawOffset)); }, 0.05);
      c.metric("ANGLE + SPORT, full stick then brake: worst heading offset", worst, "deg");
      c.ok(worst < 3, "hard brake from 25 m/s: heading offset stays within 3 deg (was -9.8)");
      for (const [nm, seed] of [["a", 3], ["b", 12]]) {
        const e = await airborne();
        e.command(CODES.SPORT);
        e.run(0.5);
        const fly = randomPilot(seed);
        let w = 0;
        for (let sec = 0; sec < 900; sec++) { fly(e, sec); e.run(1); w = Math.max(w, Math.abs(e.info().yawOffset)); }
        c.metric(`SPORT, 15 min of flying (${nm}): worst heading offset`, w, "deg");
        c.ok(w < 3, `SPORT flying (${nm}): stays within 3 deg (was up to 31)`);
        const st = e.bridge.status;
        c.ok(st && Math.abs(st.yawOffset - e.info().yawOffset) < 1, `telemetry reports the heading offset (${nm})`);
      }
    },
  },
  {
    name: "Gyro noise and drift switched on in flight (the simulator's sliders)",
    async run(c) {
      const D = Math.PI / 180;
      const cases = [
        ["noise 2 deg/s", { gyroNoise: 2 * D }, 1, 6],
        ["drift 1 deg/s", { gyroBias: [D, -0.7 * D, 0.4 * D] }, 1, 40],
        ["drift 3 deg/s", { gyroBias: [3 * D, -2.1 * D, 1.2 * D] }, 2.5, 150],
        ["noise 2 + drift 3", { gyroNoise: 2 * D, gyroBias: [3 * D, -2.1 * D, 1.2 * D] }, 2.5, 150],
      ];
      for (const [nm, on, limOn, limOff] of cases) {
        for (const hold of [1, 0]) {
          const e = await airborne({ params: { "Pos hold": hold } });
          const p0 = e.phys.truth();
          Object.assign(e.phys.env, on);
          let far = 0, tilt = 0;
          e.run(60, (s) => { const tr = s.phys.truth(); far = Math.max(far, Math.hypot(tr.north - p0.north, tr.east - p0.east)); tilt = Math.max(tilt, tr.tilt); }, 0.05);
          const i = e.info(), tr = e.phys.truth();
          const attErr = Math.hypot(i.roll - tr.roll, i.pitch - tr.pitch);
          const lab = `${nm}, position hold ${hold ? "ON" : "OFF"}`;
          c.metric(`${lab}: moved / tilt / attitude error after 60 s`, `${far.toFixed(1)} m / ${tilt.toFixed(1)} deg / ${attErr.toFixed(1)} deg`);
          c.ok(far < (hold ? limOn : limOff) && tilt < 12 && attErr < 1.5 && i.state === 3 && !i.posFailed, `${lab}: moved less than ${hold ? limOn : limOff} m, stays upright, filter recovers`);
        }
      }
    },
  },
  {
    name: "OLED help texts and motor readings",
    async run(c) {
      const e = await engine({ params: { "Auto TO": 0 } });   // disarmed on the bench (holding BACK while armed = KILL)
      e.run(2);
      const texts = e.fc.helpTexts();
      const bad = texts.filter((h) => !h.text || /[^\x20-\x7e]/.test(h.text) || h.text.split(" ").some((w) => w.length > 21) || h.text.length > 400);
      c.metric("help texts", texts.length);
      c.ok(texts.length > 80 && bad.length === 0, "every feature has a help text that fits the OLED" + (bad.length ? ": " + bad.map((b) => b.title).join(", ") : ""));
      // walk through every screen with the buttons: nothing may crash, every screen draws
      const B = (b, ev = 1) => e.fc.button(b, ev, e.nowUs());
      let frames = 0;
      const R = () => { e.run(0.13); if (e.fc.render(e.nowUs())) frames++; };
      for (let k = 0; k < 9; k++) { B(1); R(); }               // dashboard pages
      B(2); R();                                                  // menu
      for (let k = 0; k < 17; k++) { B(2); R(); B(3, 2); R(); B(3); R(); B(3); R(); B(1); R(); }   // open, help, back...
      c.ok(frames > 60 && e.info().state === 0, "all dashboard pages, menus and full-screen help pages draw");
      // while ARMED, holding BACK in a menu is KILL (safety), not help
      const e3 = await airborne();
      e3.fc.button(2, 1, e3.nowUs()); e3.run(0.1);              // open the menu
      e3.fc.button(3, 2, e3.nowUs()); e3.run(0.2);              // hold BACK
      c.ok(e3.info().state === 0 && e3.info().reason === 7, "armed: holding BACK in a menu = KILL");
      // motor values: what the OLED shows (= FC) equals what the simulator receives
      const e2 = await airborne({ params: { "Pos hold": 0 } });
      e2.setPilot({ roll: 0.5 });
      e2.run(0.08);                                         // while it starts rolling right
      const fc = e2.info().motors, br = e2.bridge.motors;
      c.metric("FC motors (OLED)", fc.map((m) => (m * 100).toFixed(1) + "%").join(" "));
      c.metric("sim received", br.map((m) => (m * 100).toFixed(1) + "%").join(" "));
      const diff = Math.max(...fc.map((m, k) => Math.abs(m - br[k])));
      c.ok(diff < 0.02, "OLED motor % = command received by the simulator (same order FR RL FL RR)");
      c.ok(fc[1] + fc[2] > fc[0] + fc[3], "rolling right: left motors (RL, FL) run faster - correct");
    },
  },
  {
    name: "Landing with the stick: slows near the ground, comes straight down, motors stop on touch-down",
    async run(c) {
      const report = (name, r) => {
        c.metric(name, `${r.time.toFixed(1)} s, touch ${r.touchSpeed.toFixed(2)} m/s, fastest ${r.fastest.toFixed(2)} m/s, sideways ${r.side.toFixed(2)} m, bounce ${r.bounce.toFixed(2)} m, lowest motor ${(r.minMotor * 100).toFixed(0)} %`);
      };
      const check = (name, e, r, side = 0.3) => {
        report(name, r);
        e.run(0.1);     // the last motor packet is still on its way when the flight controller disarms
        c.ok(r.time > 0 && e.info().reason === 9, `${name}: motors stop by themselves, reason = landed`);
        c.ok(e.bridge.motors.every((m) => m === 0), `${name}: motors at 0`);
        c.ok(r.touchSpeed < 0.5, `${name}: touches down below 0.5 m/s`);
        c.ok(r.side < side, `${name}: comes straight down (under ${side} m sideways)`);
        c.ok(r.bounce < 0.1, `${name}: no bounce`);
      };
      for (const [name, code] of [["ECO", 17], ["NORMAL", 18], ["SPORT", 19]]) {
        const e = await airborne();
        e.command(code);
        e.run(0.5);
        const r = stickLanding(e);
        check(`${name} from 1.5 m`, e, r);
        c.ok(r.time > 2.5, `${name} from 1.5 m: takes more than 2.5 s`);
        c.ok(r.minMotor > 0.5, `${name} from 1.5 m: the motors ease off instead of plunging`);
      }
      // from high up, fast
      const hi = await airborne();
      hi.command(19);
      hi.run(0.5);
      hi.setPilot({ thr: 3.0 });
      hi.until((s) => s.phys.truth().alt > 20, 15);
      hi.setPilot({ thr: 2.5 });
      hi.run(3);
      check("SPORT from 20 m", hi, stickLanding(hi));
      // half stick, wind, position hold off
      const half = await airborne();
      check("half stick", half, stickLanding(half, 2.25));
      const wind = await airborne({ env: { wind: [3, 0, 0], gust: 1.0 } });
      wind.run(8);
      check("wind 3 m/s + gusts", wind, stickLanding(wind), 0.6);
      const noHold = await airborne({ params: { "Pos hold": 0 } });
      check("position hold OFF", noHold, stickLanding(noHold));
      // stick released low down: it stays in the air
      const low = await airborne();
      low.setPilot({ thr: 2.0 });
      low.until((s) => s.phys.truth().alt < 0.6, 15);
      low.setPilot({ thr: 2.5 });
      low.run(5);
      const lowAlt = low.phys.truth().alt;
      c.metric("stick released at 0.6 m: height after 5 s", lowAlt, "m");
      c.ok(low.info().state === 3 && lowAlt > 0.35 && lowAlt < 0.7, "stick released near the ground: keeps flying at that height");
      // the T key flies again after a stick landing
      const again = await airborne();
      stickLanding(again);
      again.setPilot({ thr: 2.5 });
      again.run(1);
      again.command(CODES.TAKEOFF);
      c.ok(again.until((s) => s.info().state === 3, 12) > 0, "take-off works again after a stick landing");
    },
  },
  {
    name: "Left stick: pulling down with a wobbling thumb does not turn the drone",
    async run(c) {
      const pad = { connected: true, axes: [0, 0, 0, 0], buttons: [], id: "test pad", mapping: "standard" };
      const pilot = new Pilot();
      pilot.getPads = () => [pad];
      const read = (axes) => { pad.axes = axes; return pilot.update(0.02); };
      c.ok(read([0.4, 0.9, 0, 0]).yaw === 0 && pilot.output().thr < 2.2, "down + a small sideways part: descends, yaw stays 0");
      c.ok(read([-0.4, -0.9, 0, 0]).yaw === 0 && pilot.output().thr > 2.8, "up + a small sideways part: climbs, yaw stays 0");
      c.ok(Math.abs(read([0.9, 0.4, 0, 0]).thr - 2.5) < 1e-6 && pilot.output().yaw > 0.5, "turning + a small down part: turns, height stays");
      const both = read([0.8, 0.8, 0, 0]);
      c.ok(both.yaw > 0.3 && both.thr < 2.3, "a real diagonal still turns and descends together");
      const e = await airborne();
      const y0 = e.phys.truth().yaw;
      pad.axes = [0.4, 1, 0, 0];
      const a = e.phys.truth();
      e.run(10, (s) => { s.setPilot(pilot.update(0.02)); return s.info().state !== 0; }, 0.02);
      const tr = e.phys.truth();
      c.metric("descent with a wobbling thumb", `heading ${Math.abs(wrap180(tr.yaw - y0)).toFixed(1)} deg, sideways ${Math.hypot(tr.north - a.north, tr.east - a.east).toFixed(2)} m`);
      c.ok(Math.abs(wrap180(tr.yaw - y0)) < 3, "keeps its heading all the way down");
      c.ok(Math.hypot(tr.north - a.north, tr.east - a.east) < 0.3, "comes straight down");
    },
  },
  {
    name: "Organisers' keyboard tool: W goes forward, typed commands and our page follow their protocol",
    async run(c) {
      const noseAfter = (e, p) => { e.setPilot({ pitch: p }); e.run(1.5); return e.phys.truth().pitch; };
      // the same packets their tool sends: throttle 0, values kept until they change
      const tool = async (params = {}) => { const e = await airborne({ params: { "Pos hold": 0, ...params } }); e.pilotMode = "stock"; e.setPilot({ thr: 0, roll: 0, pitch: 0, yaw: 0 }); e.run(0.5); return e; };
      let e = await tool();
      noseAfter(e, 0.1); noseAfter(e, 0.2);
      let a = noseAfter(e, 0.3);
      c.metric("W pressed 3 times: pitch", a, "deg");
      c.ok(a < -2, "W steps (+0.1 each): nose down, forward");
      noseAfter(e, 0);
      noseAfter(e, -0.1);
      a = noseAfter(e, -0.2);
      c.metric("S pressed 2 times: pitch", a, "deg");
      c.ok(a > 1.5, "S steps (-0.1 each): nose up, back");
      noseAfter(e, 0);
      a = noseAfter(e, -0.2);
      c.metric("--pitch -0.2: pitch", a, "deg");
      c.ok(a < -1.5, "--pitch -0.2 after using the keys: nose down, as their README says");
      noseAfter(e, 0);
      a = noseAfter(e, 0.2);
      c.ok(a > 1.5, "--pitch 0.2: nose up, as their protocol says");
      noseAfter(e, 0);
      noseAfter(e, 0.1);
      a = noseAfter(e, 0.2);
      c.ok(a < -1.5, "back to the keys: W goes forward again");
      e = await tool({ "Inv pitch": 1 });
      noseAfter(e, 0.1);
      a = noseAfter(e, 0.2);
      c.ok(a > 1.5, "Inv pitch still reverses it, if the organisers want that");
      // a joystick that follows their protocol never lands on steps of exactly 0.1
      e = await tool();
      for (let p = 0; p > -0.37; p -= 0.037) { e.setPilot({ pitch: p }); e.run(0.04); }
      e.run(1.2);
      c.ok(e.phys.truth().pitch < -2, "smooth joystick values follow the protocol: negative = forward");
      // our own page's stream is not touched
      e = await airborne({ params: { "Pos hold": 0 } });
      e.setPilot({ thr: 2.5, pitch: 0.1 });
      e.run(0.5);
      e.setPilot({ pitch: 0.2 });
      e.run(1.5);
      c.ok(e.phys.truth().pitch > 1.5, "our page's sticks: never flipped");
    },
  },
  {
    name: "Landing and taking off again with only the organisers' tool (u, j, SPACE, codes 12 and 13)",
    async run(c) {
      const tool = async () => { const e = await airborne(); e.pilotMode = "stock"; e.setPilot({ thr: 0, roll: 0, pitch: 0, yaw: 0 }); e.run(0.5); return e; };
      let e = await tool();
      e.setPilot({ thr: 0.60 });           // j pressed 3 times
      let t = e.until((s) => s.info().state === 0, 20);
      c.metric("j to the ground: time until the motors stop", t, "s");
      c.ok(t > 0 && e.info().reason === 9, "manual throttle below hover: touches down, motors stop, reason = landed");
      e.run(0.2);
      c.ok(e.bridge.motors.every((m) => m === 0), "motors at 0");
      e.setPilot({ thr: 0 });              // SPACE
      e.run(1);
      c.ok(e.info().state === 0, "SPACE does not start the motors");
      e.setPilot({ thr: 0.68 });           // u
      c.ok(e.until((s) => s.phys.truth().alt > 1.0, 15) > 0, "u after SPACE: flies again");
      e.setPilot({ thr: 0 });
      e.run(3);
      c.ok(e.info().state === 3 && e.info().mode === 0, "SPACE: holds height again");
      // u straight after landing, without SPACE
      e = await tool();
      e.setPilot({ thr: 0.60 });
      e.until((s) => s.info().state === 0, 20);
      e.run(1);
      e.setPilot({ thr: 0.62 });           // u
      c.ok(e.until((s) => s.phys.truth().alt > 1.0, 15) > 0, "u straight after landing: takes off");
      // hovering by hand must not count as landed, and KILL must not restart
      e = await tool();
      e.setPilot({ thr: 0.66 });
      e.run(6);
      c.ok(e.info().state === 3, "manual throttle near hover, in the air: keeps flying");
      e.command(CODES.KILL);
      e.run(1);
      e.setPilot({ thr: 0.70 });
      e.run(2);
      c.ok(e.info().state === 0, "after KILL, throttle does not restart the motors");
      // the command codes through their --throttle option
      e = await tool();
      e.setPilot({ thr: 13 });
      e.run(0.1);
      e.setPilot({ thr: 0 });
      t = e.until((s) => s.info().state === 0, 20);
      c.ok(t > 0 && e.info().reason === 9, "--throttle 13: lands and stops");
      e.run(1);
      e.setPilot({ thr: 12 });
      e.run(0.1);
      e.setPilot({ thr: 0 });
      c.ok(e.until((s) => s.phys.truth().alt > 1.0, 15) > 0, "--throttle 12: takes off again");
    },
  },
  {
    name: "A bridge too slow to fly on: stays on the ground and says so; PLOTS page draws",
    async run(c) {
      // the organisers' bridge without Gazebo's Python bindings: about one burst of sensors a second
      for (const period of [1.0, 1.6]) {       // 1.6: some one-second windows hold no packet at all
        const e = await engine({ bridge: { burstPeriod: period } });
        let moved = false;
        e.run(70, (s) => { if (s.info().state !== 0) moved = true; }, 0.05);
        c.metric(`one sensor burst every ${period} s: height after 70 s`, e.phys.truth().alt, "m");
        c.ok(!moved && e.phys.truth().alt < 0.2, `one burst every ${period} s: never arms, can't fly away`);
        c.ok(e.bridge.motors.every((m) => m === 0), `one burst every ${period} s: motors stay at 0`);
      }
      // 25 bursts a second is slow, but flies
      const ok = await airborne({ bridge: { burstPeriod: 0.04 } });
      c.ok(ok.info().state === 3, "25 sensor bursts per second: flies as usual");
      // the live plots: UP from the first page wraps to PLOTS
      const p = await airborne();
      p.fc.button(1, 1, p.nowUs());
      let fb = null;
      const fly = (secs) => { for (let t = 0; t < secs; t += 0.04) { p.run(0.04); const f = p.fc.render(p.nowUs()); if (f) fb = f.slice(0); } };   // as often as the board draws
      p.setPilot({ thr: 2.5, roll: 0.8 });
      fly(3);
      p.setPilot({ roll: 0 });
      fly(2);
      c.ok(!!fb, "the PLOTS page draws");
      if (fb) {
        const lit = (y0, y1) => { let n = 0; for (let y = y0; y <= y1; y++) for (let x = 12; x < 128; x++) if (fb[(y >> 3) * 128 + x] >> (y & 7) & 1) n++; return n; };
        const litAt = (y) => { let n = 0; for (let x = 12; x < 128; x++) if (fb[(y >> 3) * 128 + x] >> (y & 7) & 1) n++; return n; };
        c.metric("lit pixels per band RP / YT / AT / AL", `${lit(9, 20)} / ${lit(22, 33)} / ${lit(35, 46)} / ${lit(48, 63)}`);
        c.ok(lit(9, 20) > 80 && lit(22, 33) > 80 && lit(35, 46) > 80 && lit(48, 63) > 80, "all four graphs have a trace");
        c.ok(litAt(9) + litAt(10) + litAt(11) > 15, "the roll stick push shows in the top band");
      }
    },
  },
  {
    name: "Page says if commands really reach the flight controller; launchers open it in Gazebo mode",
    async run(c) {
      const base = { mode: "gazebo", bridgeSeen: true, relay: "http://localhost:8770", connectedFor: 100, postStatus: 200, telemetryAge: 0.1, pilotHz: 25, rcCut: false, hidden: false, focused: true, pad: false };
      const say = (o) => linkStatus({ ...base, ...o });
      c.ok(say({}).level === "ok" && say({}).text.includes("25 per second"), "all good: green, with the rate the flight controller counts");
      // the mistake that hid the problem: Gazebo running, page flying its own copy
      c.ok(say({ mode: "software" }).level === "bad" && say({ mode: "software" }).action === "gazebo" && say({ mode: "software" }).text.includes("built-in copy"), "page flying its own copy while Gazebo runs: red, with a button");
      c.ok(say({ mode: "none" }).action === "gazebo", "page not connected while Gazebo runs: says so, with a button");
      c.ok(say({ mode: "software", bridgeSeen: false }) === null, "no Gazebo running: no banner");
      c.ok(say({ postStatus: 0, connectedFor: 5 }).level === "wait", "Gazebo still starting: waits");
      c.ok(say({ postStatus: 0 }).level === "bad" && say({ postStatus: 0 }).text.includes("8770"), "bridge gone: red, names the address");
      c.ok(say({ postStatus: 409 }).text.includes("launcher 2"), "board not connected: points to launcher 2");
      c.ok(say({ telemetryAge: 5 }).level === "bad", "no telemetry: red");
      c.ok(say({ pilotHz: 50 }).text.includes("Two pilots"), "a second page: says so");
      c.ok(say({ pilotHz: 0 }).level === "wait", "commands not counted yet: waits");
      c.ok(say({ focused: false }).text.includes("Click on this page"), "keyboard elsewhere: asks for a click");
      c.ok(say({ focused: false, pad: true }).level === "ok", "gamepad: no click needed");
      c.ok(say({ rcCut: true }).text.includes("cut the RC link"), "RC link test: says so");
      // the launchers must not hand "?mode=gazebo" to a batch file: it splits at "=" and the page loses its mode
      for (const f of ["3_FLY_ARDUINO_in_Gazebo", "4_FLY_SOFTWARE_FC_in_Gazebo"]) {
        const bat = await (await fetch(`../gazebo/${f}.bat`)).text();
        c.ok(bat.includes('START_SIMULATOR.bat" gazebo') && !bat.includes("?mode="), `${f}: opens the page in Gazebo mode`);
      }
      const page = await (await fetch("index.html")).text();
      c.ok(page.includes('id="relayUrl" value="http://127.0.0.1:8770"'), "the page talks to the bridge on 127.0.0.1: \"localhost\" adds 320 ms to every command");
      const start = await (await fetch("../START_SIMULATOR.bat")).text();
      c.ok(start.includes('"%~1"=="gazebo"') && start.includes("?mode=gazebo"), "START_SIMULATOR turns the word gazebo into the page's mode");
      // the throttle card, from real telemetry
      const e = await airborne();
      let s = null;
      e.bridge.onStatus = (v) => { s = v; };
      e.run(1);
      let v = throttleView(s);
      c.metric("hover: stick / power", `${v.stickText} / ${v.powerText}`);
      c.ok(v.stick === 0 && v.stickText.startsWith("centre") && Math.abs(v.power) <= 2, "hovering: stick centred, power about 0");
      e.setPilot({ thr: 3.0 });
      e.run(0.3);
      v = throttleView(s);
      c.metric("climb starting: stick / power", `${v.stickText} / ${v.powerText}`);
      c.ok(v.stickText === "up 100 %" && v.power >= 5, "starting to climb: stick up, power above hover");
      e.setPilot({ thr: 2.5 });
      e.run(4);
      e.setPilot({ thr: 2.0 });
      e.run(0.5);
      v = throttleView(s);
      c.metric("descent starting: stick / power", `${v.stickText} / ${v.powerText}`);
      c.ok(v.stickText === "down 100 %" && v.power <= -1, "starting to descend: stick down, power below hover");
      c.ok(throttleView({ ...s, rcThr: 0.72 }).stickText === "manual 72 %", "manual throttle is shown as manual");
      c.ok(throttleView({ ...s, rcThr: 0 }).stickText === "no stick received", "no pilot packet: says so");
      c.ok(s.pilotHz >= 20, "telemetry counts the pilot commands");
      e.rcCut = true;
      e.run(1.5);
      c.ok(s.pilotHz === 0, "commands stop: the count goes to 0 instead of keeping the old value");
    },
  },
];

export async function runAll(filter = null, onResult = null) {
  const results = [];
  for (const s of SCENARIOS) {
    if (filter && !s.name.toLowerCase().includes(filter.toLowerCase())) continue;
    const c = new Check();
    const t0 = performance.now();
    try { await s.run(c); } catch (err) { c.ok(false, "EXCEPTION: " + (err && err.stack ? err.stack : err)); }
    const r = { name: s.name, pass: c.pass, items: c.items, metrics: c.metrics, ms: performance.now() - t0 };
    results.push(r);
    if (onResult) onResult(r);
  }
  return results;
}
