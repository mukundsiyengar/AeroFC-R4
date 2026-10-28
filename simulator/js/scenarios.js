// scenarios.js - automated closed-loop tests: the real firmware (wasm) flies the X3 replica faster than real time.
import { SimEngine, CODES } from "./sim_engine.js";
import { crc32, buildPacket, Parser, T } from "./protocol.js";

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
      const e = await airborne();
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
      const e = await airborne();
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
      c.ok(aHold - a0 > 1.4 && aHold - a0 < 2.6, "climbs ~1 m/s");
      e.run(3);
      c.metric("drift 3 s after release", e.phys.truth().alt - aHold, "m");
      c.ok(Math.abs(e.phys.truth().alt - aHold) < 0.15, "holds the new altitude");
      e.setPilot({ thr: 2.0 }); e.run(2); e.setPilot({ thr: 2.5 }); e.run(3);
      c.metric("after descend", e.phys.truth().alt, "m");
      c.ok(e.phys.truth().alt < aHold - 0.8, "descends with the stick");
    },
  },
  {
    name: "Stock control_input.py semantics (throttle 0 = hold, 0.66 = manual, sticky +-0.1)",
    async run(c) {
      const e = await airborne();
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
        const e = await airborne(k.opts);
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
      B(2); B(1); B(1); B(1); B(2); B(2); B(0); B(0); B(2);
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
