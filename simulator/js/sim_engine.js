// sim_engine.js - physics (1 ms steps) + bridge replica + flight controller (software wasm FC or a real board).
import { QuadX3 } from "./physics_x3.js";
import { BridgeReplica } from "./bridge_replica.js";
import { WasmFc } from "./fc_wasm.js";

export const CODES = { ARM: 10, DISARM: 11, TAKEOFF: 12, LAND: 13, KILL: 16 };
const DT = 0.001;

export class SimEngine {
  constructor(opts = {}) {
    this.opts = { upLatencyMs: 2, downLatencyMs: 1, pilotRateHz: 25, env: {}, bridge: {}, ...opts };
    this.phys = new QuadX3(this.opts.env);
    this.t = 0;
    this.toFc = [];
    this.toBridge = [];
    this.fc = null;            // WasmFc when the software FC is used
    this.serial = null;        // real board: object with write(bytes)
    this.bridge = new BridgeReplica(this.phys, (b) => this.bridgeSend(b), () => this.t, this.opts.bridge);
    // pilot stream
    this.pilot = { thr: 2.5, roll: 0, pitch: 0, yaw: 0 };
    this.pilotMode = "extended";   // "extended" = our 25 Hz stream, "stock" = like control_input.py (on change only), "off"
    this.pilotCodes = [];
    this.nextPilot = 0;
    this.lastStockSent = null;
    this.stepHooks = [];
  }

  static async software(opts = {}) {
    const e = new SimEngine(opts);
    e.fc = await WasmFc.create(opts.storageKey || null);
    return e;
  }

  attachSerial(writer) { this.serial = writer; this.fc = null; }
  nowUs() { return Math.round(this.t * 1e6) >>> 0; }

  bridgeSend(bytes) {
    if (this.serial) this.serial.write(bytes);
    else this.toFc.push({ t: this.t + this.opts.downLatencyMs / 1000, b: bytes });
  }
  // bytes from a real board
  serialReceived(bytes) { this.bridge.receive(bytes); }

  setPilot(p) { Object.assign(this.pilot, p); }
  command(code) { for (let i = 0; i < 3; i++) this.pilotCodes.push(code); }

  servicePilot() {
    if (this.pilotMode === "off" || !this.bridge.running) return;
    if (this.t < this.nextPilot) return;
    this.nextPilot = this.t + 1 / this.opts.pilotRateHz;
    const p = this.pilot;
    if (this.pilotCodes.length) {
      this.bridge.sendControl(this.pilotCodes.shift(), p.roll, p.pitch, p.yaw);
      return;
    }
    if (this.pilotMode === "stock") {
      const key = [p.thr, p.roll, p.pitch, p.yaw].join(",");
      if (key === this.lastStockSent) return;
      this.lastStockSent = key;
    }
    this.bridge.sendControl(p.thr, p.roll, p.pitch, p.yaw);
  }

  step() {
    this.phys.step(DT);
    this.t += DT;
    this.bridge.update();
    this.servicePilot();
    if (this.fc) {
      const now = this.nowUs();
      while (this.toFc.length && this.toFc[0].t <= this.t) this.fc.feed(this.toFc.shift().b, now);
      this.fc.tick(now);
      const out = this.fc.poll(now);
      if (out) this.toBridge.push({ t: this.t + this.opts.upLatencyMs / 1000, b: out });
      while (this.toBridge.length && this.toBridge[0].t <= this.t) this.bridge.receive(this.toBridge.shift().b);
    }
    for (const h of this.stepHooks) h(this);
  }

  run(seconds, everyFn = null, everyS = 0.02) {
    const end = this.t + seconds;
    let next = this.t;
    while (this.t < end - 1e-9) {
      this.step();
      if (everyFn && this.t >= next) { next = this.t + everyS; if (everyFn(this) === false) break; }
    }
  }

  // run until cond() is true or timeout; returns the time it took (or -1)
  until(cond, timeout, everyFn = null) {
    const start = this.t;
    while (this.t - start < timeout) {
      this.step();
      if (everyFn) everyFn(this);
      if (cond(this)) return this.t - start;
    }
    return -1;
  }

  info() {
    if (!this.fc) return null;
    const a = this.fc.info();
    const d = 180 / Math.PI;
    let yaw = a[2] * d; if (yaw < 0) yaw += 360;
    let ht = a[9] * d; if (ht < 0) ht += 360;
    return {
      roll: a[0] * d, pitch: a[1] * d, yaw, alt: a[3], vz: a[4], altTarget: a[5],
      rollSp: a[6] * d, pitchSp: a[7] * d, yawRateSp: a[8] * d, headingTarget: ht,
      state: a[10], mode: a[11], reason: a[12], airborne: a[13] > 0.5, collective: a[14],
      torque: [a[15], a[16], a[17]], motors: [a[18], a[19], a[20], a[21]], bias: [a[22], a[23], a[24]],
      magHealthy: a[25] > 0.5, magFused: a[26], ekfInit: a[27] > 0.5, hoverIntegral: a[28], vzSp: a[29],
      session: a[30], imuHz: a[31], rateSp: [a[32] * d, a[33] * d, a[34] * d], armResult: a[35],
      crashed: a[36] > 0.5, streamSeen: a[37] > 0.5, linkLost: a[38] > 0.5, crcErrors: a[39], tilt: a[40],
      txActuators: a[41], txHeartbeats: a[42], rxImu: a[43], north: a[44], east: a[45],
      accRejects: a[46], magRejects: a[47], sensorsOk: a[48] > 0.5, geofence: a[49] > 0.5,
    };
  }
}
