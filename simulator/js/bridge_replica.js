// bridge_replica.js - behaves like the official arduino_hil/hil_bridge_v1.py, but uses our X3 physics instead of Gazebo.
//
//  * opens the link, sends HELLO "UAV_HIL_V1", waits up to 3 s for HELLO_ACK (ignoring anything else), sends START
//  * every 20 ms sends IMU, BARO, MAG, GPS (the very first bursts contain zeros, like the real bridge
//    before its Gazebo subscribers deliver data)
//  * ACTUATOR_COMMAND (only while running): clamp 0..1, x1000 rad/s -> rotors 0..3
//  * HELLO from the FC during the session -> HELLO_ACK + START
//  * failsafe: no packet at all from the FC for 1.5 s -> motors 0, session stops (until the FC says HELLO)
//  * CONTROL_INPUT forwarded unchanged
//  * STATUS packets are decoded for our plots (the official bridge ignores them)
import { T, Parser, buildPacket, packImu, packBaro, packMag, packGps, packControl, unpackF32, decodeStatus, HELLO_TEXT } from "./protocol.js";

export class BridgeReplica {
  // send(bytes) delivers bytes to the flight controller; now() returns simulation time in seconds
  constructor(physics, send, now, opts = {}) {
    this.phys = physics;
    this.sendRaw = send;
    this.now = now;
    this.opts = { burstPeriod: 0.02, burstJitter: 0, zeroBursts: 2, failsafeTimeout: 1.5, helloTimeout: 3.0, ...opts };
    this.parser = new Parser();
    this.seq = 0;
    this.state = "IDLE";            // IDLE -> WAIT_ACK -> RUNNING | ERROR | FAILSAFE
    this.running = false;
    this.log = [];
    this.lastActivity = 0;
    this.nextBurst = 0;
    this.bursts = 0;
    this.motors = [0, 0, 0, 0];
    this.status = null;             // last decoded STATUS
    this.statusTime = -1;
    this.counts = { actuator: 0, heartbeat: 0, status: 0, hello: 0, other: 0, sensorBursts: 0 };
    this.lastHeartbeat = -1;
    this.heartbeatGapMax = 0;
    this.onStatus = null;
    this.onLog = null;
  }

  note(msg) {
    const line = `[${this.now().toFixed(2)}s] ${msg}`;
    this.log.push(line);
    if (this.log.length > 200) this.log.shift();
    if (this.onLog) this.onLog(line);
  }

  send(type, payload) {
    this.seq = (this.seq + 1) >>> 0;
    const ns = BigInt(Math.round(this.now() * 1e9));
    this.sendRaw(buildPacket(type, this.seq, ns, payload));
  }

  start() {
    this.note("Sending HELLO to Arduino...");
    this.send(T.HELLO, HELLO_TEXT);
    this.state = "WAIT_ACK";
    this.helloAt = this.now();
  }

  sendControl(thr, roll, pitch, yaw) { this.send(T.CONTROL_INPUT, packControl(thr, roll, pitch, yaw)); }
  sendStop() { this.send(T.STOP, new Uint8Array(0)); }

  zeroMotors() {
    this.motors = [0, 0, 0, 0];
    this.phys.setRotorVelocities([0, 0, 0, 0]);
  }

  // bytes coming back from the flight controller
  receive(bytes) {
    for (const pk of this.parser.push(bytes)) {
      if (this.state === "WAIT_ACK") {        // wait_for_hello_ack(): only HELLO_ACK matters
        if (pk.type === T.HELLO_ACK) {
          this.note("HELLO_ACK received from Arduino");
          this.note("Sending START to Arduino...");
          this.send(T.START, new Uint8Array(0));
          this.state = "RUNNING";
          this.running = true;
          this.lastActivity = this.now();
          this.nextBurst = this.now();
          this.note("HIL session started successfully");
        }
        continue;
      }
      if (this.state === "IDLE" || this.state === "ERROR") continue;
      this.lastActivity = this.now();
      switch (pk.type) {
        case T.HEARTBEAT: {
          const t = this.now();
          if (this.lastHeartbeat >= 0 && this.running) this.heartbeatGapMax = Math.max(this.heartbeatGapMax, t - this.lastHeartbeat);
          this.lastHeartbeat = t;
          this.counts.heartbeat++;
          break;
        }
        case T.HELLO:
          this.counts.hello++;
          this.note("Arduino sent HELLO during session, acknowledging...");
          this.send(T.HELLO_ACK, HELLO_TEXT);
          this.send(T.START, new Uint8Array(0));
          this.running = true;
          this.state = "RUNNING";
          break;
        case T.ACTUATOR_COMMAND: {
          if (!this.running || pk.payload.length < 16) break;
          const m = unpackF32(pk.payload, 4).map((v) => (Number.isFinite(v) ? Math.min(1, Math.max(0, v)) : 0));
          this.motors = m;
          this.counts.actuator++;
          this.phys.setRotorVelocities(m.map((v) => v * 1000.0));
          break;
        }
        case T.STATUS: {
          const s = decodeStatus(pk.payload);
          if (s) {
            this.status = s;
            this.statusTime = this.now();
            this.counts.status++;
            if (this.onStatus) this.onStatus(s);
          }
          break;
        }
        default:
          this.counts.other++;
      }
    }
  }

  // call every physics step
  update() {
    const t = this.now();
    if (this.state === "WAIT_ACK" && t - this.helloAt > this.opts.helloTimeout) {
      this.note("ERROR: HELLO_ACK timeout");
      this.state = "ERROR";
      return;
    }
    if (this.running && t - this.lastActivity > this.opts.failsafeTimeout) {
      this.running = false;
      this.state = "FAILSAFE";
      this.zeroMotors();
      this.note(`FAILSAFE: Arduino activity timeout (gap=${(t - this.lastActivity).toFixed(2)}s) - motors stopped`);
    }
    if (this.running && t >= this.nextBurst) {
      const j = this.opts.burstJitter ? (Math.random() - 0.5) * 2 * this.opts.burstJitter : 0;
      this.nextBurst = t + this.opts.burstPeriod + j;
      const zero = this.bursts < this.opts.zeroBursts;
      this.bursts++;
      this.counts.sensorBursts++;
      const s = this.phys.sens;
      const imu = zero ? [0, 0, 0, 0, 0, 0] : s.imu;
      this.send(T.SENSOR_IMU, packImu(...imu));
      this.send(T.SENSOR_BARO, packBaro(zero ? 0 : s.alt));
      this.send(T.SENSOR_MAG, packMag(...(zero ? [0, 0, 0] : s.mag)));
      this.send(T.SENSOR_GPS, packGps(...(zero ? [15.8497, 74.4977, 0] : s.gps)));
    }
  }
}
