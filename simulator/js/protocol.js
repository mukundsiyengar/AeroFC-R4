// protocol.js - UAV HIL protocol v1, byte-for-byte the same as uav-hil-simulator arduino_hil/hil_protocol.py
//   "HI" | ver u8 | type u8 | len u16 | seq u32 | timestamp u64 (ns) | payload | crc32 (zlib) - little endian

export const T = {
  SENSOR_IMU: 0x01, SENSOR_BARO: 0x02, SENSOR_MAG: 0x03, SENSOR_GPS: 0x04,
  ACTUATOR_COMMAND: 0x10, CONTROL_INPUT: 0x11,
  HELLO: 0x20, HELLO_ACK: 0x21, START: 0x22, STOP: 0x23, HEARTBEAT: 0x24, STATUS: 0x25, ERROR: 0x26,
};
export const HEADER_SIZE = 18;
export const HELLO_TEXT = new TextEncoder().encode("UAV_HIL_V1");

const CRC_TABLE = (() => {
  const t = new Uint32Array(256);
  for (let n = 0; n < 256; n++) {
    let c = n;
    for (let k = 0; k < 8; k++) c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1;
    t[n] = c >>> 0;
  }
  return t;
})();

export function crc32(bytes, start = 0, end = bytes.length) {
  let c = 0xffffffff;
  for (let i = start; i < end; i++) c = CRC_TABLE[(c ^ bytes[i]) & 0xff] ^ (c >>> 8);
  return (c ^ 0xffffffff) >>> 0;
}

export function buildPacket(type, seq, tsNs, payload = new Uint8Array(0)) {
  const len = payload.length;
  const out = new Uint8Array(HEADER_SIZE + len + 4);
  const dv = new DataView(out.buffer);
  out[0] = 0x48; out[1] = 0x49; out[2] = 1; out[3] = type;
  dv.setUint16(4, len, true);
  dv.setUint32(6, seq >>> 0, true);
  dv.setBigUint64(10, BigInt.asUintN(64, BigInt(tsNs)), true);
  out.set(payload, HEADER_SIZE);
  dv.setUint32(HEADER_SIZE + len, crc32(out, 0, HEADER_SIZE + len), true);
  return out;
}

// Same framing strategy as the official bridge: find "HI", wait for the whole packet, check CRC,
// drop one byte and rescan on any error.
export class Parser {
  constructor(maxPayload = 4096) {
    this.buf = new Uint8Array(8192);
    this.n = 0;
    this.maxPayload = maxPayload;
    this.crcErrors = 0;
    this.good = 0;
  }
  push(bytes) {
    if (this.n + bytes.length > this.buf.length) {
      const nb = new Uint8Array(Math.max(this.buf.length * 2, this.n + bytes.length));
      nb.set(this.buf.subarray(0, this.n));
      this.buf = nb;
    }
    this.buf.set(bytes, this.n);
    this.n += bytes.length;
    const out = [];
    let i = 0;
    const b = this.buf;
    while (this.n - i >= 2) {
      if (b[i] !== 0x48 || b[i + 1] !== 0x49) { i++; continue; }
      if (this.n - i < HEADER_SIZE) break;
      const len = b[i + 4] | (b[i + 5] << 8);
      if (b[i + 2] !== 1 || len > this.maxPayload) { i++; continue; }
      const total = HEADER_SIZE + len + 4;
      if (this.n - i < total) break;
      const dv = new DataView(b.buffer, b.byteOffset + i, total);
      const rx = dv.getUint32(HEADER_SIZE + len, true);
      if (crc32(b, i, i + HEADER_SIZE + len) !== rx) { this.crcErrors++; i++; continue; }
      out.push({
        type: b[i + 3],
        seq: dv.getUint32(6, true),
        ts: dv.getBigUint64(10, true),
        payload: b.slice(i + HEADER_SIZE, i + HEADER_SIZE + len),
      });
      this.good++;
      i += total;
    }
    if (i > 0) { b.copyWithin(0, i, this.n); this.n -= i; }
    return out;
  }
}

const f32 = (vals) => {
  const a = new Uint8Array(4 * vals.length);
  const dv = new DataView(a.buffer);
  vals.forEach((v, k) => dv.setFloat32(4 * k, v, true));
  return a;
};
export const packImu = (ax, ay, az, gx, gy, gz) => f32([ax, ay, az, gx, gy, gz]);
export const packBaro = (alt) => f32([alt]);
export const packMag = (mx, my, mz) => f32([mx, my, mz]);
export const packControl = (thr, roll, pitch, yaw) => f32([thr, roll, pitch, yaw]);
export function packGps(lat, lon, alt) {
  const a = new Uint8Array(20);
  const dv = new DataView(a.buffer);
  dv.setFloat64(0, lat, true);
  dv.setFloat64(8, lon, true);
  dv.setFloat32(16, alt, true);
  return a;
}
export function unpackF32(payload, count) {
  const dv = new DataView(payload.buffer, payload.byteOffset, payload.byteLength);
  const r = [];
  for (let k = 0; k < count; k++) r.push(dv.getFloat32(4 * k, true));
  return r;
}

export const STATES = ["DISARMED", "IDLE", "TAKEOFF", "FLYING", "LANDING", "FAILSAFE"];
export const REASONS = ["-", "user", "pilot link", "sensors", "estimator", "CRASH", "STOP", "KILL", "idle", "landed", "geofence"];

export const SPEED_MODES = ["ECO", "NORMAL", "SPORT"];
export const POS_STATES = ["OFF", "NOT READY", "MOVING", "BRAKING", "HOLDING", "FAILED"];

// STATUS payload: v2 (64 bytes, firmware 2.x) and v1 (48 bytes, firmware 1.x) - see src/core/telemetry.h
export function decodeStatus(p) {
  if (!p || p.length < 48) return null;
  const dv = new DataView(p.buffer, p.byteOffset, p.byteLength);
  const i16 = (o) => dv.getInt16(o, true), u16 = (o) => dv.getUint16(o, true), f32 = (o) => dv.getFloat32(o, true);
  const flags = p[3];
  const common = {
    state: p[1], stateName: STATES[p[1]] || "?", mode: p[2], modeName: p[2] === 1 ? "MANUAL" : "ALT HOLD",
    armed: !!(flags & 1), airborne: !!(flags & 2), compass: !!(flags & 4), headingLock: !!(flags & 8),
    failsafe: !!(flags & 16), crashed: !!(flags & 32), pilotLink: !!(flags & 64), sensorsOk: !!(flags & 128),
    roll: i16(4) / 100, pitch: i16(6) / 100, yaw: u16(8) / 100,
  };
  if (p[0] === 2 && p.length >= 64) {
    const f2 = p[50];
    return Object.assign(common, {
      version: 2,
      alt: f32(10), altTarget: f32(14), vz: i16(18) / 100,
      rcRoll: i16(20) / 1000, rcPitch: i16(22) / 1000, rcYaw: i16(24) / 1000, rcThr: i16(26) / 1000,
      motors: [p[28] / 250, p[29] / 250, p[30] / 250, p[31] / 250],
      rollSp: i16(32) / 100, pitchSp: i16(34) / 100, headingTarget: u16(36) / 100,
      loopUs: u16(38), imuHz: p[40], reason: p[41], reasonName: REASONS[p[41]] || "?",
      crcErrors: u16(42), hoverCmd: u16(44) / 10000, collective: i16(46) / 10000, pilotHz: p[48],
      calKind: p[49] >> 4, calProgress: (p[49] & 15) / 15,
      posActive: !!(f2 & 1), posReady: !!(f2 & 2), posOn: !!(f2 & 4), altLimit: !!(f2 & 8), atCeiling: !!(f2 & 16),
      linkLost: !!(f2 & 32), gpsAid: !!(f2 & 64), stickAngle: !!(f2 & 128),
      posState: p[51] & 15, posStateName: POS_STATES[p[51] & 15] || "?",
      speedMode: p[51] >> 4, speedModeName: SPEED_MODES[p[51] >> 4] || "?",
      posN: i16(52) / 10, posE: i16(54) / 10, velN: i16(56) / 100, velE: i16(58) / 100,
      holdDist: u16(60) / 100, maxAlt: u16(62), yawOffset: p.length >= 66 ? i16(64) / 100 : NaN,
    });
  }
  if (p[0] !== 1) return null;
  return Object.assign(common, {
    version: 1,
    alt: i16(10) / 100, altTarget: i16(12) / 100, vz: i16(14) / 100,
    rcRoll: i16(16) / 1000, rcPitch: i16(18) / 1000, rcYaw: i16(20) / 1000, rcThr: i16(22) / 1000,
    motors: [p[24] / 250, p[25] / 250, p[26] / 250, p[27] / 250],
    rollSp: i16(28) / 100, pitchSp: i16(30) / 100, headingTarget: u16(32) / 100,
    loopUs: u16(34), imuHz: p[36], reason: p[37], reasonName: REASONS[p[37]] || "?",
    crcErrors: u16(38), hoverCmd: u16(40) / 10000, baroAlt: i16(42) / 100, pilotHz: p[44],
    calKind: p[45] >> 4, calProgress: (p[45] & 15) / 15, collective: i16(46) / 10000,
    posActive: false, posReady: false, posOn: false, posState: 0, posStateName: "-", speedMode: 1, speedModeName: "-",
    posN: 0, posE: 0, velN: 0, velE: 0, holdDist: 0, maxAlt: 0,
  });
}
