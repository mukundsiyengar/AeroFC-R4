// physics_x3.js - replica of the Gazebo "betaflight_x3" quadcopter used by the Aerovidya HIL simulator
// (gz_betaflight_bridge/models/betaflight_x3/model.sdf + worlds/quadcopter.sdf, MulticopterMotorModel plugin).
//
// Frames exactly like Gazebo:  world x = East, y = North, z = Up (ENU)    body = FLU (x forward, y left, z up)
// Sensors are produced the way Gazebo's sensors report them (IMU = specific force in the body frame, etc.).

export const X3 = {
  baseMass: 1.5, rotorMass: 0.005,
  inertia: [0.0347563, 0.07, 0.0977],
  rotors: [   // position (m, body FLU) and turning direction (ccw = +1, cw = -1), Gazebo actuator order 0..3
    { pos: [0.13, -0.22, 0.023], dir: +1, name: "FR" },
    { pos: [-0.13, 0.20, 0.023], dir: +1, name: "RL" },
    { pos: [0.13, 0.22, 0.023], dir: -1, name: "FL" },
    { pos: [-0.13, -0.20, 0.023], dir: -1, name: "RR" },
  ],
  motorConstant: 8.54858e-06, momentConstant: 0.016,
  timeConstantUp: 0.0125, timeConstantDown: 0.025,
  maxRotVelocity: 1000.0, rotorDragCoefficient: 8.06428e-05, rollingMomentCoefficient: 1e-06,
  gravity: 9.8,                         // SDF default gravity
  box: [0.47, 0.47, 0.11],              // base_link collision box
  // what Gazebo's magnetometer really reports (measured in gz-sim 8): its world-model field (NED, gauss) used as
  // if it were ENU, so 'magnetic north' points 90.8 deg east of GPS north (the SDF <magnetic_field> is ignored)
  magneticField: [0.39606878, -0.00573836, 0.14567122],
  lat0: 15.8497, lon0: 74.4977, elevation: 0,
};
export const SIM_DECLINATION_DEG = Math.atan2(X3.magneticField[0], X3.magneticField[1]) * 180 / Math.PI;

// ---------------------------------------------------------------- small vector / quaternion helpers
const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const scale = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
function qMul(a, b) {
  return [a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
          a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
          a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
          a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]];
}
function qNorm(q) { const n = Math.hypot(q[0], q[1], q[2], q[3]) || 1; return q.map((v) => v / n); }
export function qToMat(q) {
  const [w, x, y, z] = q;
  return [
    [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
    [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
    [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
  ];
}
const mv = (R, v) => [R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2], R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2], R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2]];
const mtv = (R, v) => [R[0][0] * v[0] + R[1][0] * v[1] + R[2][0] * v[2], R[0][1] * v[0] + R[1][1] * v[1] + R[2][1] * v[2], R[0][2] * v[0] + R[1][2] * v[1] + R[2][2] * v[2]];

// deterministic random numbers (tests must be repeatable)
export class Rng {
  constructor(seed = 1) { this.s = seed >>> 0 || 1; }
  next() { let x = this.s; x ^= x << 13; x ^= x >>> 17; x ^= x << 5; this.s = x >>> 0; return this.s / 4294967296; }
  gauss() { const u = Math.max(this.next(), 1e-12), v = this.next(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v); }
}

export const DEFAULT_ENV = {
  magDeclination: null,                      // deg; null = Gazebo's field (90.8 deg)
  magError: 0,                               // deg added to the field direction (a wrong compass)
  gpsNoise: 0, gpsDelay: 0, gpsFreeze: false, // GPS: noise (m), delay (s), frozen output (a dead GPS)
  gyroNoise: 0, gyroBias: [0, 0, 0], accNoise: 0, baroNoise: 0, magNoise: 0,
  wind: [0, 0, 0], gust: 0,                  // m/s (world ENU) and gust strength (m/s)
  inertiaScale: 1, massScale: 1, motorTauScale: 1, maxRotVelocity: 1000,
  seed: 1,
};

export class QuadX3 {
  constructor(env = {}) {
    this.env = { ...DEFAULT_ENV, ...env };
    this.rng = new Rng(this.env.seed);
    this.reset();
  }

  reset() {
    const e = this.env;
    this.mass = (X3.baseMass + 4 * X3.rotorMass) * e.massScale;
    // composite inertia: base + rotor point masses (parallel-axis), then the robustness scale factor
    const I = X3.inertia.slice();
    for (const r of X3.rotors) {
      const [x, y, z] = r.pos;
      I[0] += X3.rotorMass * (y * y + z * z);
      I[1] += X3.rotorMass * (x * x + z * z);
      I[2] += X3.rotorMass * (x * x + y * y);
    }
    this.I = I.map((v) => v * e.inertiaScale);
    this.p = [0, 0, X3.box[2] / 2];        // resting on the ground
    this.v = [0, 0, 0];
    this.q = [1, 0, 0, 0];                 // body FLU -> world ENU
    this.w = [0, 0, 0];                    // body rates (FLU)
    this.cmd = [0, 0, 0, 0];               // rad/s reference per rotor
    this.rot = [0, 0, 0, 0];               // filtered rotor speed (rad/s)
    this.t = 0;
    this.z0 = this.p[2];
    this.specForce = [0, 0, X3.gravity];
    this.gustVel = [0, 0, 0];
    this.contact = false;
    this.maxImpact = 0;
    // sensor sample-and-hold (Gazebo update rates: IMU 1000 Hz, altimeter 500, magnetometer 100, navsat 10)
    this.sens = { imu: [0, 0, 0, 0, 0, 0], alt: 0, vz: 0, mag: [0, 0, 0], gps: [X3.lat0, X3.lon0, X3.elevation] };
    this.nextAlt = 0; this.nextMag = 0; this.nextGps = 0;
    this.posHist = [];                     // for the GPS delay
    this.nextHist = 0;
    this.sampleSensors(true);
  }

  // motor commands from the bridge (rad/s), like /X3/gazebo/command/motor_speed
  setRotorVelocities(radps) { for (let i = 0; i < 4; i++) this.cmd[i] = Math.min(Math.max(radps[i], 0), this.env.maxRotVelocity); }

  step(dt) {
    const e = this.env;
    const R = qToMat(this.q);
    const wW = mv(R, this.w);                       // angular velocity in world
    let F = [0, 0, 0], tau = [0, 0, 0];             // non-gravity force and torque about the CoM (world)

    // wind with a slowly varying gust (first-order random walk)
    if (e.gust > 0) {
      const a = Math.exp(-dt / 2.0);
      for (let k = 0; k < 2; k++) this.gustVel[k] = a * this.gustVel[k] + Math.sqrt(1 - a * a) * e.gust * this.rng.gauss();
    } else this.gustVel = [0, 0, 0];
    const wind = add(e.wind, this.gustVel);

    for (let i = 0; i < 4; i++) {
      const r = X3.rotors[i];
      // first-order motor filter (Gazebo FirstOrderFilter, different up / down time constants)
      const tau_c = (this.cmd[i] > this.rot[i] ? X3.timeConstantUp : X3.timeConstantDown) * e.motorTauScale;
      const al = Math.exp(-dt / tau_c);
      this.rot[i] = al * this.rot[i] + (1 - al) * this.cmd[i];
      const omega = this.rot[i];
      const thrust = omega * omega * X3.motorConstant;
      const rW = mv(R, r.pos);
      const axis = [R[0][2], R[1][2], R[2][2]];     // rotor axis = body z in world
      const vPoint = add(this.v, cross(wW, rW));
      const vel = dot(vPoint, axis);
      const scalar = Math.min(Math.max(1 - vel / 25.0, 0), 1);
      const fThrust = scale(axis, thrust * scalar);
      const relWind = [vPoint[0] - wind[0], vPoint[1] - wind[1], vPoint[2] - wind[2]];
      const par = scale(axis, dot(relWind, axis));
      const perp = [relWind[0] - par[0], relWind[1] - par[1], relWind[2] - par[2]];
      const fDrag = scale(perp, -Math.abs(omega) * X3.rotorDragCoefficient);
      const fRotor = add(fThrust, fDrag);
      F = add(F, fRotor);
      tau = add(tau, cross(rW, fRotor));
      // reaction (drag) torque about the rotor axis + rolling moment
      tau = add(tau, scale(axis, -r.dir * thrust * X3.momentConstant));
      tau = add(tau, scale(perp, -Math.abs(omega) * X3.rollingMomentCoefficient));
    }

    // ground contact: spring-damper at the 8 corners of the collision box
    this.contact = false;
    const [hx, hy, hz] = [X3.box[0] / 2, X3.box[1] / 2, X3.box[2] / 2];
    const k = 2.0e4, c = 120, mu = 0.8;
    for (const sx of [-1, 1]) for (const sy of [-1, 1]) for (const sz of [-1, 1]) {
      const rW = mv(R, [sx * hx, sy * hy, sz * hz]);
      const z = this.p[2] + rW[2];
      if (z >= 0) continue;
      this.contact = true;
      const vP = add(this.v, cross(wW, rW));
      let fn = k * -z - c * vP[2];
      if (fn < 0) fn = 0;
      let ft = [-c * vP[0], -c * vP[1]];
      const ftn = Math.hypot(ft[0], ft[1]);
      if (ftn > mu * fn) ft = [ft[0] * mu * fn / ftn, ft[1] * mu * fn / ftn];
      const fc = [ft[0], ft[1], fn];
      F = add(F, fc);
      tau = add(tau, cross(rW, fc));
    }

    // translational motion (semi-implicit Euler, 1 ms like Gazebo's step)
    const aW = [F[0] / this.mass, F[1] / this.mass, F[2] / this.mass - X3.gravity];
    this.v = add(this.v, scale(aW, dt));
    this.p = add(this.p, scale(this.v, dt));
    // specific force (what an accelerometer measures), in the body frame
    this.specForce = mtv(R, scale(F, 1 / this.mass));
    this.maxImpact = Math.max(this.maxImpact, Math.hypot(...this.specForce));

    // rotational motion: I w' = tau_b - w x (I w)
    const tb = mtv(R, tau);
    const Iw = [this.I[0] * this.w[0], this.I[1] * this.w[1], this.I[2] * this.w[2]];
    const gyro = cross(this.w, Iw);
    for (let j = 0; j < 3; j++) this.w[j] += (tb[j] - gyro[j]) / this.I[j] * dt;
    const ang = Math.hypot(...this.w) * dt;
    let dq = [1, 0, 0, 0];
    if (ang > 1e-12) {
      const s = Math.sin(ang / 2) / (ang / dt);
      dq = [Math.cos(ang / 2), this.w[0] * s, this.w[1] * s, this.w[2] * s];
    }
    this.q = qNorm(qMul(this.q, dq));
    this.t += dt;
    this.sampleSensors(false);
  }

  sampleSensors(force) {
    const e = this.env, rng = this.rng, s = this.sens;
    const R = qToMat(this.q);
    // IMU every step (1 kHz)
    const f = this.specForce;
    s.imu = [
      f[0] + e.accNoise * rng.gauss(), f[1] + e.accNoise * rng.gauss(), f[2] + e.accNoise * rng.gauss(),
      this.w[0] + e.gyroBias[0] + e.gyroNoise * rng.gauss(),
      this.w[1] + e.gyroBias[1] + e.gyroNoise * rng.gauss(),
      this.w[2] + e.gyroBias[2] + e.gyroNoise * rng.gauss(),
    ];
    if (force || this.t >= this.nextAlt) {
      this.nextAlt = this.t + 1 / 500;
      s.alt = this.p[2] - this.z0 + e.baroNoise * rng.gauss();
      s.vz = this.v[2];
    }
    if (force || this.t >= this.nextMag) {
      this.nextMag = this.t + 1 / 100;
      let field = X3.magneticField;
      if ((e.magDeclination !== null && e.magDeclination !== undefined) || e.magError) {
        const dd = (e.magDeclination ?? SIM_DECLINATION_DEG) + (e.magError || 0);
        const h = Math.hypot(X3.magneticField[0], X3.magneticField[1]), d = dd * Math.PI / 180;
        field = [h * Math.sin(d), h * Math.cos(d), X3.magneticField[2]];
      }
      const m = mtv(R, field);
      const k = Math.hypot(...X3.magneticField) / 48.5e-6;   // magNoise is given in uT of a real 48.5 uT field
      s.mag = m.map((v) => v + e.magNoise * k * rng.gauss());
    }
    if (force || this.t >= this.nextHist) {      // position history every 10 ms for the GPS delay
      this.nextHist = this.t + 0.01;
      this.posHist.push([this.t, this.p[0], this.p[1], this.p[2]]);
      while (this.posHist.length > 200) this.posHist.shift();
    }
    if ((force || this.t >= this.nextGps) && !e.gpsFreeze) {
      this.nextGps = this.t + 1 / 10;
      let pp = this.p;
      if (e.gpsDelay > 0) {
        const tt = this.t - e.gpsDelay;
        const h = this.posHist.find((r) => r[0] >= tt) || this.posHist[0];
        if (h) pp = [h[1], h[2], h[3]];
      }
      const nx = e.gpsNoise ? e.gpsNoise * rng.gauss() : 0, ny = e.gpsNoise ? e.gpsNoise * rng.gauss() : 0;
      const lat = X3.lat0 + (pp[1] + ny) / 111320;
      const lon = X3.lon0 + (pp[0] + nx) / (111320 * Math.cos(X3.lat0 * Math.PI / 180));
      s.gps = [lat, lon, X3.elevation + pp[2]];
    }
  }

  // "truth" in the flight controller's convention (body FRD, earth NED), degrees
  truth() {
    const R = qToMat(this.q);
    // R_ned_frd = T * R_enu_flu * D   with T = ENU->NED, D = FRD->FLU = diag(1,-1,-1)
    const Rn = [
      [R[1][0], -R[1][1], -R[1][2]],
      [R[0][0], -R[0][1], -R[0][2]],
      [-R[2][0], R[2][1], R[2][2]],
    ];
    const roll = Math.atan2(Rn[2][1], Rn[2][2]);
    const pitch = -Math.asin(Math.max(-1, Math.min(1, Rn[2][0])));
    let yaw = Math.atan2(Rn[1][0], Rn[0][0]);
    const d = 180 / Math.PI;
    let yawMag = yaw * d - SIM_DECLINATION_DEG;
    while (yawMag < 0) yawMag += 360;
    while (yawMag >= 360) yawMag -= 360;
    let yawDeg = yaw * d;
    if (yawDeg < 0) yawDeg += 360;
    return {
      roll: roll * d, pitch: pitch * d, yaw: yawDeg, yawMag,
      alt: this.p[2] - this.z0, vz: this.v[2],
      north: this.p[1], east: this.p[0], speed: Math.hypot(this.v[0], this.v[1]),
      tilt: Math.acos(Math.max(-1, Math.min(1, R[2][2]))) * d,
    };
  }
}
