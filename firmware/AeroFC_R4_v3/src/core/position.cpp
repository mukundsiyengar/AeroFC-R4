#include "position.h"

const char *posStateName(uint8_t s) {
  static const char *const n[] = { "OFF", "NOT READY", "MOVING", "BRAKING", "HOLDING", "FAILED" };
  return s < 6 ? n[s] : "?";
}

// ------------------------------------------------------------------ 1-D Kalman filter [p, v, accel bias]
void PosKf1::reset(float pos, float vel) {
  p = pos; v = vel; b = 0.0f;
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) P[i][j] = 0.0f;
  P[0][0] = 0.5f; P[1][1] = 0.5f; P[2][2] = 0.2f;
}

void PosKf1::predict(float acc, float dt, float accNoise, float biasWalk) {
  if (!fcFinite(acc)) acc = 0.0f;
  const float a = acc - b;
  p += v * dt + 0.5f * a * dt * dt;
  v += a * dt;
  const float h = 0.5f * dt * dt;
  const float F[3][3] = { { 1.0f, dt, -h }, { 0.0f, 1.0f, -dt }, { 0.0f, 0.0f, 1.0f } };
  float FP[3][3], N[3][3];
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) FP[i][j] = F[i][0] * P[0][j] + F[i][1] * P[1][j] + F[i][2] * P[2][j];
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) N[i][j] = FP[i][0] * F[j][0] + FP[i][1] * F[j][1] + FP[i][2] * F[j][2];
  const float sa2 = accNoise * accNoise;
  const float G[2] = { h, dt };
  for (int i = 0; i < 2; i++)
    for (int j = 0; j < 2; j++) N[i][j] += sa2 * G[i] * G[j];
  N[2][2] += biasWalk * biasWalk * dt;
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) P[i][j] = 0.5f * (N[i][j] + N[j][i]);
}

void PosKf1::update(float meas, float R) {
  const float S = P[0][0] + R;
  const float K[3] = { P[0][0] / S, P[1][0] / S, P[2][0] / S };
  const float innov = meas - p;
  p += K[0] * innov;
  v += K[1] * innov;
  b = fcClamp(b + K[2] * innov, -3.0f, 3.0f);
  const float row0[3] = { P[0][0], P[0][1], P[0][2] };
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) P[i][j] -= K[i] * row0[j];
}

// ------------------------------------------------------------------ estimator
static const float GPS_R_MIN = 0.15f * 0.15f;   // m^2 per fix: Gazebo's NavSat has no noise (covers the 10 Hz steps)

void PositionEstimator::predict(float accN, float accE, float dt) {
  if (!init) return;
  // how fast the IMU's error may change: quick with a clean GPS (a gyro that starts drifting is caught in seconds),
  // slow with a noisy one, or the filter would follow the noise
  const float errorWalk = gpsQuiet() ? 0.3f : 0.05f;
  kn.predict(accN, dt, 0.35f, errorWalk);
  ke.predict(accE, dt, 0.35f, errorWalk);
}

// Adaptive GPS noise: the corrections themselves tell how noisy the GPS is.  A clean GPS keeps the filter sharp,
// a noisy one is trusted less, so the drone does not chase the noise.
float PositionEstimator::gpsNoiseRaw() const {
  return sqrtf(fcMax(innovSq * 0.5f - kn.P[0][0], 0.0f));
}
float PositionEstimator::gpsNoise() const {
  const float r = innovSq * 0.5f - kn.P[0][0];
  return sqrtf(fcClamp(r, GPS_R_MIN, 1.0f));
}

bool PositionEstimator::gpsFix(float n, float e, float vN, float vE) {
  if (!fcFinite(n) || !fcFinite(e)) return false;
  if (!init) {
    kn.reset(n, vN);
    ke.reset(e, vE);
    init = true;
    rejects = 0;
    innovSq = 0.0f;
    return true;
  }
  const float dn = n - kn.p, de = e - ke.p;
  const float S = kn.P[0][0] + fcSq(gpsNoise());
  // a jump of more than 3 m and 5 sigma is not believed ... unless it keeps happening (then the GPS is right)
  if ((dn * dn + de * de) > 9.0f && (dn * dn + de * de) > 25.0f * S) {
    if (++rejects < 25) return false;
    kn.reset(n, vN);
    ke.reset(e, vE);
    rejects = 0;
    return true;
  }
  rejects = 0;
  innovSq += 0.01f * (fcMin(dn * dn + de * de, 4.0f) - innovSq);
  const float R = fcSq(gpsNoise());
  kn.update(n, R);
  ke.update(e, R);
  return true;
}

float PositionEstimator::sigma() const { return init ? sqrtf(fcMax(kn.P[0][0], ke.P[0][0])) : 99.0f; }

// ------------------------------------------------------------------ controller
void PositionController::reset(float pN, float pE) {
  holdN = pN; holdE = pE;
  vSpN = vSpE = 0.0f;
  iN = iE = 0.0f;
  dist = 0.0f;
  state = POS_HOLDING;
  failed = false;
  releaseT_ = 1.0f;
  runawayT_ = 0.0f;
  lastDist_ = 0.0f;
  wasActive_ = false;
}

void PositionController::update(const Params &p, const SpeedProfile &prof, const PosCtlIn &in, float dt,
                                float &rollSp, float &pitchSp) {
  const float g = FC_GRAVITY;
  float tiltMax = prof.brakeTiltDeg * FC_DEG2RAD;
  if (in.heightAgl < 1.0f) tiltMax = fcMin(tiltMax, 10.0f * FC_DEG2RAD);   // gentle near the ground
  const float aMax = g * tanf(tiltMax);
  const float aBrake = 0.6f * aMax;
  const float speed = sqrtf(in.vN * in.vN + in.vE * in.vE);
  const float cy = cosf(in.yaw), sy = sinf(in.yaw);
  const float co = cosf(in.yawOffset), so = sinf(in.yawOffset);

  rollSp = pitchSp = 0.0f;
  // on the ground / touching down: no tilt, hold where we are, keep the wind estimate frozen
  if (!in.airborne || in.touchdown) {
    holdN = in.pN; holdE = in.pE;
    vSpN = vSpE = 0.0f;
    dist = 0.0f;
    state = POS_HOLDING;
    wasActive_ = false;
    return;
  }

  // sticks: active while moved, plus 0.2 s after release (debounce)
  const bool moved = in.sticksAllowed && (fcAbs(in.stickFwd) > 0.02f || fcAbs(in.stickRight) > 0.02f);
  releaseT_ = moved ? 0.0f : releaseT_ + dt;
  const bool active = moved || releaseT_ < 0.2f;
  // where we would stop if we braked now (keeps the hold point from springing back)
  const float lag = 0.25f;                              // time to turn the tilt around before braking starts
  const float stopN = in.pN + in.vN * (lag + speed / (2.0f * aBrake));
  const float stopE = in.pE + in.vE * (lag + speed / (2.0f * aBrake));

  if (active && in.stickStyle == STICK_ANGLE) {        // stick = tilt, exactly as without position hold
    const float maxTilt = prof.maxTiltDeg * FC_DEG2RAD;
    rollSp = in.stickRight * maxTilt;
    pitchSp = -in.stickFwd * maxTilt;
    holdN = stopN; holdE = stopE;
    state = POS_MOVING;
    wasActive_ = true;
    runawayT_ = 0.0f;
    return;
  }

  const float posP = p.posP, velP = p.velP, velI = p.velI;
  bool integrate = in.heightAgl > 0.2f;
  if (active) {                                         // SPEED sticks: stick = wanted speed (relative to the nose)
    const float vf = in.stickFwd * prof.maxSpeed, vr = in.stickRight * prof.maxSpeed;
    const float nE = cy * vf - sy * vr, eE = sy * vf + cy * vr;            // compass frame
    vSpN = co * nE + so * eE;                                              // -> GPS frame
    vSpE = -so * nE + co * eE;
    holdN = stopN; holdE = stopE;
    state = POS_MOVING;
    wasActive_ = true;
    runawayT_ = 0.0f;
  } else {
    if (wasActive_) {                                   // just released: brake to a stop first
      wasActive_ = false;
      state = POS_BRAKING;
      brakeT_ = 0.0f;
    }
    if (state == POS_MOVING) { state = POS_BRAKING; brakeT_ = 0.0f; }
    if (state == POS_BRAKING) {
      brakeT_ += dt;
      vSpN = vSpE = 0.0f;
      holdN = in.pN + in.vN * lag; holdE = in.pE + in.vE * lag;
      if (speed < 0.4f || brakeT_ > 6.0f) state = POS_HOLDING;   // stopped: hold right here
    }
    const float eN = holdN - in.pN, eE = holdE - in.pE;
    dist = sqrtf(eN * eN + eE * eE);
    if (state == POS_HOLDING) {
      float vs = posP * dist;
      vs = fcMin(vs, fcMin(prof.maxSpeed, sqrtf(2.0f * aBrake * 0.7f * dist)));   // smooth approach from far away
      if (dist > 1e-3f) { vSpN = eN / dist * vs; vSpE = eE / dist * vs; }
      else { vSpN = vSpE = 0.0f; }
    }
    // runaway detector: far away and still getting further, or fast for a long time without sticks
    if ((dist > 8.0f && dist > lastDist_ + 0.001f) || (state == POS_HOLDING && speed > 2.5f)) runawayT_ += dt;
    else runawayT_ = fcMax(0.0f, runawayT_ - dt);
    if (runawayT_ > 3.0f) failed = true;
  }
  lastDist_ = dist;

  // speed loop (PI) in the GPS frame
  const float evN = vSpN - in.vN, evE = vSpE - in.vE;
  float aN = velP * evN + iN, aE = velP * evE + iE;
  const float an = sqrtf(aN * aN + aE * aE);
  const bool saturated = an > aMax;
  if (integrate && !saturated) {
    iN = fcClamp(iN + velI * evN * dt, -1.5f, 1.5f);
    iE = fcClamp(iE + velI * evE * dt, -1.5f, 1.5f);
  }
  if (saturated) { aN *= aMax / an; aE *= aMax / an; }

  // GPS frame -> compass frame -> body (forward / right) -> tilt
  const float aNe = co * aN - so * aE, aEe = so * aN + co * aE;
  const float af = cy * aNe + sy * aEe, ar = -sy * aNe + cy * aEe;
  pitchSp = -atanf(af / g);
  rollSp = atanf(ar * cosf(pitchSp) / g);
}
