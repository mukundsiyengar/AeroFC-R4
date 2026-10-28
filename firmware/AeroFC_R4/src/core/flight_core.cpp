#include "flight_core.h"

static const float ARM_MAX_TILT_DEG = 15.0f;
static const uint32_t SENSOR_STALE_US = 300000;     // 0.3 s without IMU -> sensors not OK
static const uint32_t SENSOR_LOST_US = 1500000;     // 1.5 s without IMU while armed -> disarm

const char *stateName(uint8_t s) {
  static const char *const n[] = { "DISARMED", "IDLE", "TAKEOFF", "FLYING", "LANDING", "FAILSAFE" };
  return s < 6 ? n[s] : "?";
}
const char *reasonName(uint8_t r) {
  static const char *const n[] = { "-", "user", "pilot link", "sensors", "estimator", "CRASH", "STOP",
                                   "KILL", "idle", "landed", "geofence" };
  return r < 11 ? n[r] : "?";
}
const char *armResultName(int8_t a) {
  static const char *const n[] = { "OK", "already", "no sensors", "EKF not ready", "not level", "calibrating",
                                   "too soon" };
  return (a >= 0 && a < 7) ? n[a] : "?";
}

void FlightCore::init(Params *p, CalData *c) {
  prm = p;
  cal = c;
  sens.setup(p, c);
  ekf.reset();
  altf.reset(0.0f);
  att.reset(0.0f);
  altc.reset(hoverThrustFromCmd(*p));
  state = ST_DISARMED;
  mode = MODE_ALT_HOLD;
  reason = R_NONE;
  pilot = PilotState();
  setOutputsOff();
}

void FlightCore::setOutputsOff() {
  for (int i = 0; i < 4; i++) { motor[i] = 0.0f; mix.cmd[i] = 0.0f; mix.thrust[i] = 0.0f; }
  collective = 0.0f;
  torque = Vec3();
}

void FlightCore::resetEstimator() {
  ekf.reset();
  altInit_ = false;
}

bool FlightCore::startCalibration(uint8_t kind) {
  // the compass can only be turned around in flight, so it is the one calibration allowed while armed
  if (armed() && kind != SensorProcessor::CAL_MAG) return false;
  sens.startCalibration((SensorProcessor::CalKind)kind);
  return true;
}

bool FlightCore::finishCompassCalibration() {
  const bool ok = sens.calFinishMag();
  ekf.resetMagHealth();
  return ok;
}

// ------------------------------------------------------------------ sensor inputs
void FlightCore::onImu(const RawImu &r, uint64_t tsNs, uint32_t nowUs) {
  // time step: from the packet timestamps (simulator clock), else from our own clock
  float dt = 0.02f;
  if (haveImu_) {
    const float dtArr = (float)(uint32_t)(nowUs - lastImuUs_) * 1e-6f;
    dt = dtArr;
    if (tsNs > lastTs_) {
      const float dtTs = (float)(tsNs - lastTs_) * 1e-9f;
      if (dtTs > 0.001f && dtTs < 0.2f) dt = dtTs;
    }
    dt = fcClamp(dt, 0.002f, 0.1f);
  }
  haveImu_ = true;
  lastTs_ = tsNs;
  lastImuUs_ = nowUs;
  imuCount_++;

  const uint8_t calBefore = (uint8_t)sens.calibrating();
  sens.onImu(r, sf);
  // a finished gyro / level calibration: restart the estimator so it starts from the new offsets
  if (calBefore != 0 && sens.calibrating() == 0 && sens.lastCalResult == 1 &&
      (calBefore == SensorProcessor::CAL_GYRO || calBefore == SensorProcessor::CAL_LEVEL))
    resetEstimator();

  sensorsOk = sf.imuValid;
  if (!sf.imuValid) {             // zeros before Gazebo data arrives: keep sending the current (safe) outputs
    motorsReady = true;
    return;
  }
  if (!ekf.initialized()) {
    if (ekf.initStep(sf.acc, sf.mag, prm->compassOn > 0.5f, prm->magDeclDeg * FC_DEG2RAD)) {
      altf.reset(sf.baroAlt);
      altInit_ = true;
      att.reset(ekf.yaw());
    }
    // estimator (re)starting while armed: keep the drone roughly level with hover thrust and no torque
    if (armed() && state != ST_IDLE) {
      collective = 0.95f * hoverThrustFromCmd(*prm);
      mixerRun(*prm, collective, 0.0f, 0.0f, 0.0f, true, mix);
      for (int i = 0; i < 4; i++) motor[i] = mix.cmd[i];
    }
    motorsReady = true;
    return;
  }
  if ((uint8_t)prm->loopMode == LOOP_SYNC) {
    runControl(dt, true, nowUs);
  } else {
    newImu_ = true;               // the fixed-rate loop in tick() uses it
  }
}

void FlightCore::onBaro(float altM) {
  sens.onBaro(altM, sf);
  if (altInit_) altf.updateBaro(sf.baroAlt, prm->baroNoise);
}

void FlightCore::onMag(float x, float y, float z) {
  sens.onMag(x, y, z, sf);
  if (ekf.initialized() && prm->compassOn > 0.5f) ekf.updateMag(sf.mag, prm->magDeclDeg * FC_DEG2RAD, 0.09f);
}

void FlightCore::onGps(double lat, double lon, float alt) {
  sens.onGps(lat, lon, alt, sf);
  if (lat == 0.0 && lon == 0.0) return;
  if (!gpsHome) {
    lat0_ = lat; lon0_ = lon; gpsHome = true; gpsChain.reset(); imuChain.reset();
    aidYawOffset = 0.0f;
    aidErrN = aidErrE = 0.0f;
    for (int a = 0; a < 3; a++) for (int b = 0; b < 3; b++) aidP[a][b] = 0.0f;
    aidP[0][0] = fcSq(0.5f); aidP[1][1] = aidP[2][2] = fcSq(0.5f);
  }
  const double kLat = 111320.0;
  homeNorth = (float)((lat - lat0_) * kLat);
  homeEast = (float)((lon - lon0_) * kLat * (double)cosf((float)lat0_ * FC_DEG2RAD));
  if (!ekf.initialized()) return;
  const uint32_t now = lastImuUs_;
  gpsChain.push(homeNorth, homeEast, now);
  imuChain.push(imuPosN_, imuPosE_, now);
  lastGpsUs_ = now;

  // Small Kalman filter, state x = [yaw offset between GPS (true north) and EKF (compass) frames, errN, errE]:
  //   imuChainAcc - gpsChainAcc = yawOffset * rot90(gpsAcc) + err
  // A heading offset grows with the acceleration and turns with it, the attitude error does not, so both can be
  // told apart as the drone manoeuvres.  This is why the compass declination does not need to be known.
  // The GPS only changes 10 times a second, so while the acceleration changes quickly the two chains briefly
  // disagree: learn only while the acceleration is steady.
  const float jerk = sqrtf(fcSq(gpsChain.accN - aidPrevAccN) + fcSq(gpsChain.accE - aidPrevAccE)) * 50.0f;
  aidPrevAccN = gpsChain.accN;
  aidPrevAccE = gpsChain.accE;
  if (gpsChain.ready() && imuChain.ready() && jerk < 1.5f) {
    const float aN = gpsChain.accN, aE = gpsChain.accE;
    const float zN = imuChain.accN - gpsChain.accN, zE = imuChain.accE - gpsChain.accE;
    aidP[1][1] += 0.0005f; aidP[2][2] += 0.0005f;    // the attitude error changes slowly (the EKF corrects it)
    aidP[0][0] += 1e-7f;
    const float H[2][3] = { { -aE, 1.0f, 0.0f }, { aN, 0.0f, 1.0f } };
    const float z[2] = { zN, zE };
    for (int r = 0; r < 2; r++) {
      float PH[3], S = fcSq(0.05f);
      for (int a = 0; a < 3; a++) PH[a] = aidP[a][0] * H[r][0] + aidP[a][1] * H[r][1] + aidP[a][2] * H[r][2];
      for (int a = 0; a < 3; a++) S += H[r][a] * PH[a];
      const float pred = H[r][0] * aidYawOffset + H[r][1] * aidErrN + H[r][2] * aidErrE;
      const float innov = fcClamp(z[r] - pred, -2.0f, 2.0f);
      float K[3];
      for (int a = 0; a < 3; a++) K[a] = PH[a] / S;
      aidYawOffset = fcClamp(aidYawOffset + K[0] * innov, -0.8f, 0.8f);
      aidErrN += K[1] * innov;
      aidErrE += K[2] * innov;
      for (int a = 0; a < 3; a++)
        for (int b = 0; b < 3; b++) aidP[a][b] -= K[a] * PH[b];
    }
  }
}

void DiffChain::push(float n, float e, uint32_t now) {
  const uint8_t i = head;
  pN[i] = n; pE[i] = e; t[i] = now;
  float vn = velN, ve = velE;
  bool haveV = false;
  const int LAG = 10;   // ~0.2 s at 50 packets per second (GPS itself updates at 10 Hz)
  if (count >= LAG) {
    const uint8_t j = (uint8_t)((i + N - LAG) % N);
    const float dt = (float)(uint32_t)(now - t[j]) * 1e-6f;
    if (dt > 0.05f && dt < 1.0f) { vn = (n - pN[j]) / dt; ve = (e - pE[j]) / dt; haveV = true; }
  }
  vN[i] = vn; vE[i] = ve;
  if (haveV) {
    const float k = 0.15f;
    velN += k * (vn - velN);
    velE += k * (ve - velE);
    if (count >= 2 * LAG) {
      const uint8_t j = (uint8_t)((i + N - LAG) % N);
      const float dt = (float)(uint32_t)(now - t[j]) * 1e-6f;
      if (dt > 0.05f && dt < 1.0f) {
        accN += k * (fcClamp((vn - vN[j]) / dt, -8.0f, 8.0f) - accN);
        accE += k * (fcClamp((ve - vE[j]) / dt, -8.0f, 8.0f) - accE);
      }
    }
  }
  head = (uint8_t)((i + 1) % N);
  if (count < 255) count++;
}

// ------------------------------------------------------------------ pilot input
void FlightCore::onControlInput(float thr, float r, float pt, float y, uint32_t nowUs) {
  if (!fcFinite(thr)) thr = 0.0f;
  if (!fcFinite(r)) r = 0.0f;
  if (!fcFinite(pt)) pt = 0.0f;
  if (!fcFinite(y)) y = 0.0f;
  pilot.rawThr = thr; pilot.rawRoll = r; pilot.rawPitch = pt; pilot.rawYaw = y;
  pilot.lastUs = nowUs;
  pilot.everReceived = true;

  // shape the sticks
  float sticks[3] = { fcClamp(r, -1.0f, 1.0f), fcClamp(pt, -1.0f, 1.0f), fcClamp(y, -1.0f, 1.0f) };
  if (prm->invRoll > 0.5f) sticks[0] = -sticks[0];
  if (prm->invPitch > 0.5f) sticks[1] = -sticks[1];
  if (prm->invYaw > 0.5f) sticks[2] = -sticks[2];
  for (int i = 0; i < 3; i++) sticks[i] = fcExpo(fcDeadband(sticks[i], prm->deadband), prm->expo);
  pilot.roll = sticks[0];
  pilot.pitch = sticks[1];
  pilot.yaw = sticks[2];

  bool extended = false;
  if (thr >= 9.5f && thr < 20.0f) {                  // command code
    const int code = (int)(thr + 0.5f);
    extended = true;
    if (code != pilot.lastCode) { pilot.lastCode = code; handleCode(code, nowUs); }
  } else {
    pilot.lastCode = 0;
    if (thr >= 1.5f && thr <= 3.5f) {                 // our pilot: altitude hold + climb stick
      pilot.manual = false;
      pilot.climb = fcDeadband(fcClamp((thr - 2.5f) * 2.0f, -1.0f, 1.0f), 0.05f);
      extended = true;
    } else if (thr > 0.0005f && thr <= 1.0f) {        // stock semantics: manual collective
      pilot.manual = true;
      pilot.manualCmd = thr;
      pilot.climb = 0.0f;
    } else {                                          // 0 = altitude hold (stock semantics)
      pilot.manual = false;
      pilot.climb = 0.0f;
    }
  }

  // pilot-link monitoring only for a continuous stream of OUR packets (the stock tool sends only on key presses)
  if (!extended) {
    pilot.streamSeen = false;   // stock tool (sends only on key presses): never treat its silence as link loss
    pilot.winCount = 0;
  } else {
    if ((uint32_t)(nowUs - pilot.winStartUs) >= 1000000u) {
      pilot.rateHz = (uint8_t)fcMin((float)pilot.winCount, 255.0f);
      if (pilot.winCount >= 10) pilot.streamSeen = true;
      pilot.winCount = 0;
      pilot.winStartUs = nowUs;
    }
    pilot.winCount++;
  }
  if (pilot.linkLost) {
    pilot.linkLost = false;
    if (state == ST_FAILSAFE && reason == R_PILOT_LINK) {   // link is back: give control back to the pilot
      reason = R_NONE;
      enterFlying(MODE_ALT_HOLD);
    }
  }
}

void FlightCore::handleCode(int code, uint32_t nowUs) {
  switch (code) {
    case CODE_ARM: lastArmResult = cmdArm(nowUs); break;
    case CODE_DISARM: if (!airborne) cmdDisarm(R_USER); break;
    case CODE_TAKEOFF: lastArmResult = cmdTakeoff(nowUs); break;
    case CODE_LAND: cmdLand(); break;
    case CODE_KILL: cmdKill(); break;
    default: break;
  }
}

void FlightCore::onSessionStart(uint32_t nowUs) {
  if (armed()) return;                 // re-handshake in flight: just keep flying
  if (prm->autoTakeoffOn > 0.5f) {
    pendingAutoTakeoff_ = true;        // the first sensor packets are zeros: retry until the EKF is ready
    autoTakeoffUntil_ = nowUs + 15000000u;
  }
}

void FlightCore::onSessionStop() {
  pendingAutoTakeoff_ = false;
  cmdDisarm(R_STOP);
}

// ------------------------------------------------------------------ commands
int8_t FlightCore::cmdArm(uint32_t nowUs) {
  if (armed()) return ARM_ALREADY;
  if (sens.calibrating()) return ARM_CALIBRATING;
  if (!sensorsOk || !haveImu_ || (uint32_t)(nowUs - lastImuUs_) > SENSOR_STALE_US) return ARM_NO_SENSORS;
  if (!ekf.initialized() || !ekf.healthy()) return ARM_EST_NOT_READY;
  if (tiltDeg > ARM_MAX_TILT_DEG) return ARM_NOT_LEVEL;
  if ((uint32_t)(nowUs - disarmUs_) < 500000u && disarmUs_ != 0) return ARM_TOO_SOON;
  state = ST_IDLE;
  mode = pilot.manual ? MODE_MANUAL : MODE_ALT_HOLD;
  reason = R_NONE;
  crashed = geofence = false;
  airborne = false;
  groundAlt = altf.alt;
  altTarget = groundAlt;
  att.reset(ekf.yaw());
  altc.reset(hoverThrustFromCmd(*prm));
  idleTimer_ = landTimer_ = crashTimer_ = 0.0f;
  return ARM_OK;
}

void FlightCore::cmdDisarm(uint8_t why) {
  if (armed()) reason = why;
  if (why == R_CRASH) crashed = true;
  state = ST_DISARMED;
  airborne = false;
  disarmUs_ = lastImuUs_ ? lastImuUs_ : 1u;
  setOutputsOff();
  motorsReady = true;
}

int8_t FlightCore::cmdTakeoff(uint32_t nowUs) {
  if (state == ST_DISARMED) {
    const int8_t a = cmdArm(nowUs);
    if (a != ARM_OK) return a;
  }
  if (state == ST_IDLE) {
    state = ST_TAKEOFF;
    mode = MODE_ALT_HOLD;
    pilot.manual = false;
    takeoffGoal = fcMin(groundAlt + prm->takeoffAlt, prm->maxAlt);
    altTarget = altf.alt;
    takeoffTimer_ = 0.0f;
    return ARM_OK;
  }
  if (state == ST_LANDING || (state == ST_FAILSAFE && reason != R_PILOT_LINK)) {
    enterFlying(MODE_ALT_HOLD);         // cancel the landing, hold here
    reason = R_NONE;
  }
  return ARM_OK;
}

void FlightCore::cmdLand() {
  if (state == ST_IDLE) { cmdDisarm(R_USER); return; }
  if (state == ST_TAKEOFF || state == ST_FLYING) {
    state = ST_LANDING;
    landTimer_ = 0.0f;
  }
}

void FlightCore::cmdKill() {
  pendingAutoTakeoff_ = false;
  cmdDisarm(R_KILL);
}

void FlightCore::enterFlying(uint8_t m) {
  state = ST_FLYING;
  if (m == MODE_ALT_HOLD && mode == MODE_MANUAL) {
    // bumpless switch: start the altitude loop from the thrust the pilot was using
    altc.integ = fcClamp(collective * fcMax(fcCosTilt(ekf.q), 0.82f), 0.05f, 0.95f);
  }
  mode = m;
  altTarget = altf.alt;
  holdCapture_ = false;
}

// ------------------------------------------------------------------ periodic
void FlightCore::tick(uint32_t nowUs) {
  if ((uint32_t)(nowUs - rateWinUs_) >= 1000000u) {
    const float s = (float)(uint32_t)(nowUs - rateWinUs_) * 1e-6f;
    imuHz = (float)imuCount_ / s;
    ctrlHz = (float)ctrlCount_ / s;
    imuCount_ = ctrlCount_ = 0;
    rateWinUs_ = nowUs;
  }
  const uint32_t sinceImu = haveImu_ ? (uint32_t)(nowUs - lastImuUs_) : 0xFFFFFFFFu;
  if (sinceImu > SENSOR_STALE_US) sensorsOk = false;
  if (armed() && sinceImu > SENSOR_LOST_US) cmdDisarm(R_SENSORS);

  // pilot link loss (only after our continuous stream was seen)
  if (pilot.streamSeen && !pilot.linkLost && (uint32_t)(nowUs - pilot.lastUs) > (uint32_t)(prm->pilotTimeout * 1e6f)) {
    pilot.linkLost = true;
    pilot.roll = pilot.pitch = pilot.yaw = pilot.climb = 0.0f;
    pilot.manual = false;
    if ((state == ST_FLYING || state == ST_TAKEOFF) && (uint8_t)prm->fsAction == FS_LAND) {
      state = ST_FAILSAFE;
      reason = R_PILOT_LINK;
      landTimer_ = 0.0f;
    } else if (state == ST_FLYING && mode == MODE_MANUAL) {
      enterFlying(MODE_ALT_HOLD);   // HOLD / NONE: never keep a fixed manual throttle without a pilot
    }
  }

  // fixed-rate loop (100 / 200 Hz): predict between sensor bursts
  const uint8_t lm = (uint8_t)prm->loopMode;
  if (lm != LOOP_SYNC && ekf.initialized() && sensorsOk) {
    const uint32_t period = (lm == LOOP_100HZ) ? 10000u : 5000u;
    const uint32_t el = (uint32_t)(nowUs - lastCtrlUs_);
    if (el >= period) {
      runControl(fcClamp((float)el * 1e-6f, 0.002f, 0.05f), newImu_, nowUs);
      newImu_ = false;
      lastCtrlUs_ = nowUs;
    }
  }
}

// ------------------------------------------------------------------ the control step
void FlightCore::runControl(float dt, bool newImu, uint32_t nowUs) {
  ctrlCount_++;
  lastCtrlUs_ = nowUs;
  const Params &p = *prm;

  // ---- attitude estimate
  ekf.predict(sf.gyro, dt, p.ekfGyroNoise);
  const float tiltCmd = fcMax(fcAbs(rollSp), fcAbs(pitchSp));
  // horizontal acceleration seen by the IMU in the EKF frame, integrated twice for the GPS comparison chain
  {
    const Vec3 fe = ekf.q.rotate(sf.acc);
    imuAccN_ = fe.x;
    imuAccE_ = fe.y;
    imuVelN_ += imuAccN_ * dt; imuVelE_ += imuAccE_ * dt;
    imuPosN_ += imuVelN_ * dt; imuPosE_ += imuVelE_ * dt;
  }
  gpsAidOk = p.gpsAidOn > 0.5f && gpsChain.ready() && imuChain.ready() && (uint32_t)(nowUs - lastGpsUs_) < 500000u;
  if (newImu) {
    if (!airborne) {
      ekf.updateAccel(sf.acc, 0.5f, 0.15f);                                   // on the ground: trust it
    } else if (gpsAidOk) {
      // GPS-aided (complementary): fast part from the IMU, slow part from the GPS.  Both chains have the same
      // delay, so (GPS - IMU) through the chain is the slow attitude error, without any lag problem.
      // remove the (slow) attitude-induced error from the instantaneous IMU acceleration
      const float yo = aidYawOffset;
      const float eN = imuChain.accN - (gpsChain.accN - yo * gpsChain.accE);
      const float eE = imuChain.accE - (gpsChain.accE + yo * gpsChain.accN);
      const Vec3 ref(imuAccN_ - eN, imuAccE_ - eE, -FC_GRAVITY);
      ekf.updateAccel(sf.acc, p.ekfAccNoiseFlight, 0.1f, &ref);
    } else if (tiltCmd < 5.0f * FC_DEG2RAD) {
      ekf.updateAccel(sf.acc, p.ekfAccNoiseFlight, 0.05f);                    // no GPS: gently, only when level
    }
  }
  if (!ekf.healthy()) {
    resetEstimator();
    if (armed() && airborne) { state = ST_FAILSAFE; reason = R_ESTIMATOR; }
    motorsReady = true;
    return;
  }
  const float cosT = fcCosTilt(ekf.q);
  tiltDeg = acosf(fcClamp(cosT, -1.0f, 1.0f)) * FC_RAD2DEG;

  // ---- vertical estimate: earth-frame vertical acceleration = rotated specific force + gravity
  const Vec3 fe = ekf.q.rotate(sf.acc);
  const float aUp = -(fe.z + FC_GRAVITY);
  altf.predict(aUp, dt, 0.5f, 0.01f);

  flightLogic(dt, nowUs);
  motorsReady = true;
}

void FlightCore::flightLogic(float dt, uint32_t nowUs) {
  const Params &p = *prm;
  const float alt = altf.alt, vz = altf.vz;
  const float hoverT = hoverThrustFromCmd(p);

  // ---- automatic take-off requested by the simulator's START
  if (pendingAutoTakeoff_) {
    if (armed()) pendingAutoTakeoff_ = false;
    else if (cmdTakeoff(nowUs) == ARM_OK) pendingAutoTakeoff_ = false;
    else if ((int32_t)(nowUs - autoTakeoffUntil_) > 0) pendingAutoTakeoff_ = false;
  }

  if (!armed()) {
    setOutputsOff();
    rollSp = pitchSp = 0.0f;
    att.reset(ekf.yaw());
    altTarget = alt;
    return;
  }

  // ---- crash detection
  if (p.crashDetectOn > 0.5f && state != ST_IDLE) {
    crashTimer_ = (tiltDeg > p.crashTiltDeg) ? crashTimer_ + dt : 0.0f;
    const bool impact = airborne && sf.acc.norm() > 6.0f * FC_GRAVITY && tiltDeg > 30.0f;   // not a hard landing
    if (crashTimer_ > 0.2f || impact) { cmdDisarm(R_CRASH); return; }
  }

  // ---- airborne?
  if (state != ST_IDLE && !airborne && (alt - groundAlt > 0.15f || vz > 0.3f)) airborne = true;

  // ---- pilot mode changes while flying (stock: throttle > 0 = manual, throttle 0 = altitude hold)
  if (state == ST_FLYING) {
    if (pilot.manual && mode != MODE_MANUAL) mode = MODE_MANUAL;
    else if (!pilot.manual && mode == MODE_MANUAL) enterFlying(MODE_ALT_HOLD);
  }

  // ---- attitude setpoints
  const bool pilotOk = !(state == ST_FAILSAFE) && !pilot.linkLost;
  const float maxTilt = p.maxTiltDeg * FC_DEG2RAD;
  float yawStick = 0.0f;
  if (pilotOk && state != ST_IDLE) {
    rollSp = pilot.roll * maxTilt;
    pitchSp = pilot.pitch * maxTilt;
    yawStick = pilot.yaw;
  } else {
    rollSp = pitchSp = 0.0f;
  }

  // ---- vertical
  bool useAlt = true;
  float vzFeed = 0.0f;
  bool manualThrust = false;
  switch (state) {
    case ST_IDLE: {
      idleTimer_ += dt;
      if (idleTimer_ > p.idleDisarmS) { cmdDisarm(R_IDLE); return; }
      if (pilot.manual && pilot.manualCmd > 0.15f) { enterFlying(MODE_MANUAL); mode = MODE_MANUAL; }
      else if (!pilot.manual && pilot.climb > 0.5f) cmdTakeoff(nowUs);
      if (state == ST_IDLE) {
        for (int i = 0; i < 4; i++) { mix.cmd[i] = p.motorIdle; mix.thrust[i] = cmdToThrust(p, p.motorIdle); motor[i] = p.motorIdle; }
        collective = 0.0f;
        torque = Vec3();
        att.reset(ekf.yaw());
        return;
      }
      break;
    }
    case ST_TAKEOFF: {
      takeoffTimer_ += dt;
      if (altTarget < takeoffGoal) {
        // climb at take-off speed, braking at 0.5 m/s^2 when getting close (no overshoot)
        const float spd = fcClamp(sqrtf(2.0f * 0.5f * (takeoffGoal - altTarget)), 0.1f, p.takeoffSpeed);
        altTarget = fcMin(altTarget + spd * dt, takeoffGoal);
        vzFeed = spd;
      }
      const bool reached = altTarget >= takeoffGoal && fcAbs(alt - takeoffGoal) < 0.15f && fcAbs(vz) < 0.3f;
      const bool pilotTakes = pilot.manual || fcAbs(pilot.climb) > 0.05f;
      if (reached || pilotTakes || takeoffTimer_ > 15.0f) {
        enterFlying(pilot.manual ? MODE_MANUAL : MODE_ALT_HOLD);
        if (reached) altTarget = takeoffGoal;
      }
      break;
    }
    case ST_LANDING:
    case ST_FAILSAFE: {
      const bool hold = state == ST_FAILSAFE && reason == R_PILOT_LINK && (uint8_t)p.fsAction == FS_HOLD;
      if (hold) break;   // altitude hold at altTarget
      useAlt = false;
      vzFeed = (alt - groundAlt > 1.0f) ? -p.landSpeed : -p.landSpeedLow;
      altTarget = alt;
      // touch-down: low, not moving vertically and the thrust has dropped below hover
      const bool low = alt - groundAlt < 0.3f;
      if (low && fcAbs(vz) < 0.2f && altc.thrust < 0.95f * hoverT) landTimer_ += dt;
      else landTimer_ = 0.0f;
      if (landTimer_ > 0.8f || (!airborne && low && altc.thrust < 0.6f * hoverT)) {
        cmdDisarm(state == ST_LANDING ? R_LANDED : reason);
        return;
      }
      break;
    }
    case ST_FLYING:
    default: {
      if (mode == MODE_MANUAL) {
        manualThrust = true;
        if (alt > p.maxAlt) { enterFlying(MODE_ALT_HOLD); geofence = true; manualThrust = false; altTarget = p.maxAlt; }
      }
      if (!manualThrust) {
        const float c = pilot.climb;
        if (p.altHoldOn < 0.5f) {
          useAlt = false;
          vzFeed = c > 0.0f ? c * p.maxClimb : c * p.maxDescent;
        } else if (fcAbs(c) > 0.0f) {
          useAlt = false;
          vzFeed = c > 0.0f ? c * p.maxClimb : c * p.maxDescent;
          altTarget = alt;
          holdCapture_ = true;
        } else if (holdCapture_) {
          altTarget = alt + 0.3f * vz;      // stop smoothly: aim a little ahead
          holdCapture_ = false;
        }
        if (altTarget > p.maxAlt) { altTarget = p.maxAlt; geofence = true; }
        if (alt >= p.maxAlt && vzFeed > 0.0f) { vzFeed = 0.0f; useAlt = true; altTarget = p.maxAlt; geofence = true; }
      }
      break;
    }
  }

  // ---- controllers
  torque = att.update(p, ekf.q, ekf.rate, rollSp, pitchSp, yawStick, airborne, dt);
  if (manualThrust) {
    collective = cmdToThrust(p, pilot.manualCmd);
    altc.thrust = collective;
  } else {
    collective = altc.update(p, alt, vz, altTarget, vzFeed, useAlt, fcCosTilt(ekf.q), airborne, dt);
  }
  // until airborne, keep the attitude integrators and torques small (the drone sits on its legs)
  if (!airborne) torque = torque * 0.3f;
  mixerRun(p, collective, torque.x, torque.y, torque.z, true, mix);
  for (int i = 0; i < 4; i++) motor[i] = mix.cmd[i];
}
