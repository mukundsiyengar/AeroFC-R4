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
    // "Mag decl" is about right, and a big error has its own path below: start sure enough not to be fooled by one move
    aidP[0][0] = fcSq(0.15f); aidP[1][1] = aidP[2][2] = fcSq(0.5f);
    aidBigN_ = 0;
  }
  const double kLat = 111320.0;
  homeNorth = (float)((lat - lat0_) * kLat);
  homeEast = (float)((lon - lon0_) * kLat * (double)cosf((float)lat0_ * FC_DEG2RAD));
  if (!ekf.initialized()) return;
  const uint32_t now = lastImuUs_;
  gpsChain.push(homeNorth, homeEast, now);
  imuChain.push(imuPosN_, imuPosE_, now);
  lastGpsUs_ = now;
  // the GPS changes 10 times a second; the bridge repeats the last value 5 times. Use each NEW fix once, and when
  // the value does not change for 120 ms the drone is standing still: use it then too.
  // (an unchanged value while the drone is moving means the GPS is stuck: then no fixes, and position hold
  //  switches itself off after 0.5 s)
  const bool changed = lat != lastFixLat_ || lon != lastFixLon_;
  const bool slow = !pos.initialized() || (fcSq(pos.kn.v) + fcSq(pos.ke.v)) < 0.25f;
  if (changed || ((uint32_t)(now - lastFixUs_) >= 120000u && slow)) {
    lastFixLat_ = lat;
    lastFixLon_ = lon;
    lastFixUs_ = now;
    pos.gpsFix(homeNorth, homeEast, gpsChain.velN, gpsChain.velE);
  }

  // Heading offset between the GPS frame (true north) and the EKF frame (compass + "Mag decl").
  // Small Kalman filter, state x = [yaw offset, errN, errE]:   imuChainAcc = R(yawOffset) * gpsChainAcc + err
  // A heading offset turns the acceleration, an attitude error adds a constant, so both can be told apart as the
  // drone manoeuvres.  A wrong compass or declination is therefore corrected by itself, whatever its size.
  // The GPS only changes 10 times a second, so while the acceleration changes quickly the two chains briefly
  // disagree: learn only while the acceleration is steady.
  const float jerk = sqrtf(fcSq(gpsChain.accN - aidPrevAccN) + fcSq(gpsChain.accE - aidPrevAccE)) * 50.0f;
  aidPrevAccN = gpsChain.accN;
  aidPrevAccE = gpsChain.accE;
  const float gAcc = sqrtf(fcSq(gpsChain.accN) + fcSq(gpsChain.accE));
  // The GPS moves in 10 Hz steps, so the faster the drone flies, the noisier the acceleration measured from it (at
  // 25 m/s a step is 2.5 m): demand a stronger signal at speed, or the noise is learned as a heading offset.
  // And nothing is learned above 15 m/s or while the drone rotates fast (brake flips): then the two accelerations
  // honestly differ.  Normal flying stays below both, also with a wrong compass.
  const float gpsSpeed = sqrtf(fcSq(gpsChain.velN) + fcSq(gpsChain.velE));
  const bool canLearn = gpsSpeed < 15.0f && ekf.rate.norm() < 0.7f;
  // a real (even gentle) manoeuvre, and a clean GPS: a noisy one would teach it nonsense
  const bool plausible = canLearn && gAcc > 0.3f + 0.1f * gpsSpeed && gAcc < 6.0f && pos.gpsQuiet();
  const float gN = gpsChain.accN, gE = gpsChain.accE, iN = imuChain.accN, iE = imuChain.accE;
  // A big offset (the first manoeuvre with a wrong compass) is too far for the filter: measure the angle between
  // the two accelerations directly, and when it stays the same for 0.5 s, jump there.  Not limited to steady
  // accelerations: with a badly wrong compass position hold pushes the wrong way and the drone accelerates hard
  // (and the GPS then looks "noisy" to the position filter).  GPS noise cannot fake it: the two accelerations must
  // agree in size, and their angle must stay the same.
  const float iAcc = sqrtf(iN * iN + iE * iE);
  if (canLearn && gpsChain.ready() && imuChain.ready() && gAcc > 1.0f + 0.2f * gpsSpeed && gAcc < 12.0f &&
      fcAbs(iAcc - gAcc) < 0.3f * gAcc) {
    const float ang = fcAtan2(gN * iE - gE * iN, gN * iN + gE * iE);
    const float away = fcAbs(fcWrapPi(ang - aidYawOffset));
    // The two chains are a little apart in time (more over usbipd), so while the acceleration turns, its angle looks
    // wrong by up to 30 deg without any compass error (measured in the real Gazebo). Such a sample proves nothing:
    // it neither counts nor clears the count. Only a steady acceleration, or more than 35 deg, is evidence.
    const bool turning = jerk >= 1.5f && away <= 0.6f;
    if (away > 0.35f && !turning) {                                // more than 20 deg away from the estimate
      if (aidBigN_ > 0 && fcAbs(fcWrapPi(ang - aidBigAng_)) < 0.2f) aidBigN_++;
      else { aidBigN_ = 1; aidBigAng_ = ang; }
      aidBigAng_ = fcWrapPi(aidBigAng_ + 0.2f * fcWrapPi(ang - aidBigAng_));
      if (aidBigN_ >= 25) {
        aidYawOffset = aidBigAng_;
        aidErrN = aidErrE = 0.0f;
        for (int a = 0; a < 3; a++) for (int b = 0; b < 3; b++) aidP[a][b] = 0.0f;
        aidP[0][0] = fcSq(0.15f); aidP[1][1] = aidP[2][2] = fcSq(0.3f);
        aidBigN_ = 0;
        // everything learned with the wrong heading is wrong too: restart the position filter from the GPS,
        // forget the wind estimate, and give position hold a new chance
        pos.kn.reset(homeNorth, gpsChain.velN);
        pos.ke.reset(homeEast, gpsChain.velE);
        posCtl.iN = posCtl.iE = 0.0f;
        if (posFailed) { posFailed = false; posCtl.retry(); }
      }
    } else if (!turning) {
      aidBigN_ = 0;
    }
  }
  // turning fast turns the acceleration too, and the same time difference then reads as a small offset
  if (gpsChain.ready() && imuChain.ready() && jerk < 1.5f && plausible && aidBigN_ < 3 && fcAbs(ekf.rate.z) < 0.25f) {
    aidP[1][1] += 0.0005f; aidP[2][2] += 0.0005f;    // the attitude error changes slowly (the EKF corrects it)
    aidP[0][0] += 1e-7f;
    for (int r = 0; r < 2; r++) {
      const float co = cosf(aidYawOffset), so = sinf(aidYawOffset);
      // predicted IMU acceleration and its slope with respect to the state
      const float pred = r == 0 ? co * gN - so * gE + aidErrN : so * gN + co * gE + aidErrE;
      const float H[3] = { r == 0 ? -so * gN - co * gE : co * gN - so * gE, r == 0 ? 1.0f : 0.0f, r == 0 ? 0.0f : 1.0f };
      float PH[3], S = fcSq(0.12f);
      for (int a = 0; a < 3; a++) PH[a] = aidP[a][0] * H[0] + aidP[a][1] * H[1] + aidP[a][2] * H[2];
      for (int a = 0; a < 3; a++) S += H[a] * PH[a];
      const float innov = fcClamp((r == 0 ? iN : iE) - pred, -2.0f, 2.0f);
      float K[3];
      for (int a = 0; a < 3; a++) K[a] = PH[a] / S;
      aidYawOffset = fcWrapPi(aidYawOffset + K[0] * innov);
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
  const int LAG = 20;   // ~0.4 s at 50 packets per second (GPS itself updates at 10 Hz); long = robust to GPS noise
  if (count >= LAG) {
    const uint8_t j = (uint8_t)((i + N - LAG) % N);
    const float dt = (float)(uint32_t)(now - t[j]) * 1e-6f;
    if (dt > 0.05f && dt < 1.0f) { vn = (n - pN[j]) / dt; ve = (e - pE[j]) / dt; haveV = true; }
  }
  vN[i] = vn; vE[i] = ve;
  if (haveV) {
    const float k = 0.08f;
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
  if (thr >= 9.5f && thr < 29.5f) {                  // command code
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
    case CODE_ECO: prm->speedMode = SPEED_ECO; break;
    case CODE_NORMAL: prm->speedMode = SPEED_NORMAL; break;
    case CODE_SPORT: prm->speedMode = SPEED_SPORT; break;
    case CODE_POSHOLD_TOGGLE: prm->posHoldOn = prm->posHoldOn > 0.5f ? 0.0f : 1.0f; break;
    case CODE_STICK_TOGGLE: prm->posStick = prm->posStick > 0.5f ? (float)STICK_SPEED : (float)STICK_ANGLE; break;
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
  posCtl.reset(pos.kn.p, pos.ke.p);
  posFailed = false;
  posFailCount_ = 0;
  blendT_ = 1.0f;
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
    takeoffGoal = groundAlt + prm->takeoffAlt;
    if (prm->altLimitOn > 0.5f) takeoffGoal = fcMin(takeoffGoal, groundAlt + prm->maxAlt);
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
  descentSp_ = 0.0f;
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

  // the pilot rate is worked out when a packet arrives: with no packets for a second it is 0, not the old value
  if (pilot.rateHz && (uint32_t)(nowUs - pilot.lastUs) > 1000000u) pilot.rateHz = 0;

  // pilot link loss (only after our continuous stream was seen)
  if (pilot.streamSeen && !pilot.linkLost && (uint32_t)(nowUs - pilot.lastUs) > (uint32_t)(prm->pilotTimeout * 1e6f)) {
    pilot.linkLost = true;
    pilot.roll = pilot.pitch = pilot.yaw = pilot.climb = 0.0f;
    pilot.manual = false;
    const uint8_t act = (uint8_t)prm->fsAction;
    if ((state == ST_FLYING || state == ST_TAKEOFF) && (act == FS_LAND || act == FS_HOLD)) {
      if (act == FS_HOLD) { enterFlying(MODE_ALT_HOLD); altTarget = altf.alt; }
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
  // horizontal acceleration seen by the IMU in the EKF frame: drives the position filter, and (integrated twice)
  // the comparison chain that learns the compass/GPS heading offset
  {
    const Vec3 fe = ekf.q.rotate(sf.acc);
    imuAccN_ = fe.x;
    imuAccE_ = fe.y;
    imuVelN_ += imuAccN_ * dt; imuVelE_ += imuAccE_ * dt;
    imuPosN_ += imuVelN_ * dt; imuPosE_ += imuVelE_ * dt;
    const float co = cosf(aidYawOffset), so = sinf(aidYawOffset);
    pos.predict(co * imuAccN_ + so * imuAccE_, -so * imuAccN_ + co * imuAccE_, dt);
  }
  // GPS-aided attitude: the position filter compares the IMU with the GPS and keeps an estimate of the IMU's
  // acceleration error (its bias state).  Most of that error is a small attitude error (1 deg = 0.17 m/s^2), so
  // "IMU acceleration - error" is the true acceleration, and the attitude is corrected towards it.  The filter has
  // already averaged the GPS, so this works with Gazebo's clean GPS and with a noisy one.
  // It waits only while a wrong heading offset is being measured (then the error is not an attitude error).
  gpsAidOk = p.gpsAidOn > 0.5f && pos.initialized() && lastFixUs_ && (uint32_t)(nowUs - lastFixUs_) < 500000u &&
             pos.sigma() < 1.0f && aidBigN_ < 3;
  if (newImu) {
    if (!airborne) {
      ekf.updateAccel(sf.acc, 0.5f, 0.15f);                                   // on the ground: trust it
    } else if (gpsAidOk) {
      const float co = cosf(aidYawOffset), so = sinf(aidYawOffset);
      float bN = co * pos.kn.b - so * pos.ke.b, bE = so * pos.kn.b + co * pos.ke.b;         // GPS -> EKF frame
      // a large error is corrected at a limited rate, never switched off: a drifting gyro needs it most then
      const float bMag = sqrtf(bN * bN + bE * bE);
      if (bMag > 3.0f) { bN *= 3.0f / bMag; bE *= 3.0f / bMag; }
      const Vec3 ref(imuAccN_ - bN, imuAccE_ - bE, -FC_GRAVITY);
      // a clean GPS makes this reference better than the plain "assume level" below, so it is trusted more
      if (ekf.updateAccel(sf.acc, p.ekfAccNoiseFlight * (pos.gpsQuiet() ? 0.33f : 1.0f), 0.1f, &ref)) {
        // the error has now moved from the bias into the attitude: take the same amount out of the bias,
        // so it is not corrected twice
        const Vec3 fe2 = ekf.q.rotate(sf.acc);
        const float dN = fe2.x - imuAccN_, dE = fe2.y - imuAccE_;
        if (fcFinite(dN) && fcFinite(dE)) {
          pos.kn.b = fcClamp(pos.kn.b + co * dN + so * dE, -3.0f, 3.0f);
          pos.ke.b = fcClamp(pos.ke.b - so * dN + co * dE, -3.0f, 3.0f);
        }
      }
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
  const SpeedProfile &prof = speedProfile(p);
  const float alt = altf.alt, vz = altf.vz;
  const float hoverT = hoverThrustFromCmd(p);

  // ---- position hold gave up: this low-drag drone would slide away for ever, so try again after 5 s (brake, then
  // hold where it stops). After 3 failures in a row it stays off until the next arm.
  if (posFailed && posFailCount_ < 3) {
    posFailT_ += dt;
    if (posFailT_ >= 5.0f) { posFailed = false; posCtl.retry(); }
  }

  // ---- position hold readiness: GPS fresh, estimator converged, heading trustworthy
  const bool gpsFresh = gpsHome && lastFixUs_ && (uint32_t)(nowUs - lastFixUs_) < 500000u && pos.initialized();
  const bool headingOk = (p.compassOn > 0.5f && ekf.magHealthy) || (ekf.initialized() && ekf.yawSigma() < 10.0f * FC_DEG2RAD && p.compassOn > 0.5f);
  if (!gpsFresh) posWhy = "NO GPS";
  else if (pos.sigma() > 1.0f) posWhy = "GPS SETTLING";
  else if (!headingOk) posWhy = "NO COMPASS";
  else if (posFailed) posWhy = "FAILED";
  else posWhy = "";
  posReady = posWhy[0] == 0;

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
    posActive = false;
    posState = posReady ? (p.posHoldOn > 0.5f ? POS_HOLDING : POS_OFF) : POS_NOT_READY;
    if (pos.initialized()) posCtl.reset(pos.kn.p, pos.ke.p);
    ceilingAbs = (p.altLimitOn > 0.5f) ? alt + p.maxAlt : 1e9f;
    atCeiling = false;
    return;
  }
  ceilingAbs = (p.altLimitOn > 0.5f) ? groundAlt + p.maxAlt : 1e9f;

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

  // ---- attitude setpoints: position hold, or the sticks as angles
  const bool pilotOk = !(state == ST_FAILSAFE) && !pilot.linkLost;
  const float maxTilt = prof.maxTiltDeg * FC_DEG2RAD;
  float yawStick = 0.0f;
  const bool flyingState = state == ST_TAKEOFF || state == ST_FLYING || state == ST_LANDING || state == ST_FAILSAFE;
  // position hold flies when switched on - and ALWAYS during a failsafe, so a lost link lands on the spot
  posActive = posReady && flyingState && (p.posHoldOn > 0.5f || state == ST_FAILSAFE);
  if (pilotOk && state != ST_IDLE) yawStick = pilot.yaw;
  if (posActive) {
    if (!wasPosActive_) posCtl.engage();
    PosCtlIn in;
    in.pN = pos.kn.p; in.pE = pos.ke.p; in.vN = pos.kn.v; in.vE = pos.ke.v;
    in.yaw = ekf.yaw();
    in.yawOffset = aidYawOffset;
    in.stickFwd = -pilot.pitch;     // pitch stick - = forward
    in.stickRight = pilot.roll;
    in.sticksAllowed = pilotOk && state != ST_IDLE;
    in.airborne = airborne;
    in.heightAgl = alt - groundAlt;
    in.touchdown = landTimer_ > 0.0f;
    in.stickStyle = (uint8_t)(p.posStick + 0.5f);
    posCtl.update(p, prof, in, dt, rollSp, pitchSp);
    if (posCtl.failed) { posFailed = true; posFailCount_++; posFailT_ = 0.0f; }
    posGoodT_ = (posCtl.state == POS_HOLDING && posCtl.dist < 2.0f) ? posGoodT_ + dt : 0.0f;
    if (posGoodT_ > 30.0f) posFailCount_ = 0;           // holding well again: forget the earlier failures
    posState = posCtl.state;
  } else {
    if (pilotOk && state != ST_IDLE) {
      rollSp = pilot.roll * maxTilt;
      pitchSp = pilot.pitch * maxTilt;
    } else {
      rollSp = pitchSp = 0.0f;
    }
    posState = posFailed ? POS_FAILED : (posReady ? POS_OFF : POS_NOT_READY);
  }
  // switching between position hold and stick angles: move from the old tilt to the new one over 0.7 s
  if (posActive != wasPosActive_ && armed()) blendT_ = 0.0f;
  if (blendT_ < 1.0f) {
    blendT_ = fcMin(1.0f, blendT_ + dt / 0.7f);
    rollSp = blendRoll_ + (rollSp - blendRoll_) * blendT_;
    pitchSp = blendPitch_ + (pitchSp - blendPitch_) * blendT_;
  }
  blendRoll_ = rollSp;
  blendPitch_ = pitchSp;
  wasPosActive_ = posActive;

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
      {
        const float h = alt - groundAlt;
        vzFeed = h > 20.0f ? -p.landSpeedHigh : (h > 1.0f ? -p.landSpeed : -p.landSpeedLow);
      }
      altTarget = alt;
      // touch-down: low, not moving vertically and the thrust has dropped below hover
      const bool low = alt - groundAlt < 0.3f;
      if (low && fcAbs(vz) < 0.2f && altc.thrust < 0.95f * hoverT) landTimer_ += dt;
      else landTimer_ = 0.0f;
      if (landTimer_ > 0.8f || (!airborne && low && altc.thrust < 0.6f * hoverT)) {
        cmdDisarm(state == ST_LANDING ? (uint8_t)R_LANDED : reason);
        return;
      }
      break;
    }
    case ST_FLYING:
    default: {
      if (mode == MODE_MANUAL) {
        manualThrust = true;
        if (alt > ceilingAbs) { enterFlying(MODE_ALT_HOLD); geofence = true; manualThrust = false; altTarget = ceilingAbs; }
      }
      if (!manualThrust) {
        const float c = pilot.climb;
        // going down by stick: the ground comes up gently whatever the stick says (0.6 m/s per metre of height, down
        // to the slow landing speed), and the speed builds up gradually, so the motors ease off instead of plunging
        const float wantDown = c < 0.0f ? fcMin(-c * prof.descent, fcMax(p.landSpeedLow, 0.6f * (alt - groundAlt))) : 0.0f;
        descentSp_ = wantDown > descentSp_ ? fcMin(wantDown, descentSp_ + 1.5f * dt) : wantDown;
        const float stickVz = c > 0.0f ? c * prof.climb : -descentSp_;
        // stick held down and sitting on the ground: the same touch-down test as a landing, then the motors stop
        const bool low = alt - groundAlt < 0.3f;
        if (c < -0.3f && low && fcAbs(vz) < 0.2f && altc.thrust < 0.95f * hoverT) landTimer_ += dt;
        else landTimer_ = 0.0f;
        if (landTimer_ > 0.8f) { cmdDisarm(R_LANDED); return; }
        if (p.altHoldOn < 0.5f) {
          useAlt = false;
          vzFeed = stickVz;
        } else if (fcAbs(c) > 0.0f) {
          useAlt = false;
          vzFeed = stickVz;
          altTarget = alt;
          holdCapture_ = true;
        } else if (holdCapture_) {
          altTarget = alt + 0.3f * vz;      // stop smoothly: aim a little ahead
          holdCapture_ = false;
        }
        // height ceiling (Alt limit): never aim above it, stop climbing at it, come down if it was lowered
        if (altTarget > ceilingAbs) { altTarget = ceilingAbs; geofence = true; }
        if (vzFeed > 0.0f && p.altLimitOn > 0.5f)       // brake at 1.5 m/s^2 so it stops exactly at the ceiling
          vzFeed = fcMin(vzFeed, sqrtf(2.0f * 1.0f * fcMax(ceilingAbs - 0.1f - alt, 0.0f)));
        if (alt >= ceilingAbs - 0.05f && vzFeed > 0.0f) { vzFeed = 0.0f; useAlt = true; altTarget = ceilingAbs; geofence = true; }
        if (!useAlt && alt > ceilingAbs) { useAlt = true; altTarget = ceilingAbs; }
      }
      break;
    }
  }

  // ---- controllers
  atCeiling = p.altLimitOn > 0.5f && alt >= ceilingAbs - 0.3f;
  torque = att.update(p, ekf.q, ekf.rate, rollSp, pitchSp, yawStick, prof.yawRateDps * FC_DEG2RAD, airborne, dt);
  if (manualThrust) {
    collective = cmdToThrust(p, pilot.manualCmd);
    altc.thrust = collective;
  } else {
    const float maxDown = (state == ST_LANDING || state == ST_FAILSAFE) ? fcMax(prof.descent, p.landSpeedHigh) : prof.descent;
    collective = altc.update(p, alt, vz, altTarget, vzFeed, useAlt, fcCosTilt(ekf.q), airborne, dt,
                             fcMax(prof.climb, p.takeoffSpeed), maxDown);
  }
  // until airborne, keep the attitude integrators and torques small (the drone sits on its legs)
  if (!airborne) torque = torque * 0.3f;
  mixerRun(p, collective, torque.x, torque.y, torque.z, true, mix);
  for (int i = 0; i < 4; i++) motor[i] = mix.cmd[i];
}
