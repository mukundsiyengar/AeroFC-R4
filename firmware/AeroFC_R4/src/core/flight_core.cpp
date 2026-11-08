#include "flight_core.h"

static const float ARM_MAX_TILT_DEG = 15.0f;
static const uint32_t SENSOR_STALE_US = 300000;     // 0.3 s without IMU -> sensors not OK
static const uint32_t SENSOR_LOST_US = 1500000;     // 1.5 s without IMU while armed -> disarm
static const uint16_t PILOT_MIN_HZ = 5;             // fewer pilot packets a second than this ...
static const uint8_t PILOT_SLOW_WINDOWS = 2;        // ... for this many seconds in a row counts as a lost link
static const float STALE_MIN_SPEED = 0.3f;          // m/s: slower than this, a repeated IMU reading can be a drone at rest
static const float IMU_STUCK_S = 1.5f;              // the same as SENSOR_LOST_US: a stuck IMU is no IMU
static const float IMU_STUCK_THRUST = 0.002f;       // this much change of thrust would show on a working accelerometer
static const float MAG_STUCK_TURN = 15.0f * FC_DEG2RAD;   // the gyro says it has turned this far, and the compass hasn't changed by a single bit
static const float MAG_FAR_OFF = 10.0f * FC_DEG2RAD;      // the compass disagrees by this much with where the gyro has kept the heading
static const float POS_RETRY_S = 2.0f;              // position hold gave up: try again after this long
static const uint8_t AID_BIG_FORGET = 25;           // GPS packets (0.5 s) with no evidence of a big heading offset: forget what there was

const char *stateName(uint8_t s) {
  static const char *const n[] = { "DISARMED", "IDLE", "TAKEOFF", "FLYING", "LANDING", "FAILSAFE", "AUTO" };
  return s < 7 ? n[s] : "?";
}
const char *reasonName(uint8_t r) {
  static const char *const n[] = { "-", "user", "pilot link", "sensors", "estimator", "CRASH", "STOP",
                                   "KILL", "idle", "landed", "geofence", "no navigation", "held" };
  return r < 13 ? n[r] : "?";
}
const char *armResultName(int8_t a) {
  static const char *const n[] = { "OK", "already", "no sensors", "EKF not ready", "not level", "calibrating",
                                   "too soon", "bridge too slow" };
  return (a >= 0 && a < 8) ? n[a] : "?";
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
    } else {
      imuTimeBad++;
    }
    dt = fcClamp(dt, 0.002f, 0.1f);
  }
  // Exactly the same values while moving: the simulator stalled, and the bridge repeats its last reading. A real sensor
  // never does this. Nothing has moved since, so the estimators must not integrate it again: that made the speed estimate
  // fall too fast while braking, and the drone coasted for metres. Sitting still in a noise-free simulator the values
  // really do repeat, and the estimators must carry on then, or touch-down is never noticed.
  const bool moving = pos.initialized() && (fcSq(pos.kn.v) + fcSq(pos.ke.v)) > fcSq(STALE_MIN_SPEED);
  const bool repeated = haveImu_ && airborne && r.ax == lastRaw_.ax && r.ay == lastRaw_.ay && r.az == lastRaw_.az &&
                        r.gx == lastRaw_.gx && r.gy == lastRaw_.gy && r.gz == lastRaw_.gz;
  imuStale_ = repeated && moving;
  imuRepeated_ = repeated;
  if (imuStale_) imuStaleCount++;
  // The IMU repeats itself while the barometer, compass or GPS go on changing: the world is running, so the IMU has died.
  // Without it nothing can keep the drone level, so it counts as lost. If everything stops together, the simulator has only
  // stalled or been paused, and the drone can carry on when it does.
  // In a noise-free simulator the values really can repeat: on the ground, or at a steady manual throttle in the air. What
  // can't happen to a working IMU is the same reading exactly while the motors change, so that has to be seen too.
  if (!repeated) {
    imuStuckT_ = 0.0f;
  } else {
    if (imuStuckT_ == 0.0f) {                       // what counts is what changes from here on
      worldMoved_ = false;
      stuckThrustLo_ = stuckThrustHi_ = collective;
    }
    imuStuckT_ += dt;
    stuckThrustLo_ = fcMin(stuckThrustLo_, collective);
    stuckThrustHi_ = fcMax(stuckThrustHi_, collective);
  }
  if (imuStuckT_ > IMU_STUCK_S && worldMoved_ && stuckThrustHi_ - stuckThrustLo_ > IMU_STUCK_THRUST && altf.alt - groundAlt > 0.3f) cmdDisarm(R_SENSORS);
  lastRaw_ = r;
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
  if (altM != lastBaroRaw_) { lastBaroRaw_ = altM; worldMoved_ = true; }
  sens.onBaro(altM, sf);
  // on the ground the height filter follows the barometer closely: arming takes its height as the ground, and straight after
  // a hard landing it is still off by 0.1 m, which then stays in every height of the flight
  if (altInit_) altf.updateBaro(sf.baroAlt, armed() ? prm->baroNoise : prm->baroNoise * 0.1f);
}

void FlightCore::onMag(float x, float y, float z) {
  if (x != lastMagRaw_[0] || y != lastMagRaw_[1] || z != lastMagRaw_[2]) {
    lastMagRaw_[0] = x; lastMagRaw_[1] = y; lastMagRaw_[2] = z;
    worldMoved_ = true;
    magTurn_ = 0.0f;
    magGyroZero_ = ekf.bias.z;
  }
  sens.onMag(x, y, z, sf);
  if (!ekf.initialized() || prm->compassOn < 0.5f) return;
  // A compass that has stuck says the drone isn't turning, so the filter takes every real turn for gyro drift. The heading stays
  // put while the drone really spins, faster and faster. No working compass reads exactly the same after a turn like this, so
  // ignore it, undo what it taught the filter, and carry on with the gyro until it changes again.
  const bool stuck = fcAbs(magTurn_) > MAG_STUCK_TURN;
  if (stuck) {
    if (!magStuck) ekf.bias.z = magGyroZero_;
    magStuck = true;
    ekf.magHealthy = false;
    return;
  }
  magStuck = false;
  const float yawBefore = ekf.yaw();
  // The filter's heading can end up far from a compass that is right again, and then it refuses every reading. In the air that
  // is the safe choice. On the ground the drone can't have been turned by a bad gyro any more: nothing else can settle it.
  if (!armed() && ekf.magDisagrees()) {
    ekf.alignYawToMag(sf.mag, prm->magDeclDeg * FC_DEG2RAD);
    aidYawOffset = 0.0f;                     // if the compass is wrong after all, the first move in the air shows it, as on any first flight
    aidErrN = aidErrE = 0.0f;
    return;
  }
  ekf.updateMag(sf.mag, prm->magDeclDeg * FC_DEG2RAD, 0.09f);
  // A compass that has been wrong and comes right again moves the filter's heading, though the drone hasn't turned. Position hold
  // works with heading minus offset, so the offset moves with it, or what was learned while it was wrong would now be wrong.
  if (fcAbs(ekf.lastMagInnov) > MAG_FAR_OFF) aidYawOffset = fcWrapPi(aidYawOffset + fcWrapPi(ekf.yaw() - yawBefore));
}

void FlightCore::onGps(double lat, double lon, float alt) {
  if (lat != lastLatRaw_ || lon != lastLonRaw_) { lastLatRaw_ = lat; lastLonRaw_ = lon; worldMoved_ = true; }
  sens.onGps(lat, lon, alt, sf);
  if (lat == 0.0 && lon == 0.0) return;
  if (!gpsHome) {
    lat0_ = lat; lon0_ = lon; gpsHome = true; gpsChain.reset(); imuChain.reset();
    // Metres per degree here, on the WGS84 ellipsoid as Gazebo's NavSat uses. A round 111320 m is 0.6 % too many going north at
    // this latitude: nothing for position hold, which only compares a place with itself, but 0.25 m on a 40 m trip.
    const float sinLat = sinf((float)lat * FC_DEG2RAD), cosLat = cosf((float)lat * FC_DEG2RAD);
    const double WGS84_A = 6378137.0, WGS84_E2 = 0.00669437999;
    const double w = 1.0 - WGS84_E2 * (double)sinLat * (double)sinLat;
    const double rootW = (double)sqrtf((float)w);
    mPerDegLat_ = (double)FC_DEG2RAD * WGS84_A * (1.0 - WGS84_E2) / (w * rootW);
    mPerDegLon_ = (double)FC_DEG2RAD * WGS84_A * (double)cosLat / rootW;
    aidYawOffset = 0.0f;
    aidErrN = aidErrE = 0.0f;
    for (int a = 0; a < 3; a++) for (int b = 0; b < 3; b++) aidP[a][b] = 0.0f;
    // "Mag decl" is about right, and a big error has its own path below: start sure enough not to be fooled by one move
    aidP[0][0] = fcSq(0.15f); aidP[1][1] = aidP[2][2] = fcSq(0.5f);
    aidBigN_ = 0;
    gpsShiftN_ = gpsShiftE_ = 0.0f;
  }
  homeNorth = (float)((lat - lat0_) * mPerDegLat_) - gpsShiftN_;
  homeEast = (float)((lon - lon0_) * mPerDegLon_) - gpsShiftE_;
  if (!ekf.initialized()) return;
  const uint32_t now = lastImuUs_;
  lastGpsUs_ = now;
  // the GPS changes 10 times a second; the bridge repeats the last value 5 times. Use each NEW fix once, and when
  // the value does not change for 120 ms the drone is standing still: use it then too.
  // (an unchanged value while the drone is moving means the GPS is stuck: then no fixes, and position hold
  //  switches itself off after 0.5 s)
  // On the ground it isn't moving, whatever the estimate says: the fix is what stops an estimate that has run away.
  const bool changed = lat != lastFixLat_ || lon != lastFixLon_;
  const bool slow = !airborne || !pos.initialized() || (fcSq(pos.kn.v) + fcSq(pos.ke.v)) < 0.25f;
  if (!changed && (uint32_t)(now - lastFixUs_) > 500000u) gpsWasStuck_ = true;
  if (changed && gpsWasStuck_) { gpsWasStuck_ = false; pos.gpsBack = true; }
  if (changed || ((uint32_t)(now - lastFixUs_) >= 120000u && slow)) {
    lastFixLat_ = lat;
    lastFixLon_ = lon;
    lastFixUs_ = now;
    pos.gpsFix(homeNorth, homeEast, gpsChain.velN, gpsChain.velE);
    if (pos.shiftN != 0.0f || pos.shiftE != 0.0f) {
      gpsShiftN_ += pos.shiftN; gpsShiftE_ += pos.shiftE;
      homeNorth -= pos.shiftN; homeEast -= pos.shiftE;
      pos.shiftN = pos.shiftE = 0.0f;
    }
  }
  // Nothing about a jump is true: it would read as a huge speed and acceleration, here and in what is learned from them below.
  if (pos.jumping()) return;
  gpsChain.push(homeNorth, homeEast, now);
  imuChain.push(imuPosN_, imuPosE_, now);

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
  // Evidence of a big offset that stops coming proves nothing. Hovering there is none either way, and while this count stayed up the
  // attitude got no help from the GPS, so a small gyro drift tilted the estimate by 19 degrees.
  if (aidBigAge_ < AID_BIG_FORGET) aidBigAge_++;
  else aidBigN_ = 0;
  if (canLearn && gpsChain.ready() && imuChain.ready() && gAcc > 1.0f + 0.2f * gpsSpeed && gAcc < 12.0f &&
      fcAbs(iAcc - gAcc) < 0.3f * gAcc) {
    const float ang = fcAtan2(gN * iE - gE * iN, gN * iN + gE * iE);
    const float away = fcAbs(fcWrapPi(ang - aidYawOffset));
    // The two chains are a little apart in time (more over usbipd), so while the acceleration turns, its angle looks
    // wrong by up to 30 deg without any compass error (measured in the real Gazebo). Such a sample proves nothing:
    // it neither counts nor clears the count. Only a steady acceleration, or more than 35 deg, is evidence.
    const bool turning = jerk >= 1.5f && away <= 0.6f;
    if (away > 0.35f && !turning) {                                // more than 20 deg away from the estimate
      aidBigAge_ = 0;
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
  // Values that aren't numbers: as if nothing came, so a broken transmitter counts as a lost link.
  if (!fcFinite(thr) || !fcFinite(r) || !fcFinite(pt) || !fcFinite(y)) {
    pilot.badPackets++;
    return;
  }
  // These carry numbers, not sticks: a target would be read as full stick. They can come from a tool beside the pilot, so
  // they say nothing about the pilot link either.
  if (thr >= 22.5f && thr < 23.5f) { cmdGoto(r, pt, y, nowUs); return; }
  if (thr >= 23.5f && thr < 24.5f) {
    if (y > 0.5f && pos.initialized()) nav.setMarker(pos.kn.p + r, pos.ke.p + pt);
    return;
  }
  if (thr >= 24.5f && thr < 25.5f) {
    const int number = (int)(y / 1000.0f);
    nav.setWaypoint(number, r, pt, y - 1000.0f * (float)number);
    return;
  }
  pilot.rawThr = thr; pilot.rawRoll = r; pilot.rawPitch = pt; pilot.rawYaw = y;
  // A command says nothing about the pilot link: a tool can send one with the stream gone, so it neither keeps the link
  // alive nor brings it back.
  const bool isCode = thr >= 9.5f && thr < 29.5f;
  if (!isCode) pilot.lastUs = nowUs;
  pilot.everReceived = true;

  // shape the sticks
  float sticks[3] = { fcClamp(r, -1.0f, 1.0f), fcClamp(pt, -1.0f, 1.0f), fcClamp(y, -1.0f, 1.0f) };
  // The organisers' keyboard tool labels W "forward" but sends +pitch, which their own protocol calls backward. Nothing
  // else moves pitch in steps of exactly 0.1, so those steps are flipped: W goes forward, and a command such as
  // --pitch -0.2 or a joystick still follows the protocol. Our page's stream never gets here.
  if ((thr >= 1.5f && thr <= 3.5f) || (thr >= 9.5f && thr < 29.5f)) {
    pilot.keyTool = false;
  } else {
    const float step = fcAbs(pt - pilot.lastStockPitch);
    if (fcAbs(step - 0.1f) < 0.002f) pilot.keyTool = true;
    else if (step > 0.002f && fcAbs(pt) > 0.002f) pilot.keyTool = false;
  }
  pilot.lastStockPitch = pt;
  if (pilot.keyTool) sticks[1] = -sticks[1];
  if (prm->invRoll > 0.5f) sticks[0] = -sticks[0];
  if (prm->invPitch > 0.5f) sticks[1] = -sticks[1];
  if (prm->invYaw > 0.5f) sticks[2] = -sticks[2];
  for (int i = 0; i < 3; i++) sticks[i] = fcExpo(fcDeadband(sticks[i], prm->deadband), prm->expo);
  pilot.roll = sticks[0];
  pilot.pitch = sticks[1];
  pilot.yaw = sticks[2];

  bool extended = false;
  if (isCode) {                                       // command code
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
      // after a landing, more throttle (key u in the organisers' tool) takes off again: that tool has no other way
      if (thr > pilot.lastStockThr + 0.001f && state == ST_DISARMED && reason == R_LANDED) lastArmResult = cmdTakeoff(nowUs);
    } else {                                          // 0 = altitude hold (stock semantics)
      pilot.manual = false;
      pilot.climb = 0.0f;
    }
    pilot.lastStockThr = thr;
  }

  // pilot-link monitoring only for a continuous stream of OUR packets (the stock tool sends only on key presses)
  if (!extended) {
    pilot.streamSeen = false;   // stock tool (sends only on key presses): never treat its silence as link loss
    pilot.winCount = 0;
    pilot.slowWindows = 0;
  } else if (!isCode) {
    if ((uint32_t)(nowUs - pilot.winStartUs) >= 1000000u) {
      pilot.rateHz = (uint8_t)fcMin((float)pilot.winCount, 255.0f);
      pilot.slowWindows = (pilot.streamSeen && pilot.winCount < PILOT_MIN_HZ) ? (uint8_t)fcMin((float)pilot.slowWindows + 1.0f, 255.0f) : 0;
      if (pilot.winCount >= 10) pilot.streamSeen = true;
      pilot.winCount = 0;
      pilot.winStartUs = nowUs;
    }
    pilot.winCount++;
  }
  if (pilot.linkLost && !isCode && (!pilot.lostSlow || pilot.slowWindows == 0)) {
    pilot.linkLost = false;
    // flying home carries on until the pilot really moves a stick (flightLogic), so a short reconnect can't grab control
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
    case CODE_RTH: cmdRth(R_NONE); break;
    case CODE_MISSION_START: cmdMission(nowUs); break;
    case CODE_MISSION_CLEAR: nav.clearMission(); break;
    case CODE_NAV_CANCEL: cmdNavCancel(); break;
    case CODE_MARKER_LAND: cmdMarkerLand(); break;
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
  // The board keeps running from one bridge to the next, and how the last bridge's pilot stream ended says nothing about this one.
  pilot.streamSeen = pilot.linkLost = pilot.lostSlow = false;
  pilot.winCount = 0;
  pilot.slowWindows = 0;
  pilot.badPackets = 0;
  pilot.lastCode = 0;
  // The sensors have stopped, so this may be a restarted Gazebo, with the drone back at its start and facing another way.
  // What the last flight taught the estimators no longer applies: start them as at power-up. Otherwise the compass looks
  // wrong and position hold stays NOT READY until the first move.
  if (haveImu_ && (uint32_t)(nowUs - lastImuUs_) > SENSOR_LOST_US) {
    resetEstimator();
    pos.reset();
    gpsHome = false;
  }
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
  if (linkTooSlow()) return ARM_LINK_SLOW;
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
  nav.stop();
  rthHomeSet = pos.initialized() && !pos.rejects;   // no GPS yet, or it disagrees: flightLogic records it as soon as that is over
  rthHomeN = pos.kn.p;
  rthHomeE = pos.ke.p;
  idleTimer_ = landTimer_ = crashTimer_ = 0.0f;
  return ARM_OK;
}

void FlightCore::cmdDisarm(uint8_t why) {
  if (armed()) reason = why;
  if (why == R_CRASH) crashed = true;
  state = ST_DISARMED;
  nav.stop();
  pendingNav_ = 0;
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
  if (state == ST_TAKEOFF || state == ST_FLYING || state == ST_NAV) {
    nav.stop();
    pendingNav_ = 0;
    state = ST_LANDING;
    landTimer_ = 0.0f;
  }
}

NavIn FlightCore::navInput(float dt) const {
  NavIn in;
  in.pN = pos.kn.p; in.pE = pos.ke.p; in.vN = pos.kn.v; in.vE = pos.ke.v;
  in.heading = fcWrapPi(ekf.yaw() - aidYawOffset);      // the position filter works in the GPS frame
  in.height = altf.alt - groundAlt;
  in.vz = altf.vz;
  in.dt = dt;
  return in;
}

bool FlightCore::cmdRth(uint8_t why) {
  if (!airborne || !posReady || !rthHomeSet) return false;
  if (state != ST_FLYING && state != ST_TAKEOFF && state != ST_NAV) return false;
  pendingNav_ = 0;
  if (state != ST_NAV || nav.kind != NAV_RTH) nav.goHome(rthHomeN, rthHomeE, *prm, navInput(0.0f));
  state = ST_NAV;
  mode = MODE_ALT_HOLD;
  pilot.manual = false;
  reason = why;
  heldT_ = 0.0f;
  return true;
}

void FlightCore::cmdGoto(float north, float east, float height, uint32_t nowUs) {
  if (state == ST_LANDING || state == ST_FAILSAFE) return;
  if (fcAbs(north) > 5000.0f || fcAbs(east) > 5000.0f) return;
  pendingNav_ = 1;
  pendingN_ = north; pendingE_ = east; pendingH_ = height;
  if (!airborne) {
    lastArmResult = cmdTakeoff(nowUs);
    if (lastArmResult != ARM_OK) pendingNav_ = 0;      // it can't take off now: don't set off later as a surprise
  } else if (state == ST_FLYING || state == ST_NAV) {
    startPendingNav();
  }
}

void FlightCore::cmdMission(uint32_t nowUs) {
  if (!nav.wpCount || state == ST_LANDING || state == ST_FAILSAFE) return;
  pendingNav_ = 2;
  if (!airborne) {
    lastArmResult = cmdTakeoff(nowUs);
    if (lastArmResult != ARM_OK) pendingNav_ = 0;
  } else if (state == ST_FLYING || state == ST_NAV) {
    startPendingNav();
  }
}

void FlightCore::startPendingNav() {
  const uint8_t what = pendingNav_;
  pendingNav_ = 0;
  if (!posReady || !rthHomeSet || !what) return;
  const NavIn now = navInput(0.0f);
  if (what == 1) {
    const float h = pendingH_ > 0.0f ? fcClamp(pendingH_, 0.5f, 1000.0f) : now.height;
    nav.goTo(rthHomeN + pendingN_, rthHomeE + pendingE_, h, now);
  } else if (!nav.startMission(rthHomeN, rthHomeE, now)) {
    return;
  }
  state = ST_NAV;
  mode = MODE_ALT_HOLD;
  pilot.manual = false;
  reason = R_NONE;
}

bool FlightCore::cmdMarkerLand() {
  if (!airborne || !posReady || (state != ST_FLYING && state != ST_NAV)) return false;
  if (!nav.goMarker(navInput(0.0f))) return false;
  pendingNav_ = 0;
  state = ST_NAV;
  mode = MODE_ALT_HOLD;
  pilot.manual = false;
  reason = R_NONE;
  return true;
}

void FlightCore::cmdNavCancel() {
  pendingNav_ = 0;
  if (state != ST_NAV) return;
  nav.stop();
  reason = R_NONE;
  enterFlying(MODE_ALT_HOLD);
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
    const bool enough = imuHz >= 9.0f;
    goodWindows_ = enough ? (uint8_t)fcMin((float)goodWindows_ + 1.0f, 255.0f) : 0;
    // a slow link has windows with no packet at all, so zero counts as slow too, until the data has really stopped
    if (enough || !haveImu_ || (uint32_t)(nowUs - lastImuUs_) > 3000000u) slowWindows_ = 0;
    else if (slowWindows_ < 255) slowWindows_++;
  }
  const uint32_t sinceImu = haveImu_ ? (uint32_t)(nowUs - lastImuUs_) : 0xFFFFFFFFu;
  if (sinceImu > SENSOR_STALE_US) sensorsOk = false;
  if (armed() && sinceImu > SENSOR_LOST_US) cmdDisarm(R_SENSORS);

  // the pilot rate is worked out when a packet arrives: with no packets for a second it is 0, not the old value
  if (pilot.rateHz && (uint32_t)(nowUs - pilot.lastUs) > 1000000u) pilot.rateHz = 0;

  // pilot link loss (only after our continuous stream was seen): the packets stopped, or too few still come
  const bool silent = (uint32_t)(nowUs - pilot.lastUs) > (uint32_t)(prm->pilotTimeout * 1e6f);
  const bool tooFew = pilot.slowWindows >= PILOT_SLOW_WINDOWS;
  if (pilot.streamSeen && !pilot.linkLost && (silent || tooFew)) {
    pilot.linkLost = true;
    pilot.lostSlow = !silent;
    pilot.roll = pilot.pitch = pilot.yaw = pilot.climb = 0.0f;
    pilot.manual = false;
    const uint8_t act = (uint8_t)prm->fsAction;
    const bool flying = state == ST_FLYING || state == ST_TAKEOFF || state == ST_NAV;
    if (act == FS_RTH && flying && cmdRth(R_PILOT_LINK)) {
      // on its way home
    } else if (act == FS_RTH && flying) {    // it can't navigate (no GPS, or not off the ground yet): land here
      nav.stop();
      state = ST_FAILSAFE;
      reason = R_PILOT_LINK;
      landTimer_ = 0.0f;
    } else if (flying && act != FS_NONE) {   // LAND or HOLD, also if a trip was running
      nav.stop();
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
  nav.markerClock(dt);
  const Params &p = *prm;

  // ---- attitude estimate
  if (!imuStale_) ekf.predict(sf.gyro, dt, p.ekfGyroNoise);
  if (!airborne) magTurn_ = 0.0f;           // sitting still in a noise-free simulator a compass really does repeat itself
  else if (!imuRepeated_) magTurn_ += (sf.gyro.z - magGyroZero_) * dt;
  const float tiltCmd = fcMax(fcAbs(rollSp), fcAbs(pitchSp));
  // horizontal acceleration seen by the IMU in the EKF frame: drives the position filter, and (integrated twice)
  // the comparison chain that learns the compass/GPS heading offset
  {
    const Vec3 fe = ekf.q.rotate(sf.acc);
    imuAccN_ = fe.x;
    imuAccE_ = fe.y;
    if (!imuStale_) {
      imuVelN_ += imuAccN_ * dt; imuVelE_ += imuAccE_ * dt;
      imuPosN_ += imuVelN_ * dt; imuPosE_ += imuVelE_ * dt;
      const float co = cosf(aidYawOffset), so = sinf(aidYawOffset);
      pos.predict(co * imuAccN_ + so * imuAccE_, -so * imuAccN_ + co * imuAccE_, dt);
    }
  }
  // GPS-aided attitude: the position filter compares the IMU with the GPS and keeps an estimate of the IMU's
  // acceleration error (its bias state).  Most of that error is a small attitude error (1 deg = 0.17 m/s^2), so
  // "IMU acceleration - error" is the true acceleration, and the attitude is corrected towards it.  The filter has
  // already averaged the GPS, so this works with Gazebo's clean GPS and with a noisy one.
  // It waits only while a wrong heading offset is being measured (then the error is not an attitude error).
  gpsAidOk = p.gpsAidOn > 0.5f && pos.initialized() && lastFixUs_ && (uint32_t)(nowUs - lastFixUs_) < 500000u &&
             pos.sigma() < 1.0f && aidBigN_ < 3;
  if (newImu && !imuStale_) {
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
  if (!imuStale_) altf.predict(aUp, dt, 0.5f, 0.01f);

  flightLogic(dt, nowUs);
  motorsReady = true;
}

void FlightCore::flightLogic(float dt, uint32_t nowUs) {
  const Params &p = *prm;
  const SpeedProfile &prof = speedProfile(p);
  const float alt = altf.alt, vz = altf.vz;
  const float hoverT = hoverThrustFromCmd(p);

  // ---- position hold gave up: this low-drag drone would slide away for ever, so try again soon (brake, then
  // hold where it stops). After 3 failures in a row it stays off until the next arm.
  if (posFailed && posFailCount_ < 3) {
    posFailT_ += dt;
    if (posFailT_ >= POS_RETRY_S) { posFailed = false; posCtl.retry(); }
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

  // ---- the take-off point, if there was no GPS yet when it was armed
  if (!rthHomeSet && pos.initialized() && !pos.rejects && pos.sigma() < 1.0f && alt - groundAlt < 1.0f) {
    rthHomeN = pos.kn.p;
    rthHomeE = pos.ke.p;
    rthHomeSet = true;
  }

  // ---- flying by itself
  if (pendingNav_ && state == ST_FLYING) startPendingNav();
  NavOut navOut;
  if (state == ST_NAV) {
    const bool sticksMoved = !pilot.linkLost && (pilot.manual || fcAbs(pilot.roll) > 0.3f || fcAbs(pilot.pitch) > 0.3f ||
                                                 fcAbs(pilot.yaw) > 0.3f || fcAbs(pilot.climb) > 0.3f);
    if (!posReady) {                       // GPS or compass gone: nothing to navigate with, so land here
      nav.stop();
      state = ST_FAILSAFE;
      reason = R_NAV;
      landTimer_ = 0.0f;
    } else if (sticksMoved) {              // the pilot takes over
      cmdNavCancel();
    } else {
      const NavEvent ev = nav.update(navInput(dt), p, navOut);
      if (ev == NAV_EV_NO_PROGRESS) {
        nav.stop();
        state = ST_FAILSAFE;
        reason = R_NAV;
        landTimer_ = 0.0f;
      } else if (ev == NAV_EV_LAND_NOW) {   // low enough over the marker, or it has been lost: position hold keeps the place
        cmdLand();
      } else if (ev == NAV_EV_MISSION_DONE) {
        cmdRth(R_NONE);
        nav.update(navInput(0.0f), p, navOut);
      } else if (ev == NAV_EV_ARRIVED) {
        const float h = groundAlt + navOut.height;
        cmdNavCancel();
        altTarget = h;
      }
    }
  }

  // ---- attitude setpoints: position hold, or the sticks as angles
  const bool pilotOk = !(state == ST_FAILSAFE) && !pilot.linkLost;
  const float maxTilt = prof.maxTiltDeg * FC_DEG2RAD;
  float yawStick = 0.0f;
  const bool flyingState = state == ST_TAKEOFF || state == ST_FLYING || state == ST_LANDING || state == ST_FAILSAFE || state == ST_NAV;
  // position hold flies when switched on - and ALWAYS during a failsafe, so a lost link lands on the spot, and on a trip
  posActive = posReady && flyingState && (p.posHoldOn > 0.5f || state == ST_FAILSAFE || state == ST_NAV);
  if (state == ST_NAV) yawStick = navOut.yawCmd;
  else if (pilotOk && state != ST_IDLE) yawStick = pilot.yaw;
  if (posActive) {
    if (!wasPosActive_) posCtl.engage();
    PosCtlIn in;
    in.pN = pos.kn.p; in.pE = pos.ke.p; in.vN = pos.kn.v; in.vE = pos.ke.v;
    in.yaw = ekf.yaw();
    in.yawOffset = aidYawOffset;
    in.stickFwd = -pilot.pitch;     // pitch stick - = forward
    in.stickRight = pilot.roll;
    in.sticksAllowed = pilotOk && state != ST_IDLE && state != ST_NAV;
    in.nav = state == ST_NAV && navOut.fly;
    in.navN = navOut.targetN; in.navE = navOut.targetE; in.navSpeed = navOut.speed;
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
    case ST_NAV: {
      // to the wanted height along a ramp, the same way as a take-off
      float goal = groundAlt + navOut.height;
      if (p.altLimitOn > 0.5f) goal = fcMin(goal, ceilingAbs - 0.1f);
      const float e = goal - altTarget;
      if (fcAbs(e) > 0.001f) {
        const float down = navOut.descentLimit > 0.0f ? fcMin(navOut.descentLimit, prof.descent) : prof.descent;
        const float spd = fcClamp(sqrtf(2.0f * 0.5f * fcAbs(e)), 0.1f, e > 0.0f ? prof.climb : down);
        const float step = fcMin(spd * dt, fcAbs(e));
        altTarget += e > 0.0f ? step : -step;
        vzFeed = e > 0.0f ? spd : -spd;
      }
      // Hovering at home: a steady push or pull means someone has the drone. It takes much more thrust, or much less, than
      // hovering alone, or it can't stay at its height. Wind changes the thrust far less, and slowly.
      if (nav.stage == STAGE_HOVER && p.holdDetectOn > 0.5f) {
        const float t = altc.thrust;
        if (nav.hoverT < 3.0f) {
          hoverThrust_ = nav.hoverT < dt * 2.0f ? t : hoverThrust_ + (t - hoverThrust_) * fcMin(dt * 2.0f, 1.0f);
          heldT_ = 0.0f;
        } else {
          const bool pushed = fcAbs(t - hoverThrust_) > 0.2f * fcMax(hoverThrust_, 0.05f) || fcAbs(alt - altTarget) > 0.4f;
          heldT_ = pushed ? heldT_ + dt : 0.0f;
          if (!pushed) hoverThrust_ += (t - hoverThrust_) * fcMin(dt / 20.0f, 1.0f);
          if (heldT_ > 1.0f) { cmdDisarm(R_HELD); return; }
        }
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
      // stick held down, or manual throttle below hover, and sitting on the ground: the same touch-down test as a
      // landing, then the motors stop
      const bool low = alt - groundAlt < 0.3f;
      if ((manualThrust || pilot.climb < -0.3f) && low && fcAbs(vz) < 0.2f && altc.thrust < 0.95f * hoverT) landTimer_ += dt;
      else landTimer_ = 0.0f;
      if (landTimer_ > 0.8f) { cmdDisarm(R_LANDED); return; }
      if (!manualThrust) {
        const float c = pilot.climb;
        // going down by stick: the ground comes up gently whatever the stick says (0.6 m/s per metre of height, down
        // to the slow landing speed), and the speed builds up gradually, so the motors ease off instead of plunging
        const float wantDown = c < 0.0f ? fcMin(-c * prof.descent, fcMax(p.landSpeedLow, 0.6f * (alt - groundAlt))) : 0.0f;
        descentSp_ = wantDown > descentSp_ ? fcMin(wantDown, descentSp_ + 1.5f * dt) : wantDown;
        const float stickVz = c > 0.0f ? c * prof.climb : -descentSp_;
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
