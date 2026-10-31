#include "controllers.h"

// ---------------------------------------------------------------- attitude
void AttitudeController::reset(float currentYaw) {
  pidRoll.reset();
  pidPitch.reset();
  pidYaw.reset();
  headingTarget = currentYaw;
  headingLocked = true;
  releaseTimer_ = 0.0f;
  rateSp = Vec3();
  yawRateSp = 0.0f;
}

Vec3 AttitudeController::update(const Params &p, const Quat &q, const Vec3 &rate, float rollSp, float pitchSp,
                                float yawStick, float maxYawRate, bool airborne, float dt) {
  const float roll = q.roll(), pitch = q.pitch(), yaw = q.yaw();

  // ---- yaw: rate command from the stick, heading hold when the stick is centred
  if (!airborne) {
    headingTarget = yaw;            // on the ground: follow, so we hold the take-off heading
    headingLocked = true;
    releaseTimer_ = 0.0f;
    yawRateSp = yawStick * maxYawRate;
  } else if (fcAbs(yawStick) > 0.01f || p.headingHoldOn < 0.5f) {
    yawRateSp = yawStick * maxYawRate;
    headingLocked = false;
    releaseTimer_ = 0.0f;
  } else {
    if (!headingLocked) {
      releaseTimer_ += dt;
      yawRateSp = 0.0f;
      // lock once the turn has stopped (or after 0.3 s), so the drone does not swing back
      if (releaseTimer_ > 1.0f || (releaseTimer_ > 0.1f && fcAbs(rate.z) < 0.03f)) { headingTarget = yaw; headingLocked = true; }
    }
    if (headingLocked) yawRateSp = fcClamp(p.headingP * fcWrapPi(headingTarget - yaw), -maxYawRate, maxYawRate);
  }

  // ---- angle loop: Euler-angle rates, then exact conversion to body rates
  const float maxRate = p.maxRateDps * FC_DEG2RAD;
  const float rollDot = fcClamp(p.angleP * (rollSp - roll), -maxRate, maxRate);
  const float pitchDot = fcClamp(p.angleP * (pitchSp - pitch), -maxRate, maxRate);
  const float sr = sinf(roll), cr = cosf(roll), sp = sinf(pitch), cp = cosf(pitch);
  rateSp.x = rollDot - yawRateSp * sp;
  rateSp.y = pitchDot * cr + yawRateSp * sr * cp;
  rateSp.z = -pitchDot * sr + yawRateSp * cr * cp;

  // ---- rate loop
  const float iLim = p.rateILimit;
  Vec3 u;
  u.x = pidRoll.update(rateSp.x, rate.x, dt, p.rollP, p.rollI, p.rollD, iLim, 0.35f, p.dCutHz, airborne);
  u.y = pidPitch.update(rateSp.y, rate.y, dt, p.pitchP, p.pitchI, p.pitchD, iLim, 0.35f, p.dCutHz, airborne);
  u.z = pidYaw.update(rateSp.z, rate.z, dt, p.yawP, p.yawI, 0.0f, 0.15f, 0.25f, p.dCutHz, airborne);
  return u;
}

// ---------------------------------------------------------------- altitude
void AltitudeController::reset(float hoverThrust) {
  integ = hoverThrust;
  vzSp = accSp = 0.0f;
  thrust = 0.0f;
}

float AltitudeController::update(const Params &p, float alt, float vz, float altTarget, float vzFeed, bool useAltLoop,
                                 float cosTilt, bool airborne, float dt, float maxUp, float maxDown) {
  const float hover = hoverThrustFromCmd(p);
  if (useAltLoop) vzSp = p.altP * (altTarget - alt) + vzFeed;
  else vzSp = vzFeed;
  vzSp = fcClamp(vzSp, -maxDown, maxUp);

  const float err = vzSp - vz;
  accSp = fcClamp(p.climbP * err, -4.0f, 4.0f);
  const float k = hover / FC_GRAVITY;   // thrust per m/s^2 around hover
  float t = integ + k * accSp;
  const float ct = fcMax(cosTilt, 0.82f);   // tilt compensation up to ~35 deg
  t /= ct;
  // integrate only in the air; stop when the output is saturated in the same direction
  const bool satHigh = t >= 1.0f && err > 0.0f;
  const bool satLow = t <= 0.05f && err < 0.0f;
  if (airborne && !satHigh && !satLow)
    integ = fcClamp(integ + k * p.climbI * err * dt, 0.4f * hover, fcMin(2.0f * hover, 0.95f));
  thrust = fcClamp(t, 0.0f, 1.0f);
  return thrust;
}

// ---------------------------------------------------------------- mixer
//                         FR(ccw) RL(ccw) FL(cw) RR(cw)
static const float kRoll[4]  = { -1.0f, +1.0f, +1.0f, -1.0f };   // +roll  (right side down) -> left motors up
static const float kPitch[4] = { +1.0f, -1.0f, +1.0f, -1.0f };   // +pitch (nose up)         -> front motors up
static const float kYaw[4]   = { +1.0f, +1.0f, -1.0f, -1.0f };   // +yaw   (nose right)      -> CCW props up

void mixerRun(const Params &p, float T, float r, float pt, float y, bool armed, MixerOut &out) {
  out.saturated = false;
  if (!armed) {
    for (int i = 0; i < 4; i++) { out.cmd[i] = 0.0f; out.thrust[i] = 0.0f; }
    return;
  }
  const float tMin = cmdToThrust(p, p.motorIdle);
  const float tMax = 1.0f;
  const float avail = tMax - tMin;
  if (!fcFinite(T)) T = 0.0f;
  if (!fcFinite(r)) r = 0.0f;
  if (!fcFinite(pt)) pt = 0.0f;
  if (!fcFinite(y)) y = 0.0f;

  // 1) roll/pitch must fit into the available range (rare: only with huge commands)
  float rp[4], rpMax = -10.0f, rpMin = 10.0f;
  for (int i = 0; i < 4; i++) {
    rp[i] = kRoll[i] * r + kPitch[i] * pt;
    rpMax = fcMax(rpMax, rp[i]);
    rpMin = fcMin(rpMin, rp[i]);
  }
  float spread = rpMax - rpMin;
  if (spread > avail) {
    const float s = avail / spread;
    for (int i = 0; i < 4; i++) rp[i] *= s;
    rpMax *= s; rpMin *= s; spread = avail;
    out.saturated = true;
  }
  // 2) yaw gets what is left (yaw is the least important axis)
  float m[4], mMax = -10.0f, mMin = 10.0f;
  float yaw = y;
  for (int pass = 0; pass < 2; pass++) {
    mMax = -10.0f; mMin = 10.0f;
    for (int i = 0; i < 4; i++) {
      m[i] = rp[i] + kYaw[i] * yaw;
      mMax = fcMax(mMax, m[i]);
      mMin = fcMin(mMin, m[i]);
    }
    if (mMax - mMin <= avail + 1e-6f || pass == 1) break;
    // shrink yaw so that the total spread fits
    const float room = fcMax(avail - spread, 0.0f);
    const float ya = fcAbs(yaw);
    yaw = (ya > 1e-6f) ? yaw * fcClamp(0.5f * room / ya, 0.0f, 1.0f) : 0.0f;
    out.saturated = true;
  }
  // 3) shift the collective (at most +-0.15) so that no motor leaves [tMin, tMax]
  float Tc = T;
  if (Tc + mMax > tMax) { Tc = fcMax(tMax - mMax, T - 0.15f); out.saturated = true; }
  if (Tc + mMin < tMin) { Tc = fcMin(tMin - mMin, T + 0.15f); out.saturated = true; }
  for (int i = 0; i < 4; i++) {
    const float t = fcClamp(Tc + m[i], tMin, tMax);
    out.thrust[i] = t;
    out.cmd[i] = thrustToCmd(p, t);
  }
}
