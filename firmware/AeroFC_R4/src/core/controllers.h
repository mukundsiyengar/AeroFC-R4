// controllers.h - attitude (angle -> rate -> torque), altitude (alt -> climb rate -> thrust) and the motor mixer.
//
// Units used between the blocks ("normalized"):
//   torque  r, p, y : change of one motor's thrust as a fraction of its maximum thrust
//   thrust  T       : collective, fraction of maximum thrust of one motor (hover ~0.44 on the Gazebo X3)
// The mixer converts thrust to motor command with cmd = motorMax * sqrt(thrust), because propeller thrust grows
// with speed squared. That makes every loop behave the same at any throttle ("thrust linearization").
#pragma once
#include "fc_math.h"
#include "params.h"
#include "pid.h"

class AttitudeController {
 public:
  void reset(float currentYaw);
  // rollSp / pitchSp in rad, yawStick -1..1. Returns normalized torques (FRD roll, pitch, yaw).
  Vec3 update(const Params &p, const Quat &q, const Vec3 &rate, float rollSp, float pitchSp, float yawStick,
              float maxYawRate, bool airborne, float dt);
  Vec3 rateSp;           // body rate setpoint (rad/s)
  float headingTarget = 0.0f;
  bool headingLocked = false;
  float yawRateSp = 0.0f;
  Pid pidRoll, pidPitch, pidYaw;
 private:
  float releaseTimer_ = 0.0f;
};

class AltitudeController {
 public:
  void reset(float hoverThrust);
  // useAltLoop: hold altTarget; otherwise follow vzFeed (m/s up). Returns collective thrust 0..1.
  // maxUp / maxDown: climb / descent speed limits (m/s, positive)
  float update(const Params &p, float alt, float vz, float altTarget, float vzFeed, bool useAltLoop, float cosTilt,
               bool airborne, float dt, float maxUp, float maxDown);
  float vzSp = 0.0f, accSp = 0.0f;
  float integ = 0.0f;    // = learned hover thrust while hovering
  float thrust = 0.0f;
};

struct MixerOut {
  float cmd[4];          // motor commands 0..1 (what is sent to the simulator)
  float thrust[4];       // per-motor normalized thrust
  bool saturated;
};

// Quad-X mixer for the Gazebo X3 motor order:
//   0 front-right (CCW), 1 rear-left (CCW), 2 front-left (CW), 3 rear-right (CW)
void mixerRun(const Params &p, float collective, float r, float pt, float y, bool armed, MixerOut &out);

static inline float hoverThrustFromCmd(const Params &p) {
  const float c = fcClamp(p.hoverCmd / fcMax(p.motorMax, 0.1f), 0.0f, 1.0f);
  return c * c;
}
static inline float cmdToThrust(const Params &p, float cmd) {
  const float c = fcClamp(cmd / fcMax(p.motorMax, 0.1f), 0.0f, 1.0f);
  return c * c;
}
static inline float thrustToCmd(const Params &p, float t) { return p.motorMax * sqrtf(fcClamp(t, 0.0f, 1.0f)); }
