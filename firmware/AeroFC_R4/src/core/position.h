// position.h - GPS position hold: estimator + controller.
//
// Frames: everything here is in the GPS frame (north/east from the home point, true north).  The attitude EKF
// works in the compass frame, which can be turned by a few degrees (declination); the angle between the two
// (yawOffset, learned in flight_core) is used only to rotate small vectors: the IMU acceleration on the way in
// and the commanded acceleration on the way out.  So a change of that angle can never move the hold point.
//
//   estimator : per axis a Kalman filter [position, speed, accelerometer bias]
//               predicted with the IMU acceleration, corrected with every new GPS fix
//   controller: position error -> speed set-point -> (PI, the I term learns the wind) -> acceleration -> tilt
#pragma once
#include "fc_math.h"
#include "params.h"

struct PosKf1 {
  float p = 0.0f, v = 0.0f, b = 0.0f;
  float P[3][3];
  void reset(float pos, float vel);
  void predict(float acc, float dt, float accNoise, float biasWalk);
  void update(float meas, float R);
};

class PositionEstimator {
 public:
  void reset() { init = false; rejects = 0; jumped = false; shiftN = shiftE = 0.0f; }
  void predict(float accN, float accE, float dt);
  // returns false if the fix was rejected as a jump
  bool gpsFix(float n, float e, float velHintN, float velHintE);
  bool initialized() const { return init; }
  bool jumping() const { return rejects > 0 && jumped; }    // the GPS has just jumped, and is not believed for now
  float sigma() const;              // position standard deviation (m)
  float gpsNoise() const;           // GPS noise used by the filter (m, at least 0.15)
  float gpsNoiseRaw() const;        // measured GPS noise (m): ~0 for Gazebo's clean GPS
  bool gpsQuiet() const { return init && gpsNoiseRaw() < 0.12f; }
  PosKf1 kn, ke;
  bool init = false;
  float innovSq = 0.0f;             // average squared GPS correction (adaptive GPS noise)
  uint16_t rejects = 0;
  bool jumped = false;              // the fixes being rejected started with a step no drone could make
  bool gpsBack = false;             // the GPS has been stuck and has just changed again: a step now is only it catching up
  float shiftN = 0.0f, shiftE = 0.0f;   // the GPS has settled this far from where the drone is: to take off all its fixes from now on
  uint8_t jumps = 0;                // how many times that has happened
 private:
  float lastN_ = 0.0f, lastE_ = 0.0f;
};

enum PosState : uint8_t { POS_OFF = 0, POS_NOT_READY, POS_MOVING, POS_BRAKING, POS_HOLDING, POS_FAILED, POS_NAV };
const char *posStateName(uint8_t s);

struct PosCtlIn {
  float pN, pE, vN, vE;      // GPS frame
  float yaw;                 // EKF (compass) yaw, rad
  float yawOffset;           // rotation GPS frame -> EKF frame, rad
  float stickFwd, stickRight;// -1..1 (forward / right positive), already shaped
  bool sticksAllowed;        // false in failsafe
  bool airborne;
  float heightAgl;           // height above the take-off point
  bool touchdown;            // landing detector running: no more tilt
  uint8_t stickStyle;        // PosStick
  bool nav;                  // fly to navN, navE at navSpeed (the sticks don't count then)
  float navN, navE, navSpeed;
};

class PositionController {
 public:
  void reset(float pN, float pE);   // hold here, forget the wind estimate
  void engage() { wasActive_ = true; releaseT_ = 1.0f; }   // switched on in flight: brake, then hold
  void retry() { failed = false; runawayT_ = 0.0f; speedingT_ = 0.0f; watchSpeed_ = false; }   // after a failure: allowed to try again
  void update(const Params &p, const SpeedProfile &prof, const PosCtlIn &in, float dt, float &rollSp, float &pitchSp);
  float holdN = 0, holdE = 0;       // hold point (GPS frame)
  float vSpN = 0, vSpE = 0;         // speed set-point
  float iN = 0, iE = 0;             // wind estimate (m/s^2)
  float dist = 0;                   // distance to the hold point
  uint8_t state = POS_HOLDING;
  bool failed = false;              // runaway detected: position hold switched itself off
 private:
  float releaseT_ = 1.0f, runawayT_ = 0.0f, lastDist_ = 0.0f, brakeT_ = 0.0f;
  float slowestSpeed_ = 0.0f, speedingT_ = 0.0f;    // while it should be slowing down: the lowest speed so far, time spent getting faster again
  bool watchSpeed_ = false, wasActive_ = false;
};
