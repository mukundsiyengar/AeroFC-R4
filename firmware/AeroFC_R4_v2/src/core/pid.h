// pid.h - PID controller used by every control loop.
//   * D acts on the measurement, not the error -> no "kick" when the setpoint jumps
//   * D is low-pass filtered (sensor data arrives at only ~50 Hz)
//   * the integrator is clamped and stops growing while the output is saturated (anti-windup)
//   * the integrator can be frozen (e.g. on the ground) and reset
#pragma once
#include "fc_math.h"

struct Pid {
  float integ = 0.0f;
  float prevMeas = 0.0f;
  bool havePrev = false;
  Pt1 dFilter;
  float lastP = 0.0f, lastD = 0.0f, lastOut = 0.0f;

  void reset(float i = 0.0f) {
    integ = i;
    havePrev = false;
    dFilter.reset();
    lastP = lastD = lastOut = 0.0f;
  }

  float update(float setpoint, float meas, float dt, float kp, float ki, float kd, float iLimit, float outLimit,
               float dCutHz, bool integrate) {
    const float err = setpoint - meas;
    float d = 0.0f;
    if (havePrev && dt > 1e-4f) d = -kd * dFilter.update((meas - prevMeas) / dt, dCutHz, dt);
    prevMeas = meas;
    havePrev = true;
    const float p = kp * err;
    const float unsat = p + integ + d;
    // conditional integration: do not integrate further into a saturated output
    const bool saturatedSameSign = (unsat > outLimit && err > 0.0f) || (unsat < -outLimit && err < 0.0f);
    if (integrate && !saturatedSameSign) integ = fcClamp(integ + ki * err * dt, -iLimit, iLimit);
    if (!integrate) integ = 0.0f;
    lastP = p;
    lastD = d;
    lastOut = fcClamp(p + integ + d, -outLimit, outLimit);
    return lastOut;
  }
};
