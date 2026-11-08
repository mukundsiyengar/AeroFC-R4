// alt_ekf.h - vertical Kalman filter: state [altitude (m, up), climb rate (m/s, up), accelerometer bias (m/s^2)]
//   predict : earth-frame vertical acceleration from the IMU (fast, smooth, but drifts)
//   update  : barometer altitude (slow/noisy, but does not drift)
// The result is an altitude without lag and a clean climb rate, which the altitude-hold loop needs.
#pragma once
#include "fc_math.h"

class AltEkf {
 public:
  void reset(float altitude);
  void predict(float accUp, float dt, float accNoise = 0.5f, float biasWalk = 0.01f);
  bool updateBaro(float altMeasured, float noise);   // false = rejected as a spike
  float alt = 0.0f, vz = 0.0f, accBias = 0.0f;
  float P[3][3];
  uint16_t rejects = 0;
  float lastInnov = 0.0f;
};
