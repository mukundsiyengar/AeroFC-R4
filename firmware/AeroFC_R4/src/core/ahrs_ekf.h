// ahrs_ekf.h - Attitude and Heading Reference System built on an error-state Extended Kalman Filter.
//
//  State        : attitude quaternion q (body FRD -> earth NED) + gyro bias b (rad/s)
//  Error state  : dtheta (3, small rotation in the body frame) + dbias (3)          -> P is 6 x 6
//  Predict      : integrate (gyro - b);  P = F P F' + Q
//  Accel update : the accelerometer points "up" (specific force) when not accelerating -> corrects roll & pitch
//  Compass upd. : tilt-compensated heading -> corrects ONLY yaw (the correction is projected on the earth z axis,
//                 so a disturbed compass can never tilt the horizon)
// Every measurement is fused as a scalar (one row at a time): no matrix inversion needed.
#pragma once
#include "fc_math.h"

class AhrsEkf {
 public:
  void reset();
  // Collects still samples until it can initialise roll/pitch (and yaw from the compass if usable).
  // Returns true once initialised.
  bool initStep(const Vec3 &acc, const Vec3 &mag, bool useMag, float declRad);
  void predict(const Vec3 &gyro, float dt, float gyroNoise);
  // sigma = accelerometer noise in m/s^2; gate = allowed |a| deviation from the expected value (fraction of g).
  // refEarth = expected specific force in the earth frame (NED). Without GPS it is (0, 0, -g): "the accelerometer
  // points up".  With GPS it is (accel - gravity), so accelerating / braking no longer looks like a tilt.
  bool updateAccel(const Vec3 &acc, float sigma, float gate, const Vec3 *refEarth = 0);
  // heading measurement from the magnetometer; returns true if fused
  bool updateMag(const Vec3 &mag, float declRad, float sigmaRad);
  bool magDisagrees() const { return magSeqRejects_ >= 100; }   // about 2 s of compass readings far from this heading
  void alignYawToMag(const Vec3 &mag, float declRad);           // take the compass's word for the heading (on the ground)

  bool initialized() const { return init_; }
  float roll() const { return q.roll(); }
  float pitch() const { return q.pitch(); }
  float yaw() const { return q.yaw(); }
  float yawSigma() const { return sqrtf(fcMax(yawVar(), 0.0f)); }
  bool healthy() const;               // finite and covariance sane
  void resetMagHealth() { magNormRef = 0.0f; magSeqRejects_ = 0; }   // after a compass calibration / re-enable

  Quat q;
  Vec3 bias;            // gyro bias estimate (rad/s)
  Vec3 rate;            // last bias-corrected gyro (rad/s, FRD)
  float P[6][6];
  bool magHealthy = false;
  float magNormRef = 0.0f;
  uint32_t accRejects = 0, magRejects = 0, magFused = 0;
  float lastAccInnov = 0.0f, lastMagInnov = 0.0f;

  static float magHeading(const Quat &q, const Vec3 &mag);   // radians, 0 = magnetic north, clockwise +

 private:
  bool init_ = false;
  uint16_t initN_ = 0;
  Vec3 accSum_, magSum_;
  uint16_t magSeqRejects_ = 0;
  float yawVar() const;
  void fuseScalar(const float H[6], float residual, float R, float dx[6], float Kout[6]);
  void inject(const float dx[6]);
  void checkMagHealth(const Vec3 &mag);
};
