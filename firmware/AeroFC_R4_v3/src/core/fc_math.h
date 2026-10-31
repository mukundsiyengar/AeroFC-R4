// fc_math.h - small float-only vector / quaternion helpers used by every part of the flight controller.
//
// Frames used everywhere in the core:
//   body  = FRD  (x forward, y right, z down)
//   earth = NED  (x north,   y east,  z down)
// Quaternion q rotates a body vector into the earth frame:  v_earth = q * v_body * q^-1
// Euler angles are the usual aerospace Z-Y-X set: yaw (heading), then pitch, then roll.
//   roll  > 0 : right side down          pitch > 0 : nose up          yaw > 0 : nose turns right (clockwise from above)
#pragma once
#include <math.h>
#include <stdint.h>
#include <stddef.h>

static const float FC_PI      = 3.14159265358979f;
static const float FC_TWO_PI  = 6.28318530717959f;
static const float FC_DEG2RAD = 0.0174532925199433f;
static const float FC_RAD2DEG = 57.2957795130823f;
static const float FC_GRAVITY = 9.80665f;

static inline float fcClamp(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline float fcAbs(float v) { return v < 0.0f ? -v : v; }
static inline float fcSq(float v) { return v * v; }
static inline float fcMin(float a, float b) { return a < b ? a : b; }
static inline float fcMax(float a, float b) { return a > b ? a : b; }
// true for normal numbers, false for NaN and +-infinity (works without <cmath> isfinite quirks)
static inline bool fcFinite(float v) { return (v == v) && (v - v == 0.0f); }

static inline float fcWrapPi(float a) {
  if (!fcFinite(a)) return 0.0f;
  while (a > FC_PI) a -= FC_TWO_PI;
  while (a < -FC_PI) a += FC_TWO_PI;
  return a;
}

// Remove a small dead zone around 0 and rescale so the output still reaches +-1.
static inline float fcDeadband(float v, float db) {
  if (fcAbs(v) <= db) return 0.0f;
  return (v > 0.0f ? v - db : v + db) / (1.0f - db);
}

// Mix linear and cubic response: expo 0 = linear, 1 = fully cubic (finer control near centre).
static inline float fcExpo(float v, float expo) { return v * (1.0f - expo) + v * v * v * expo; }

// atan2 that never returns NaN
static inline float fcAtan2(float y, float x) {
  if (!fcFinite(y) || !fcFinite(x) || (y == 0.0f && x == 0.0f)) return 0.0f;
  return atan2f(y, x);
}
static inline float fcAsin(float v) { return asinf(fcClamp(v, -1.0f, 1.0f)); }

// First-order low-pass filter ("PT1"), cutoff in Hz.
struct Pt1 {
  float y = 0.0f;
  bool primed = false;
  float update(float x, float cutoffHz, float dt) {
    if (!primed || cutoffHz <= 0.0f) { y = x; primed = true; return y; }
    const float rc = 1.0f / (FC_TWO_PI * cutoffHz);
    y += (x - y) * (dt / (rc + dt));
    return y;
  }
  void reset(float v = 0.0f) { y = v; primed = false; }
};

struct Vec3 {
  float x, y, z;
  Vec3() : x(0.0f), y(0.0f), z(0.0f) {}
  Vec3(float a, float b, float c) : x(a), y(b), z(c) {}
  Vec3 operator+(const Vec3 &o) const { return Vec3(x + o.x, y + o.y, z + o.z); }
  Vec3 operator-(const Vec3 &o) const { return Vec3(x - o.x, y - o.y, z - o.z); }
  Vec3 operator*(float s) const { return Vec3(x * s, y * s, z * s); }
  Vec3 operator-() const { return Vec3(-x, -y, -z); }
  Vec3 &operator+=(const Vec3 &o) { x += o.x; y += o.y; z += o.z; return *this; }
  Vec3 &operator-=(const Vec3 &o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
  float dot(const Vec3 &o) const { return x * o.x + y * o.y + z * o.z; }
  Vec3 cross(const Vec3 &o) const { return Vec3(y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x); }
  float norm() const { return sqrtf(x * x + y * y + z * z); }
  bool finite() const { return fcFinite(x) && fcFinite(y) && fcFinite(z); }
  // returns false (and leaves the vector unchanged) if it is too short or not finite
  bool normalize() {
    const float n = norm();
    if (!(n > 1e-9f) || !fcFinite(n)) return false;
    const float k = 1.0f / n;
    x *= k; y *= k; z *= k;
    return true;
  }
  float operator[](int i) const { return i == 0 ? x : (i == 1 ? y : z); }
};

struct Quat {
  float w, x, y, z;
  Quat() : w(1.0f), x(0.0f), y(0.0f), z(0.0f) {}
  Quat(float a, float b, float c, float d) : w(a), x(b), y(c), z(d) {}

  Quat operator*(const Quat &o) const {
    return Quat(w * o.w - x * o.x - y * o.y - z * o.z,
                w * o.x + x * o.w + y * o.z - z * o.y,
                w * o.y - x * o.z + y * o.w + z * o.x,
                w * o.z + x * o.y - y * o.x + z * o.w);
  }
  Quat conj() const { return Quat(w, -x, -y, -z); }
  bool finite() const { return fcFinite(w) && fcFinite(x) && fcFinite(y) && fcFinite(z); }
  void normalize() {
    const float n = sqrtf(w * w + x * x + y * y + z * z);
    if (!(n > 1e-9f) || !fcFinite(n)) { w = 1.0f; x = y = z = 0.0f; return; }
    const float k = 1.0f / n;
    w *= k; x *= k; y *= k; z *= k;
    if (w < 0.0f) { w = -w; x = -x; y = -y; z = -z; }   // keep one consistent sign
  }

  // Rotation by the small/any angle vector r (axis * angle), exact.
  static Quat fromRotVec(const Vec3 &r) {
    const float a = r.norm();
    if (a < 1e-6f) return Quat(1.0f, 0.5f * r.x, 0.5f * r.y, 0.5f * r.z);
    const float s = sinf(0.5f * a) / a;
    return Quat(cosf(0.5f * a), r.x * s, r.y * s, r.z * s);
  }
  static Quat fromEuler(float roll, float pitch, float yaw) {
    const float cr = cosf(0.5f * roll), sr = sinf(0.5f * roll);
    const float cp = cosf(0.5f * pitch), sp = sinf(0.5f * pitch);
    const float cy = cosf(0.5f * yaw), sy = sinf(0.5f * yaw);
    return Quat(cr * cp * cy + sr * sp * sy,
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy);
  }

  // Rotation matrix rows (body -> earth).  R[2] is the third row; R[2][*] = earth z of each body axis.
  void toMatrix(float R[3][3]) const {
    const float xx = x * x, yy = y * y, zz = z * z;
    const float xy = x * y, xz = x * z, yz = y * z, wx = w * x, wy = w * y, wz = w * z;
    R[0][0] = 1.0f - 2.0f * (yy + zz); R[0][1] = 2.0f * (xy - wz);        R[0][2] = 2.0f * (xz + wy);
    R[1][0] = 2.0f * (xy + wz);        R[1][1] = 1.0f - 2.0f * (xx + zz); R[1][2] = 2.0f * (yz - wx);
    R[2][0] = 2.0f * (xz - wy);        R[2][1] = 2.0f * (yz + wx);        R[2][2] = 1.0f - 2.0f * (xx + yy);
  }
  Vec3 rotate(const Vec3 &v) const {          // body -> earth
    float R[3][3]; toMatrix(R);
    return Vec3(R[0][0] * v.x + R[0][1] * v.y + R[0][2] * v.z,
                R[1][0] * v.x + R[1][1] * v.y + R[1][2] * v.z,
                R[2][0] * v.x + R[2][1] * v.y + R[2][2] * v.z);
  }
  Vec3 rotateInv(const Vec3 &v) const {       // earth -> body
    float R[3][3]; toMatrix(R);
    return Vec3(R[0][0] * v.x + R[1][0] * v.y + R[2][0] * v.z,
                R[0][1] * v.x + R[1][1] * v.y + R[2][1] * v.z,
                R[0][2] * v.x + R[1][2] * v.y + R[2][2] * v.z);
  }
  float roll() const { return fcAtan2(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y)); }
  float pitch() const { return fcAsin(2.0f * (w * y - x * z)); }
  float yaw() const { return fcAtan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z)); }
};

// cos(tilt) = earth-z component of the body z axis.  1 = level, 0 = on its side.
static inline float fcCosTilt(const Quat &q) { return 1.0f - 2.0f * (q.x * q.x + q.y * q.y); }
