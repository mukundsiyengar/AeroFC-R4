#include "ahrs_ekf.h"
#include <string.h>

static const uint16_t INIT_SAMPLES = 25;
// rad/s per sqrt(s): how fast the gyro's drift may change. Learning time is about 6 s (1e-5 took over a minute, and
// a gyro that starts drifting in flight tilted the drone 18 deg before the filter caught up)
static const float BIAS_WALK = 1e-3f;
static const float BIAS_LIMIT = 0.1f;          // rad/s - anything bigger is not a bias

void AhrsEkf::reset() {
  q = Quat();
  bias = Vec3();
  rate = Vec3();
  memset(P, 0, sizeof(P));
  init_ = false;
  initN_ = 0;
  accSum_ = Vec3();
  magSum_ = Vec3();
  magHealthy = false;
  magNormRef = 0.0f;
  magSeqRejects_ = 0;
  accRejects = magRejects = magFused = 0;
}

float AhrsEkf::magHeading(const Quat &qa, const Vec3 &mag) {
  // rotate the field by roll and pitch only ("level it"), then the heading is atan2(-east, north)
  const Quat tilt = Quat::fromEuler(qa.roll(), qa.pitch(), 0.0f);
  const Vec3 m = tilt.rotate(mag);
  return fcAtan2(-m.y, m.x);
}

void AhrsEkf::checkMagHealth(const Vec3 &mag) {
  const float n = mag.norm();
  if (!(n > 1.0f) || !fcFinite(n)) { magHealthy = false; return; }   // < 1 uT: no field (zeros from the bridge)
  if (magNormRef <= 0.0f) magNormRef = n;
  magHealthy = n > 0.7f * magNormRef && n < 1.3f * magNormRef && magSeqRejects_ < 100;
}

bool AhrsEkf::initStep(const Vec3 &acc, const Vec3 &mag, bool useMag, float declRad) {
  if (init_) return true;
  const float an = acc.norm();
  if (!(an > 0.8f * FC_GRAVITY && an < 1.2f * FC_GRAVITY)) { initN_ = 0; accSum_ = Vec3(); magSum_ = Vec3(); return false; }
  accSum_ += acc;
  magSum_ += mag;
  if (++initN_ < INIT_SAMPLES) return false;

  const Vec3 a = accSum_ * (1.0f / (float)initN_);
  const Vec3 m = magSum_ * (1.0f / (float)initN_);
  // specific force at rest = (g sin(pitch), -g sin(roll) cos(pitch), -g cos(roll) cos(pitch))
  const float roll = fcAtan2(-a.y, -a.z);
  const float pitch = fcAtan2(a.x, sqrtf(a.y * a.y + a.z * a.z));
  float yaw = 0.0f;
  checkMagHealth(m);
  const bool magOk = useMag && magHealthy;
  if (magOk) yaw = fcWrapPi(magHeading(Quat::fromEuler(roll, pitch, 0.0f), m) + declRad);
  q = Quat::fromEuler(roll, pitch, yaw);
  bias = Vec3();
  memset(P, 0, sizeof(P));
  P[0][0] = P[1][1] = fcSq(0.05f);
  P[2][2] = magOk ? fcSq(0.2f) : fcSq(0.01f);   // without a compass, yaw simply starts at 0
  P[3][3] = P[4][4] = P[5][5] = fcSq(0.01f);
  init_ = true;
  return true;
}

void AhrsEkf::predict(const Vec3 &gyro, float dt, float gyroNoise) {
  if (!init_) return;
  rate = gyro - bias;
  q = q * Quat::fromRotVec(rate * dt);
  q.normalize();

  // F = [ I - [w x] dt   -I dt ]
  //     [ 0               I    ]
  float F[6][6];
  memset(F, 0, sizeof(F));
  const float wx = rate.x * dt, wy = rate.y * dt, wz = rate.z * dt;
  F[0][0] = 1.0f; F[0][1] = wz;   F[0][2] = -wy;
  F[1][0] = -wz;  F[1][1] = 1.0f; F[1][2] = wx;
  F[2][0] = wy;   F[2][1] = -wx;  F[2][2] = 1.0f;
  F[0][3] = F[1][4] = F[2][5] = -dt;
  F[3][3] = F[4][4] = F[5][5] = 1.0f;

  float FP[6][6];
  for (int i = 0; i < 6; i++)
    for (int j = 0; j < 6; j++) {
      float s = 0.0f;
      for (int k = 0; k < 6; k++) s += F[i][k] * P[k][j];
      FP[i][j] = s;
    }
  float qa = gyroNoise * gyroNoise * dt;
  if (dt > 0.06f) qa += fcSq(0.1f * dt);   // a packet gap: be less sure about the attitude
  const float qb = BIAS_WALK * BIAS_WALK * dt;
  for (int i = 0; i < 6; i++)
    for (int j = i; j < 6; j++) {
      float s = 0.0f;
      for (int k = 0; k < 6; k++) s += FP[i][k] * F[j][k];
      if (i == j) s += (i < 3) ? qa : qb;
      P[i][j] = s;
      P[j][i] = s;
    }
}

// One scalar measurement: residual = z - h(x), H = d h / d(error state).
// Updates the accumulated correction dx and P.  Returns the Kalman gain in Kout (if given).
void AhrsEkf::fuseScalar(const float H[6], float residual, float R, float dx[6], float Kout[6]) {
  float PH[6];
  for (int i = 0; i < 6; i++) {
    float s = 0.0f;
    for (int k = 0; k < 6; k++) s += P[i][k] * H[k];
    PH[i] = s;
  }
  float S = R;
  for (int k = 0; k < 6; k++) S += H[k] * PH[k];
  if (!(S > 1e-12f)) return;
  float Hdx = 0.0f;
  for (int k = 0; k < 6; k++) Hdx += H[k] * dx[k];
  const float innov = residual - Hdx;
  float K[6];
  for (int i = 0; i < 6; i++) {
    K[i] = PH[i] / S;
    dx[i] += K[i] * innov;
  }
  for (int i = 0; i < 6; i++)
    for (int j = 0; j < 6; j++) P[i][j] -= K[i] * PH[j];
  for (int i = 0; i < 6; i++)
    for (int j = i + 1; j < 6; j++) { const float m = 0.5f * (P[i][j] + P[j][i]); P[i][j] = m; P[j][i] = m; }
  if (Kout) memcpy(Kout, K, sizeof(K));
}

void AhrsEkf::inject(const float dx[6]) {
  q = q * Quat::fromRotVec(Vec3(dx[0], dx[1], dx[2]));
  q.normalize();
  bias = Vec3(fcClamp(bias.x + dx[3], -BIAS_LIMIT, BIAS_LIMIT),
              fcClamp(bias.y + dx[4], -BIAS_LIMIT, BIAS_LIMIT),
              fcClamp(bias.z + dx[5], -BIAS_LIMIT, BIAS_LIMIT));
}

bool AhrsEkf::updateAccel(const Vec3 &acc, float sigma, float gate, const Vec3 *refEarth) {
  if (!init_) return false;
  Vec3 ref(0.0f, 0.0f, -FC_GRAVITY);
  if (refEarth && refEarth->finite()) ref = *refEarth;
  const float expected = ref.norm();
  const float an = acc.norm();
  if (!(fcAbs(an - expected) < gate * FC_GRAVITY) || !ref.normalize()) { accRejects++; return false; }
  const Vec3 y = acc * (1.0f / an);
  // predicted direction of the specific force in the body frame: R' * ref
  const Vec3 h = q.rotateInv(ref);
  const Vec3 r = y - h;
  lastAccInnov = r.norm();
  const float Rn = fcSq(sigma / FC_GRAVITY);
  // d h / d dtheta = [h x]  (skew matrix of h)
  const float Hs[3][3] = { { 0.0f, -h.z, h.y }, { h.z, 0.0f, -h.x }, { -h.y, h.x, 0.0f } };
  float dx[6] = { 0, 0, 0, 0, 0, 0 };
  for (int i = 0; i < 3; i++) {
    const float H[6] = { Hs[i][0], Hs[i][1], Hs[i][2], 0.0f, 0.0f, 0.0f };
    fuseScalar(H, r[i], Rn, dx, 0);
  }
  inject(dx);
  return true;
}

bool AhrsEkf::updateMag(const Vec3 &mag, float declRad, float sigmaRad) {
  if (!init_) return false;
  checkMagHealth(mag);
  if (!magHealthy) return false;
  const float meas = fcWrapPi(magHeading(q, mag) + declRad);
  const float res = fcWrapPi(meas - q.yaw());
  lastMagInnov = res;
  const float gate = fcMax(30.0f * FC_DEG2RAD, 3.0f * yawSigma());
  if (fcAbs(res) > gate) {
    magRejects++;
    if (magSeqRejects_ < 1000) magSeqRejects_++;   // 100 in a row (~2 s): compass is inconsistent -> unhealthy
    return false;
  }
  magSeqRejects_ = 0;

  float Rm[3][3];
  q.toMatrix(Rm);
  // yaw change produced by a body-frame error rotation: earth-z component of (R * dtheta)
  const float H[6] = { Rm[2][0], Rm[2][1], Rm[2][2], 0.0f, 0.0f, 0.0f };
  float PH[6];
  for (int i = 0; i < 6; i++) {
    float s = 0.0f;
    for (int k = 0; k < 6; k++) s += P[i][k] * H[k];
    PH[i] = s;
  }
  float S = sigmaRad * sigmaRad;
  for (int k = 0; k < 6; k++) S += H[k] * PH[k];
  if (!(S > 1e-12f)) return false;
  float K[6];
  for (int i = 0; i < 6; i++) K[i] = PH[i] / S;
  // Keep only the part of the correction that turns around the EARTH z axis (pure heading change),
  // for both the attitude and the bias rows.  e = R' * ez (earth z in body coordinates) = H[0..2].
  const Vec3 e(H[0], H[1], H[2]);
  const Vec3 ka(K[0], K[1], K[2]), kb(K[3], K[4], K[5]);
  const Vec3 kaP = e * e.dot(ka), kbP = e * e.dot(kb);
  K[0] = kaP.x; K[1] = kaP.y; K[2] = kaP.z;
  K[3] = kbP.x; K[4] = kbP.y; K[5] = kbP.z;
  float dx[6];
  for (int i = 0; i < 6; i++) dx[i] = K[i] * res;
  for (int i = 0; i < 6; i++)
    for (int j = 0; j < 6; j++) P[i][j] -= K[i] * PH[j];
  for (int i = 0; i < 6; i++)
    for (int j = i + 1; j < 6; j++) { const float m = 0.5f * (P[i][j] + P[j][i]); P[i][j] = m; P[j][i] = m; }
  inject(dx);
  magFused++;
  return true;
}

float AhrsEkf::yawVar() const {
  float Rm[3][3];
  q.toMatrix(Rm);
  const float e[3] = { Rm[2][0], Rm[2][1], Rm[2][2] };
  float v = 0.0f;
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) v += e[i] * P[i][j] * e[j];
  return v;
}

bool AhrsEkf::healthy() const {
  if (!q.finite() || !bias.finite()) return false;
  for (int i = 0; i < 6; i++) {
    if (!fcFinite(P[i][i]) || P[i][i] < -1e-6f || P[i][i] > 100.0f) return false;
  }
  return true;
}
