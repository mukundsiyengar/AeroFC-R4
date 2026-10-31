#include "sensors.h"

static const uint16_t CAL_SAMPLES = 100;   // 2 s at 50 Hz

void calDefaults(CalData &c) {
  for (int i = 0; i < 3; i++) {
    c.gyroBias[i] = 0.0f;
    c.accOffset[i] = 0.0f;
    c.magOffset[i] = 0.0f;
    c.magScale[i] = 1.0f;
  }
  c.baroOffset = 0.0f;
  c.gyroValid = c.accValid = c.magValid = c.baroValid = 0;
}

// Emulates a 16-bit sensor: clip at full scale and round to the least significant bit.
static float quantize(float v, float fullScale, int16_t &raw) {
  const float lsb = fullScale / 32768.0f;
  float counts = v / lsb;
  if (counts > 32767.0f) counts = 32767.0f;
  if (counts < -32768.0f) counts = -32768.0f;
  raw = (int16_t)(counts >= 0.0f ? counts + 0.5f : counts - 0.5f);
  return (float)raw * lsb;
}

void SensorProcessor::onImu(const RawImu &r, SensorFrame &out) {
  // Gazebo body frame FLU -> our FRD:  (x, -y, -z)
  Vec3 g(r.gx, -r.gy, -r.gz);
  Vec3 a(r.ax, -r.ay, -r.az);
  out.imuValid = g.finite() && a.finite() && a.norm() > 0.5f;   // the bridge sends zeros before Gazebo data arrives
  if (!g.finite()) g = Vec3();
  if (!a.finite()) a = Vec3();

  if (prm && prm->sensProfileOn > 0.5f) {
    const float gfs = prm->gyroFsDps * FC_DEG2RAD;
    const float afs = prm->accFsG * FC_GRAVITY;
    g.x = quantize(g.x, gfs, out.rawGyro[0]);
    g.y = quantize(g.y, gfs, out.rawGyro[1]);
    g.z = quantize(g.z, gfs, out.rawGyro[2]);
    a.x = quantize(a.x, afs, out.rawAcc[0]);
    a.y = quantize(a.y, afs, out.rawAcc[1]);
    a.z = quantize(a.z, afs, out.rawAcc[2]);
  } else {
    for (int i = 0; i < 3; i++) { out.rawGyro[i] = 0; out.rawAcc[i] = 0; }
  }

  if (calKind == CAL_GYRO || calKind == CAL_LEVEL) calSampleImu(g, a);

  out.gyro = Vec3(g.x - cal->gyroBias[0], g.y - cal->gyroBias[1], g.z - cal->gyroBias[2]);
  out.acc = Vec3(a.x - cal->accOffset[0], a.y - cal->accOffset[1], a.z - cal->accOffset[2]);
}

void SensorProcessor::onMag(float mxT, float myT, float mzT, SensorFrame &out) {
  // Tesla -> microtesla, FLU -> FRD
  Vec3 m(mxT * 1e6f, -myT * 1e6f, -mzT * 1e6f);
  if (!m.finite()) return;
  if (calKind == CAL_MAG) {
    const float v[3] = { m.x, m.y, m.z };
    for (int i = 0; i < 3; i++) {
      if (calN == 0 || v[i] < mn[i]) mn[i] = v[i];
      if (calN == 0 || v[i] > mx[i]) mx[i] = v[i];
    }
    calN++;
  }
  out.mag = Vec3((m.x - cal->magOffset[0]) * cal->magScale[0],
                 (m.y - cal->magOffset[1]) * cal->magScale[1],
                 (m.z - cal->magOffset[2]) * cal->magScale[2]);
  out.magFresh = true;
}

void SensorProcessor::onBaro(float altM, SensorFrame &out) {
  if (!fcFinite(altM)) return;
  lastRawBaro = altM;
  if (calKind == CAL_BARO) {
    baroSum += altM;
    calN++;
    if (calN >= 25) {
      cal->baroOffset = baroSum / (float)calN;
      cal->baroValid = 1;
      lastCalResult = 1;
      calKind = CAL_NONE;
    }
  }
  out.baroAlt = altM - cal->baroOffset;
  out.baroFresh = true;
}

void SensorProcessor::onGps(double lat, double lon, float alt, SensorFrame &out) {
  out.lat = lat;
  out.lon = lon;
  out.gpsAlt = alt;
  out.gpsFresh = true;
}

void SensorProcessor::startCalibration(CalKind k) {
  calKind = k;
  calN = 0;
  baroSum = 0.0f;
  lastCalResult = 0;
  for (int i = 0; i < 3; i++) { sum[i] = sumSq[i] = 0.0f; mn[i] = mx[i] = 0.0f; }
}

float SensorProcessor::calProgress() const {
  if (calKind == CAL_GYRO || calKind == CAL_LEVEL) return (float)calN / (float)CAL_SAMPLES;
  if (calKind == CAL_BARO) return (float)calN / 25.0f;
  if (calKind == CAL_MAG) {
    float span = 0.0f;
    for (int i = 0; i < 3; i++) span += (mx[i] - mn[i]);
    return fcClamp(span / 120.0f, 0.0f, 1.0f);   // rough "how much have you rotated" indicator
  }
  return 0.0f;
}

void SensorProcessor::calSampleImu(const Vec3 &g, const Vec3 &a) {
  const Vec3 &v = (calKind == CAL_GYRO) ? g : a;
  sum[0] += v.x; sum[1] += v.y; sum[2] += v.z;
  sumSq[0] += v.x * v.x; sumSq[1] += v.y * v.y; sumSq[2] += v.z * v.z;
  calN++;
  if (calN < CAL_SAMPLES) return;
  const float n = (float)calN;
  float mean[3], var = 0.0f;
  for (int i = 0; i < 3; i++) {
    mean[i] = sum[i] / n;
    var += sumSq[i] / n - mean[i] * mean[i];
  }
  if (calKind == CAL_GYRO) {
    // reject if the board was moving (std dev > ~1.7 deg/s)
    if (var < 0.0009f && fcAbs(mean[0]) < 0.2f && fcAbs(mean[1]) < 0.2f && fcAbs(mean[2]) < 0.2f) {
      for (int i = 0; i < 3; i++) cal->gyroBias[i] = mean[i];
      cal->gyroValid = 1;
      lastCalResult = 1;
    } else {
      lastCalResult = -1;
    }
  } else {
    // level calibration: the average should be (0, 0, -g) when level; store the difference
    const float norm = sqrtf(mean[0] * mean[0] + mean[1] * mean[1] + mean[2] * mean[2]);
    if (var < 0.05f && norm > 0.8f * FC_GRAVITY && norm < 1.2f * FC_GRAVITY && mean[2] < 0.0f) {
      cal->accOffset[0] = mean[0];
      cal->accOffset[1] = mean[1];
      cal->accOffset[2] = mean[2] + norm;
      cal->accValid = 1;
      lastCalResult = 1;
    } else {
      lastCalResult = -1;
    }
  }
  calKind = CAL_NONE;
}

bool SensorProcessor::calFinishMag() {
  if (calKind != CAL_MAG) return false;
  // A drone can only be turned around its yaw axis in flight, so x and y must have been swept;
  // z is calibrated only if it was swept too (board tilted around), otherwise it keeps offset 0 / scale 1.
  float half[3];
  for (int i = 0; i < 3; i++) half[i] = 0.5f * (mx[i] - mn[i]);
  const bool ok = calN > 50 && half[0] > 5.0f && half[1] > 5.0f;   // field is ~25-50 uT
  if (ok) {
    const bool zSwept = half[2] > 5.0f;
    const float avg = zSwept ? (half[0] + half[1] + half[2]) / 3.0f : 0.5f * (half[0] + half[1]);
    for (int i = 0; i < 3; i++) {
      if (i == 2 && !zSwept) { cal->magOffset[2] = 0.0f; cal->magScale[2] = 1.0f; continue; }
      cal->magOffset[i] = 0.5f * (mx[i] + mn[i]);
      cal->magScale[i] = avg / half[i];
    }
    cal->magValid = 1;
  }
  lastCalResult = ok ? 1 : -1;
  calKind = CAL_NONE;
  return ok;
}
