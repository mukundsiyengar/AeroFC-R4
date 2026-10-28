// sensors.h - turns raw simulator packets into clean, calibrated sensor data in the FRD body frame.
//
//  raw (Gazebo FLU, SI units) --> frame change (x, -y, -z) --> board sensitivity (full-scale clip + LSB rounding,
//  exactly what a real MPU-6500 / 9250 would do) --> calibration (gyro bias, accel offset, mag hard/soft iron,
//  baro zero) --> SensorFrame used by the estimators.
#pragma once
#include "fc_math.h"
#include "params.h"

// Calibration data saved in flash next to the parameters
struct CalData {
  float gyroBias[3];     // rad/s (FRD)
  float accOffset[3];    // m/s^2 (FRD) - "level" calibration
  float magOffset[3];    // uT (hard iron)
  float magScale[3];     // (soft iron, diagonal)
  float baroOffset;      // m
  uint8_t gyroValid, accValid, magValid, baroValid;
};
void calDefaults(CalData &c);

struct RawImu { float ax, ay, az, gx, gy, gz; };   // as received (FLU, SI)

struct SensorFrame {
  Vec3 gyro;          // rad/s FRD, bias-corrected by calibration (the EKF removes what is left)
  Vec3 acc;           // m/s^2 FRD specific force (about (0,0,-9.81) when level and still)
  Vec3 mag;           // uT FRD, calibrated
  float baroAlt = 0;  // m, zeroed
  double lat = 0, lon = 0;
  float gpsAlt = 0;
  bool magFresh = false, baroFresh = false, gpsFresh = false;
  int16_t rawGyro[3] = { 0, 0, 0 }, rawAcc[3] = { 0, 0, 0 };   // "raw counts" as a real IMU would output
  bool imuValid = false;   // false while the simulator still sends all-zero packets
};

class SensorProcessor {
 public:
  void setup(const Params *p, CalData *c) { prm = p; cal = c; }
  void onImu(const RawImu &r, SensorFrame &out);
  void onMag(float mxT, float myT, float mzT, SensorFrame &out);
  void onBaro(float altM, SensorFrame &out);
  void onGps(double lat, double lon, float alt, SensorFrame &out);
  float rawBaro() const { return lastRawBaro; }

  // ---- manual calibration (only while disarmed) ----
  enum CalKind : uint8_t { CAL_NONE = 0, CAL_GYRO, CAL_LEVEL, CAL_MAG, CAL_BARO };
  void startCalibration(CalKind k);
  void abortCalibration() { calKind = CAL_NONE; }
  CalKind calibrating() const { return calKind; }
  float calProgress() const;            // 0..1
  bool calFinishMag();                  // compass: user presses SELECT after turning 360 deg
  int8_t lastCalResult = 0;             // 1 ok, -1 failed (moving / bad data)

 private:
  const Params *prm = 0;
  CalData *cal = 0;
  float lastRawBaro = 0.0f;
  CalKind calKind = CAL_NONE;
  uint16_t calN = 0;
  float sum[3] = { 0, 0, 0 }, sumSq[3] = { 0, 0, 0 };
  float mn[3] = { 0, 0, 0 }, mx[3] = { 0, 0, 0 };
  float baroSum = 0.0f;
  void calSampleImu(const Vec3 &gyroUncal, const Vec3 &accUncal);
};
