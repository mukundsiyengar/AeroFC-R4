#include "params.h"
#include <stddef.h>
#include <string.h>

void paramsDefaults(Params &p) {
  memset(&p, 0, sizeof(p));
  p.rollP = 0.055f;  p.rollI = 0.10f;  p.rollD = 0.0015f;
  p.pitchP = 0.17f;  p.pitchI = 0.30f; p.pitchD = 0.0045f;
  p.yawP = 0.60f;    p.yawI = 0.25f;
  p.dCutHz = 8.0f;
  p.rateILimit = 0.10f;
  p.angleP = 4.0f;
  p.maxTiltDeg = 25.0f;
  p.maxRateDps = 150.0f;
  p.headingP = 1.5f;
  p.maxYawRateDps = 90.0f;
  p.altP = 1.2f;
  p.climbP = 3.0f;
  p.climbI = 1.0f;
  p.maxClimb = 1.0f;
  p.maxDescent = 0.7f;
  p.hoverCmd = 0.66f;
  p.takeoffAlt = 1.5f;
  p.takeoffSpeed = 0.8f;
  p.landSpeed = 0.6f;
  p.landSpeedLow = 0.25f;
  p.maxAlt = 10.0f;
  p.altHoldOn = 1; p.headingHoldOn = 1; p.compassOn = 1; p.crashDetectOn = 1; p.autoTakeoffOn = 1; p.telemetryPortOn = 1;
  p.fsAction = FS_LAND;
  p.pilotTimeout = 2.0f;
  p.crashTiltDeg = 70.0f;
  p.idleDisarmS = 15.0f;
  p.invRoll = 0; p.invPitch = 0; p.invYaw = 0;
  p.expo = 0.0f;
  p.deadband = 0.02f;
  p.motorMax = 1.0f;
  p.motorIdle = 0.05f;
  p.ekfGyroNoise = 0.005f;
  p.ekfAccNoiseFlight = 3.0f;
  p.magDeclDeg = 0.0f;
  p.baroNoise = 0.10f;
  p.gpsAidOn = 1;
  p.loopMode = LOOP_SYNC;
  p.sensProfileOn = 1;
  p.gyroFsDps = 2000.0f;
  p.accFsG = 8.0f;
}

static const char *const kFsNames[] = { "LAND", "HOLD", "NONE" };
static const char *const kLoopNames[] = { "SYNC", "100Hz", "200Hz" };

#define PO(f) (uint16_t)offsetof(Params, f)
static const ParamDef kDefs[] = {
  // name        offset            min     max     step    group         kind     dec enum
  { "Roll P",    PO(rollP),        0.0f,   0.5f,   0.005f, PG_RATE,      PK_FLOAT, 3, 0 },
  { "Roll I",    PO(rollI),        0.0f,   1.0f,   0.01f,  PG_RATE,      PK_FLOAT, 2, 0 },
  { "Roll D",    PO(rollD),        0.0f,   0.02f,  0.0005f,PG_RATE,      PK_FLOAT, 4, 0 },
  { "Pitch P",   PO(pitchP),       0.0f,   1.0f,   0.01f,  PG_RATE,      PK_FLOAT, 3, 0 },
  { "Pitch I",   PO(pitchI),       0.0f,   2.0f,   0.02f,  PG_RATE,      PK_FLOAT, 2, 0 },
  { "Pitch D",   PO(pitchD),       0.0f,   0.05f,  0.0005f,PG_RATE,      PK_FLOAT, 4, 0 },
  { "Yaw P",     PO(yawP),         0.0f,   3.0f,   0.05f,  PG_RATE,      PK_FLOAT, 2, 0 },
  { "Yaw I",     PO(yawI),         0.0f,   2.0f,   0.05f,  PG_RATE,      PK_FLOAT, 2, 0 },
  { "D cut Hz",  PO(dCutHz),       2.0f,   25.0f,  1.0f,   PG_RATE,      PK_FLOAT, 0, 0 },
  { "I limit",   PO(rateILimit),   0.0f,   0.3f,   0.01f,  PG_RATE,      PK_FLOAT, 2, 0 },
  { "Angle P",   PO(angleP),       0.5f,   10.0f,  0.1f,   PG_ANGLE,     PK_FLOAT, 1, 0 },
  { "Max tilt",  PO(maxTiltDeg),   5.0f,   45.0f,  1.0f,   PG_ANGLE,     PK_FLOAT, 0, 0 },
  { "Max rate",  PO(maxRateDps),   30.0f,  360.0f, 10.0f,  PG_ANGLE,     PK_FLOAT, 0, 0 },
  { "Head P",    PO(headingP),     0.0f,   5.0f,   0.1f,   PG_ANGLE,     PK_FLOAT, 1, 0 },
  { "Yaw rate",  PO(maxYawRateDps),10.0f,  180.0f, 5.0f,   PG_ANGLE,     PK_FLOAT, 0, 0 },
  { "Alt P",     PO(altP),         0.1f,   4.0f,   0.1f,   PG_ALT,       PK_FLOAT, 1, 0 },
  { "Climb P",   PO(climbP),       0.5f,   8.0f,   0.1f,   PG_ALT,       PK_FLOAT, 1, 0 },
  { "Climb I",   PO(climbI),       0.0f,   4.0f,   0.1f,   PG_ALT,       PK_FLOAT, 1, 0 },
  { "Max up",    PO(maxClimb),     0.2f,   3.0f,   0.1f,   PG_ALT,       PK_FLOAT, 1, 0 },
  { "Max down",  PO(maxDescent),   0.2f,   2.0f,   0.1f,   PG_ALT,       PK_FLOAT, 1, 0 },
  { "Hover",     PO(hoverCmd),     0.2f,   0.9f,   0.005f, PG_ALT,       PK_FLOAT, 3, 0 },
  { "TO alt",    PO(takeoffAlt),   0.5f,   5.0f,   0.1f,   PG_ALT,       PK_FLOAT, 1, 0 },
  { "TO speed",  PO(takeoffSpeed), 0.2f,   2.0f,   0.1f,   PG_ALT,       PK_FLOAT, 1, 0 },
  { "Land spd",  PO(landSpeed),    0.2f,   1.5f,   0.05f,  PG_ALT,       PK_FLOAT, 2, 0 },
  { "Land low",  PO(landSpeedLow), 0.1f,   0.8f,   0.05f,  PG_ALT,       PK_FLOAT, 2, 0 },
  { "Max alt",   PO(maxAlt),       2.0f,   50.0f,  1.0f,   PG_ALT,       PK_FLOAT, 0, 0 },
  { "Alt hold",  PO(altHoldOn),    0, 1, 1, PG_FEATURES, PK_BOOL, 0, 0 },
  { "Head hold", PO(headingHoldOn),0, 1, 1, PG_FEATURES, PK_BOOL, 0, 0 },
  { "Compass",   PO(compassOn),    0, 1, 1, PG_FEATURES, PK_BOOL, 0, 0 },
  { "Crash det", PO(crashDetectOn),0, 1, 1, PG_FEATURES, PK_BOOL, 0, 0 },
  { "Auto TO",   PO(autoTakeoffOn),0, 1, 1, PG_FEATURES, PK_BOOL, 0, 0 },
  { "Telem D1",  PO(telemetryPortOn),0,1, 1, PG_FEATURES, PK_BOOL, 0, 0 },
  { "FS action", PO(fsAction),     0, 2, 1, PG_FAILSAFE, PK_ENUM, 0, kFsNames },
  { "Link t/o",  PO(pilotTimeout), 0.5f,   10.0f,  0.5f,   PG_FAILSAFE,  PK_FLOAT, 1, 0 },
  { "Crash deg", PO(crashTiltDeg), 30.0f,  90.0f,  5.0f,   PG_FAILSAFE,  PK_FLOAT, 0, 0 },
  { "Idle disr", PO(idleDisarmS),  3.0f,   60.0f,  1.0f,   PG_FAILSAFE,  PK_FLOAT, 0, 0 },
  { "Inv roll",  PO(invRoll),      0, 1, 1, PG_INPUT, PK_BOOL, 0, 0 },
  { "Inv pitch", PO(invPitch),     0, 1, 1, PG_INPUT, PK_BOOL, 0, 0 },
  { "Inv yaw",   PO(invYaw),       0, 1, 1, PG_INPUT, PK_BOOL, 0, 0 },
  { "Expo",      PO(expo),         0.0f,   1.0f,   0.05f,  PG_INPUT,     PK_FLOAT, 2, 0 },
  { "Deadband",  PO(deadband),     0.0f,   0.2f,   0.01f,  PG_INPUT,     PK_FLOAT, 2, 0 },
  { "Motor max", PO(motorMax),     0.5f,   1.0f,   0.01f,  PG_MOTOR,     PK_FLOAT, 2, 0 },
  { "Idle",      PO(motorIdle),    0.0f,   0.2f,   0.01f,  PG_MOTOR,     PK_FLOAT, 2, 0 },
  { "Gyro nse",  PO(ekfGyroNoise), 0.0005f,0.1f,   0.0005f,PG_ESTIMATOR, PK_FLOAT, 4, 0 },
  { "Acc nse",   PO(ekfAccNoiseFlight),0.2f,10.0f, 0.1f,   PG_ESTIMATOR, PK_FLOAT, 1, 0 },
  { "Mag decl",  PO(magDeclDeg),   -30.0f, 30.0f,  0.5f,   PG_ESTIMATOR, PK_FLOAT, 1, 0 },
  { "Baro nse",  PO(baroNoise),    0.01f,  2.0f,   0.01f,  PG_ESTIMATOR, PK_FLOAT, 2, 0 },
  { "GPS aid",   PO(gpsAidOn),     0, 1, 1, PG_ESTIMATOR, PK_BOOL, 0, 0 },
  { "Loop",      PO(loopMode),     0, 2, 1, PG_LOOP, PK_ENUM, 0, kLoopNames },
  { "Profile",   PO(sensProfileOn),0, 1, 1, PG_SENSOR, PK_BOOL, 0, 0 },
  { "Gyro FS",   PO(gyroFsDps),    250.0f, 2000.0f,250.0f, PG_SENSOR,    PK_FLOAT, 0, 0 },
  { "Acc FS",    PO(accFsG),       2.0f,   16.0f,  2.0f,   PG_SENSOR,    PK_FLOAT, 0, 0 },
};
#undef PO

int paramsCount() { return (int)(sizeof(kDefs) / sizeof(kDefs[0])); }
const ParamDef &paramDef(int i) { return kDefs[i]; }

static float *slot(Params &p, int i) { return (float *)((uint8_t *)&p + kDefs[i].offset); }
static const float *slot(const Params &p, int i) { return (const float *)((const uint8_t *)&p + kDefs[i].offset); }

float paramGet(const Params &p, int i) { return *slot(p, i); }

void paramSet(Params &p, int i, float v) {
  const ParamDef &d = kDefs[i];
  if (!(v == v)) v = d.minV;   // NaN -> minimum
  if (v < d.minV) v = d.minV;
  if (v > d.maxV) v = d.maxV;
  if (d.kind != PK_FLOAT) v = (float)(int)(v + 0.5f);
  *slot(p, i) = v;
}

void paramsSanitize(Params &p) {
  for (int i = 0; i < paramsCount(); i++) paramSet(p, i, paramGet(p, i));
}

const char *paramGroupName(uint8_t g) {
  static const char *const names[PG_COUNT] = { "Rate PID", "Angle/Yaw", "Altitude", "Features", "Failsafe",
                                               "Input", "Motors", "Estimator", "Loop rate", "Sensor prof" };
  return g < PG_COUNT ? names[g] : "?";
}
