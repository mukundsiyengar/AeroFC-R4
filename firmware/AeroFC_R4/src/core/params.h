// params.h - every tunable number of the flight controller in ONE place.
//
// The same table drives: the OLED menu (names, limits, step size), flash storage (the struct is saved as-is)
// and the simulator.  To add a parameter: add a field to Params, give it a default in paramsDefaults()
// and add a row to the table in params.cpp.  Bump PARAMS_VERSION when you change the layout.
#pragma once
#include <stdint.h>

static const uint16_t PARAMS_VERSION = 4;

enum ParamGroup : uint8_t {
  PG_RATE = 0, PG_ANGLE, PG_ALT, PG_FEATURES, PG_FAILSAFE, PG_INPUT, PG_MOTOR, PG_ESTIMATOR, PG_LOOP, PG_SENSOR, PG_COUNT
};

enum ParamKind : uint8_t { PK_FLOAT = 0, PK_BOOL, PK_ENUM };

// Failsafe action when the pilot link is lost
enum FailsafeAction : uint8_t { FS_LAND = 0, FS_HOLD = 1, FS_NONE = 2 };
// Control loop timing
enum LoopMode : uint8_t { LOOP_SYNC = 0, LOOP_100HZ = 1, LOOP_200HZ = 2 };

struct Params {
  // --- rate loop (inner): output = normalized torque per rad/s of error
  float rollP, rollI, rollD;
  float pitchP, pitchI, pitchD;
  float yawP, yawI;
  float dCutHz;            // D-term low-pass cutoff
  float rateILimit;        // integrator limit (normalized torque)
  // --- angle loop (outer)
  float angleP;            // rad/s per rad of angle error
  float maxTiltDeg;        // full stick = this tilt
  float maxRateDps;        // limit of the angle loop's rate request
  float headingP;          // rad/s per rad of heading error
  float maxYawRateDps;     // full yaw stick
  // --- altitude
  float altP;              // m/s per m
  float climbP;            // m/s^2 per m/s
  float climbI;            // m/s^2 per m (integral of climb-rate error)
  float maxClimb, maxDescent;   // m/s
  float hoverCmd;          // motor command that hovers (learned in flight); 0.66 for the Gazebo X3
  float takeoffAlt, takeoffSpeed, landSpeed, landSpeedLow;
  float maxAlt;            // geofence ceiling (m)
  // --- features (0 / 1)
  float altHoldOn, headingHoldOn, compassOn, crashDetectOn, autoTakeoffOn, telemetryPortOn;
  // --- failsafe
  float fsAction;          // FailsafeAction
  float pilotTimeout;      // s without pilot packets (only once a stream was seen)
  float crashTiltDeg;
  float idleDisarmS;       // armed on the ground without taking off -> disarm
  // --- pilot input
  float invRoll, invPitch, invYaw;
  float expo, deadband;
  // --- motors
  float motorMax;          // command sent for "full thrust" (1.0 for the 1000 rad/s Gazebo world)
  float motorIdle;         // command while armed on the ground
  // --- estimator
  float ekfGyroNoise;      // rad/s
  float ekfAccNoiseFlight; // m/s^2  (bigger = trust accelerometer less in flight)
  float magDeclDeg;        // added to the compass heading
  float baroNoise;         // m
  float gpsAidOn;          // use GPS acceleration to keep the horizon right while accelerating
  // --- loop
  float loopMode;          // LoopMode
  // --- sensor profile ("board sensitivity"): emulate a real IMU's full scale and resolution
  float sensProfileOn;
  float gyroFsDps;         // 250 / 500 / 1000 / 2000
  float accFsG;            // 2 / 4 / 8 / 16
};

struct ParamDef {
  const char *name;        // <= 10 characters, shown on the OLED
  uint16_t offset;         // byte offset inside Params
  float minV, maxV, step;
  uint8_t group;           // ParamGroup
  uint8_t kind;            // ParamKind
  uint8_t decimals;
  const char *const *enumNames;  // for PK_ENUM
};

void paramsDefaults(Params &p);
void paramsSanitize(Params &p);           // clamp everything into range (used after loading from flash)
int paramsCount();
const ParamDef &paramDef(int i);
float paramGet(const Params &p, int i);
void paramSet(Params &p, int i, float v);  // clamps to range
const char *paramGroupName(uint8_t g);
