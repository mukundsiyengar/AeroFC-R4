#include "params.h"
#include <stddef.h>
#include <string.h>

void paramsDefaults(Params &p) {
  memset(&p, 0, sizeof(p));
  p.speedMode = SPEED_NORMAL;
  p.posHoldOn = 1;
  p.posStick = STICK_SPEED;
  p.altHoldOn = 1; p.headingHoldOn = 1;
  p.rollP = 0.055f;  p.rollI = 0.10f;  p.rollD = 0.0015f;
  p.pitchP = 0.17f;  p.pitchI = 0.30f; p.pitchD = 0.0045f;
  p.yawP = 0.60f;    p.yawI = 0.25f;
  p.dCutHz = 8.0f;
  p.rateILimit = 0.10f;
  p.angleP = 4.0f;
  p.maxRateDps = 150.0f;
  p.headingP = 1.5f;
  p.posP = 0.7f;
  p.velP = 1.6f;
  p.velI = 0.3f;
  p.altP = 1.2f;
  p.climbP = 3.0f;
  p.climbI = 1.0f;
  p.hoverCmd = 0.66f;
  p.takeoffAlt = 1.5f;
  p.takeoffSpeed = 0.8f;
  p.landSpeed = 0.6f;
  p.landSpeedLow = 0.25f;
  p.landSpeedHigh = 2.0f;
  p.altLimitOn = 0;
  p.maxAlt = 120.0f;
  p.compassOn = 1; p.crashDetectOn = 1; p.autoTakeoffOn = 1; p.telemetryPortOn = 0;
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
  p.magDeclDeg = 90.8f;   // Gazebo (gz-sim 8): its compass points 90.8 deg away from GPS north in this world
  p.baroNoise = 0.10f;
  p.gpsAidOn = 1;
  p.loopMode = LOOP_SYNC;
  p.sensProfileOn = 1;
  p.gyroFsDps = 2000.0f;
  p.accFsG = 8.0f;
}

//                                       speed  tilt  brake  climb  descent  yaw
static const SpeedProfile kProfiles[3] = { { 2.0f, 15.0f, 12.0f, 1.0f, 0.7f, 45.0f },     // ECO
                                           { 5.0f, 25.0f, 20.0f, 2.0f, 1.0f, 90.0f },     // NORMAL
                                           { 10.0f, 35.0f, 30.0f, 4.0f, 2.5f, 150.0f } }; // SPORT

const SpeedProfile &speedProfile(const Params &p) {
  int m = (int)(p.speedMode + 0.5f);
  if (m < 0) m = 0;
  if (m > 2) m = 2;
  return kProfiles[m];
}

static const char *const kSpeedNames[] = { "ECO", "NORMAL", "SPORT" };
static const char *const kStickNames[] = { "SPEED", "ANGLE" };
static const char *const kFsNames[] = { "LAND", "HOLD", "NONE" };
static const char *const kLoopNames[] = { "SYNC", "100Hz", "200Hz" };

const char *speedModeName(uint8_t m) { return m < 3 ? kSpeedNames[m] : "?"; }

#define PO(f) (uint16_t)offsetof(Params, f)
static const ParamDef kDefs[] = {
  // ---------------------------------------------------------------- flight modes
  { "Speed",     PO(speedMode),    0, 2, 1, PG_MODES, PK_ENUM, 0, kSpeedNames,
    "ECO: gentle (2 m/s, 15 deg). NORMAL: 5 m/s, 25 deg. SPORT: fast (10 m/s, 35 deg). Also keys 1/2/3." },
  { "Pos hold",  PO(posHoldOn),    0, 1, 1, PG_MODES, PK_BOOL, 0, 0,
    "GPS position hold: let go of the sticks and the drone brakes and stays on its spot, even in wind." },
  { "Stick",     PO(posStick),     0, 1, 1, PG_MODES, PK_ENUM, 0, kStickNames,
    "With position hold: SPEED = stick sets flying speed (like DJI). ANGLE = stick sets tilt. Both stop on release." },
  { "Alt hold",  PO(altHoldOn),    0, 1, 1, PG_MODES, PK_BOOL, 0, 0,
    "ON: throttle centred holds the height. OFF: throttle centred holds the climb rate (height can drift)." },
  { "Head hold", PO(headingHoldOn),0, 1, 1, PG_MODES, PK_BOOL, 0, 0,
    "Heading hold: after a turn the drone keeps pointing where the turn stopped." },
  // ---------------------------------------------------------------- rate PID
  { "Roll P",    PO(rollP),        0.0f,   0.5f,   0.005f, PG_RATE, PK_FLOAT, 3, 0,
    "Roll rate P: how hard it fights roll-speed errors. Too high = fast shaking, too low = soft." },
  { "Roll I",    PO(rollI),        0.0f,   1.0f,   0.01f,  PG_RATE, PK_FLOAT, 2, 0,
    "Roll rate I: removes slow, steady roll errors (an off-centre weight). Too high = slow wobble." },
  { "Roll D",    PO(rollD),        0.0f,   0.02f,  0.0005f,PG_RATE, PK_FLOAT, 4, 0,
    "Roll rate D: damping, brakes fast roll movements. Too high = noisy motors." },
  { "Pitch P",   PO(pitchP),       0.0f,   1.0f,   0.01f,  PG_RATE, PK_FLOAT, 3, 0,
    "Pitch rate P (the X3 is heavier around pitch, so it is bigger than roll)." },
  { "Pitch I",   PO(pitchI),       0.0f,   2.0f,   0.02f,  PG_RATE, PK_FLOAT, 2, 0,
    "Pitch rate I: removes slow, steady pitch errors." },
  { "Pitch D",   PO(pitchD),       0.0f,   0.05f,  0.0005f,PG_RATE, PK_FLOAT, 4, 0,
    "Pitch rate D: damping of fast pitch movements." },
  { "Yaw P",     PO(yawP),         0.0f,   3.0f,   0.05f,  PG_RATE, PK_FLOAT, 2, 0,
    "Yaw rate P: how firmly it holds the turning speed." },
  { "Yaw I",     PO(yawI),         0.0f,   2.0f,   0.05f,  PG_RATE, PK_FLOAT, 2, 0,
    "Yaw rate I: removes a slow unwanted spin." },
  { "D cut Hz",  PO(dCutHz),       2.0f,   25.0f,  1.0f,   PG_RATE, PK_FLOAT, 0, 0,
    "Filter for the D terms. Lower = smoother but slower." },
  { "I limit",   PO(rateILimit),   0.0f,   0.3f,   0.01f,  PG_RATE, PK_FLOAT, 2, 0,
    "Most the I terms may add (stops them from winding up)." },
  // ---------------------------------------------------------------- angle / heading
  { "Angle P",   PO(angleP),       0.5f,   10.0f,  0.1f,   PG_ANGLE, PK_FLOAT, 1, 0,
    "Angle P: how fast it moves to the wanted tilt. Too high = wobble, too low = lazy." },
  { "Max rate",  PO(maxRateDps),   30.0f,  360.0f, 10.0f,  PG_ANGLE, PK_FLOAT, 0, 0,
    "Fastest roll/pitch rotation the angle loop may ask for (deg/s)." },
  { "Head P",    PO(headingP),     0.0f,   5.0f,   0.1f,   PG_ANGLE, PK_FLOAT, 1, 0,
    "Heading hold strength: how fast it turns back to the held heading." },
  // ---------------------------------------------------------------- position hold PID
  { "Pos P",     PO(posP),         0.1f,   3.0f,   0.05f,  PG_POSITION, PK_FLOAT, 2, 0,
    "Position P: speed asked for per metre away from the hold point. Too high = circling." },
  { "Vel P",     PO(velP),         0.2f,   5.0f,   0.1f,   PG_POSITION, PK_FLOAT, 1, 0,
    "Speed P: how hard it tilts to correct a speed error." },
  { "Vel I",     PO(velI),         0.0f,   2.0f,   0.05f,  PG_POSITION, PK_FLOAT, 2, 0,
    "Speed I: learns the wind push so the drone stays exactly on its spot." },
  // ---------------------------------------------------------------- altitude
  { "Alt P",     PO(altP),         0.1f,   4.0f,   0.1f,   PG_ALT, PK_FLOAT, 1, 0,
    "Height P: climb rate asked for per metre of height error." },
  { "Climb P",   PO(climbP),       0.5f,   8.0f,   0.1f,   PG_ALT, PK_FLOAT, 1, 0,
    "Climb-rate P: how hard thrust changes to reach the wanted climb rate." },
  { "Climb I",   PO(climbI),       0.0f,   4.0f,   0.1f,   PG_ALT, PK_FLOAT, 1, 0,
    "Climb-rate I: learns the exact hover thrust (shown as Hover cmd)." },
  { "Hover",     PO(hoverCmd),     0.2f,   0.9f,   0.005f, PG_ALT, PK_FLOAT, 3, 0,
    "Starting guess of the motor command that hovers (0.66 for the Gazebo X3)." },
  { "TO alt",    PO(takeoffAlt),   0.5f,   10.0f,  0.1f,   PG_ALT, PK_FLOAT, 1, 0,
    "Take-off height (m above the take-off point)." },
  { "TO speed",  PO(takeoffSpeed), 0.2f,   3.0f,   0.1f,   PG_ALT, PK_FLOAT, 1, 0,
    "Climb speed during the automatic take-off (m/s)." },
  { "Land spd",  PO(landSpeed),    0.2f,   2.0f,   0.05f,  PG_ALT, PK_FLOAT, 2, 0,
    "Landing descent speed between 20 m and 1 m (m/s)." },
  { "Land low",  PO(landSpeedLow), 0.1f,   0.8f,   0.05f,  PG_ALT, PK_FLOAT, 2, 0,
    "Slow landing speed for the last metre (m/s)." },
  { "Land high", PO(landSpeedHigh),0.5f,   5.0f,   0.1f,   PG_ALT, PK_FLOAT, 1, 0,
    "Landing descent speed above 20 m (m/s)." },
  { "Alt limit", PO(altLimitOn),   0, 1, 1, PG_ALT, PK_BOOL, 0, 0,
    "Height ceiling ON/OFF. ON: the drone cannot climb above Max alt. OFF: no limit." },
  { "Max alt",   PO(maxAlt),       2.0f,   1000.0f,5.0f,   PG_ALT, PK_FLOAT, 0, 0,
    "Ceiling in metres above the take-off point (used when Alt limit is ON). Hold UP/DOWN to go fast." },
  // ---------------------------------------------------------------- features
  { "Compass",   PO(compassOn),    0, 1, 1, PG_FEATURES, PK_BOOL, 0, 0,
    "Use the magnetometer for the heading. Needed for position hold." },
  { "Crash det", PO(crashDetectOn),0, 1, 1, PG_FEATURES, PK_BOOL, 0, 0,
    "Crash detection: motors off when the drone tips over (more than Crash deg)." },
  { "Auto TO",   PO(autoTakeoffOn),0, 1, 1, PG_FEATURES, PK_BOOL, 0, 0,
    "Take off by itself to TO alt when the simulator sends START." },
  { "Telem D1",  PO(telemetryPortOn),0,1, 1, PG_FEATURES, PK_BOOL, 0, 0,
    "Send telemetry on pin D1 (460800 baud) for a USB-TTL adapter. Leave OFF if nothing is connected." },
  // ---------------------------------------------------------------- failsafe
  { "RC lost",   PO(fsAction),     0, 2, 1, PG_FAILSAFE, PK_ENUM, 0, kFsNames,
    "When the pilot link is lost: LAND = safe landing on the spot, HOLD = hover in place, NONE = nothing." },
  { "Link t/o",  PO(pilotTimeout), 0.5f,   10.0f,  0.5f,   PG_FAILSAFE, PK_FLOAT, 1, 0,
    "Seconds without pilot packets before the RC-lost action starts." },
  { "Crash deg", PO(crashTiltDeg), 30.0f,  90.0f,  5.0f,   PG_FAILSAFE, PK_FLOAT, 0, 0,
    "Tilt that counts as a crash (deg)." },
  { "Idle disr", PO(idleDisarmS),  3.0f,   60.0f,  1.0f,   PG_FAILSAFE, PK_FLOAT, 0, 0,
    "Disarm by itself after this many seconds armed on the ground without taking off." },
  // ---------------------------------------------------------------- input
  { "Inv roll",  PO(invRoll),      0, 1, 1, PG_INPUT, PK_BOOL, 0, 0, "Reverse the roll stick." },
  { "Inv pitch", PO(invPitch),     0, 1, 1, PG_INPUT, PK_BOOL, 0, 0,
    "Reverse the pitch stick (if forward/back is swapped with another joystick tool)." },
  { "Inv yaw",   PO(invYaw),       0, 1, 1, PG_INPUT, PK_BOOL, 0, 0, "Reverse the yaw (turn) stick." },
  { "Expo",      PO(expo),         0.0f,   1.0f,   0.05f,  PG_INPUT, PK_FLOAT, 2, 0,
    "Softer stick near the centre (0 = linear)." },
  { "Deadband",  PO(deadband),     0.0f,   0.2f,   0.01f,  PG_INPUT, PK_FLOAT, 2, 0,
    "Stick movement that is ignored around the centre." },
  // ---------------------------------------------------------------- motors
  { "Motor max", PO(motorMax),     0.5f,   1.0f,   0.01f,  PG_MOTOR, PK_FLOAT, 2, 0,
    "Highest motor command. 1.00 for the 1000 rad/s Gazebo world, 0.80 for an 800 rad/s world." },
  { "Idle",      PO(motorIdle),    0.0f,   0.2f,   0.01f,  PG_MOTOR, PK_FLOAT, 2, 0,
    "Motor command while armed on the ground." },
  // ---------------------------------------------------------------- estimator
  { "Gyro nse",  PO(ekfGyroNoise), 0.0005f,0.1f,   0.0005f,PG_ESTIMATOR, PK_FLOAT, 4, 0,
    "EKF: how noisy the gyro is (rad/s). Bigger = trusts the accelerometer more." },
  { "Acc nse",   PO(ekfAccNoiseFlight),0.2f,10.0f, 0.1f,   PG_ESTIMATOR, PK_FLOAT, 1, 0,
    "EKF: accelerometer noise in flight (m/s2). Bigger = trusts the gyro more." },
  { "Mag decl",  PO(magDeclDeg),   -180.0f, 180.0f, 0.5f,  PG_ESTIMATOR, PK_FLOAT, 1, 0,
    "Angle added to the compass heading (deg). 90.8 = right for Gazebo's compass. If wrong, it is learned in flight." },
  { "Baro nse",  PO(baroNoise),    0.01f,  2.0f,   0.01f,  PG_ESTIMATOR, PK_FLOAT, 2, 0,
    "Altitude filter: barometer noise (m)." },
  { "GPS aid",   PO(gpsAidOn),     0, 1, 1, PG_ESTIMATOR, PK_BOOL, 0, 0,
    "Use GPS so accelerating or braking does not fool the horizon." },
  { "Loop",      PO(loopMode),     0, 2, 1, PG_LOOP, PK_ENUM, 0, kLoopNames,
    "SYNC = one control step per sensor packet (best). 100/200 Hz = predict between packets." },
  { "Profile",   PO(sensProfileOn),0, 1, 1, PG_SENSOR, PK_BOOL, 0, 0,
    "Board sensitivity: treat the sensors like a real 16-bit IMU (clip at full scale, 1-LSB steps)." },
  { "Gyro FS",   PO(gyroFsDps),    250.0f, 2000.0f,250.0f, PG_SENSOR, PK_FLOAT, 0, 0,
    "Gyro full scale (deg/s). Bigger = more range, coarser steps." },
  { "Acc FS",    PO(accFsG),       2.0f,   16.0f,  2.0f,   PG_SENSOR, PK_FLOAT, 0, 0,
    "Accelerometer full scale (g)." },
};
#undef PO

int paramsCount() { return (int)(sizeof(kDefs) / sizeof(kDefs[0])); }
const ParamDef &paramDef(int i) { return kDefs[i]; }

int paramIndexOf(uint16_t offset) {
  for (int i = 0; i < paramsCount(); i++)
    if (kDefs[i].offset == offset) return i;
  return -1;
}

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
  static const char *const names[PG_COUNT] = { "Flight modes", "Rate PID", "Angle/Yaw", "Position PID", "Altitude",
                                               "Features", "Failsafe", "Input", "Motors", "Estimator", "Loop rate",
                                               "Sensor prof" };
  return g < PG_COUNT ? names[g] : "?";
}

const char *paramGroupHelp(uint8_t g) {
  static const char *const help[PG_COUNT] = {
    "Speed mode (ECO/NORMAL/SPORT), position hold, stick style, altitude and heading hold.",
    "Inner loop: turns rotation-speed errors into motor differences. Change in small steps.",
    "Outer loop: turns angle errors into rotation speed; heading hold strength.",
    "Gains of GPS position hold: position -> speed -> tilt. Vel I learns the wind.",
    "Height control, take-off and landing speeds, and the height ceiling (Alt limit).",
    "Compass, crash detection, take-off on START, telemetry port D1.",
    "What happens when the pilot link (RC) is lost, crash tilt, idle disarm.",
    "Stick direction (invert), expo and deadband.",
    "Highest motor command and idle command.",
    "Noise settings of the attitude and altitude filters, declination, GPS aid.",
    "Control loop timing. Also shows the measured sensor rate and step time.",
    "Board sensitivity: emulate a real 16-bit IMU (full scale and resolution).",
  };
  return g < PG_COUNT ? help[g] : "";
}
