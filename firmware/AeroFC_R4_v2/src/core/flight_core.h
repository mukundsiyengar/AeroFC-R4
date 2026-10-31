// flight_core.h - the "brain": sensors -> estimators -> flight state machine -> controllers -> mixer.
//
// States:
//   DISARMED  motors off
//   IDLE      armed on the ground, motors at idle, waiting for TAKEOFF (auto-disarm after a while)
//   TAKEOFF   climbs to the take-off height along a smooth ramp, then -> FLYING
//   FLYING    ALT_HOLD (throttle stick = climb rate, centred = hold height) or MANUAL (throttle = collective)
//   LANDING   descends (fast, then slow near the ground), detects touch-down and disarms
//   FAILSAFE  automatic landing (or hold) because something went wrong; `reason` tells what
#pragma once
#include "fc_math.h"
#include "params.h"
#include "sensors.h"
#include "ahrs_ekf.h"
#include "alt_ekf.h"
#include "controllers.h"
#include "position.h"

enum FlightState : uint8_t { ST_DISARMED = 0, ST_IDLE, ST_TAKEOFF, ST_FLYING, ST_LANDING, ST_FAILSAFE };
enum FlightMode : uint8_t { MODE_ALT_HOLD = 0, MODE_MANUAL = 1 };
enum Reason : uint8_t {
  R_NONE = 0, R_USER, R_PILOT_LINK, R_SENSORS, R_ESTIMATOR, R_CRASH, R_STOP, R_KILL, R_IDLE, R_LANDED, R_GEOFENCE
};
enum ArmResult : int8_t {
  ARM_OK = 0, ARM_ALREADY, ARM_NO_SENSORS, ARM_EST_NOT_READY, ARM_NOT_LEVEL, ARM_CALIBRATING, ARM_TOO_SOON
};
// Command codes carried in the CONTROL_INPUT throttle field (our extension, forwarded untouched by the bridge)
enum PilotCode : uint8_t {
  CODE_ARM = 10, CODE_DISARM = 11, CODE_TAKEOFF = 12, CODE_LAND = 13, CODE_KILL = 16,
  CODE_ECO = 17, CODE_NORMAL = 18, CODE_SPORT = 19, CODE_POSHOLD_TOGGLE = 20, CODE_STICK_TOGGLE = 21
};

const char *stateName(uint8_t s);
const char *reasonName(uint8_t r);
const char *armResultName(int8_t a);

// Positions sampled once per GPS packet -> velocity and acceleration by finite differences over ~0.2 s, low-pass
// filtered.  The SAME chain is used for the GPS and for the IMU (integrated twice), so both come out with the
// same delay and can be compared directly.
struct DiffChain {
  static const int N = 32;
  float pN[N], pE[N], vN[N], vE[N];
  uint32_t t[N];
  uint8_t head = 0, count = 0;
  float velN = 0, velE = 0, accN = 0, accE = 0;
  void reset() { head = 0; count = 0; velN = velE = accN = accE = 0.0f; }
  void push(float n, float e, uint32_t now);
  bool ready() const { return count >= 50; }
};

struct PilotState {
  float roll = 0, pitch = 0, yaw = 0;   // shaped sticks -1..1 (after invert / deadband / expo)
  float climb = 0;                      // -1..1 in altitude hold
  float manualCmd = 0;                  // motor command 0..1 in manual mode
  bool manual = false;
  float rawThr = 0, rawRoll = 0, rawPitch = 0, rawYaw = 0;   // as received (for telemetry / OLED)
  uint32_t lastUs = 0;
  bool everReceived = false, streamSeen = false, linkLost = false;
  uint16_t winCount = 0;
  uint32_t winStartUs = 0;
  uint8_t rateHz = 0;
  int lastCode = 0;
};

class FlightCore {
 public:
  void init(Params *p, CalData *c);

  // ---- inputs from the simulator
  void onImu(const RawImu &r, uint64_t tsNs, uint32_t nowUs);
  void onBaro(float altM);
  void onMag(float x, float y, float z);
  void onGps(double lat, double lon, float alt);
  void onControlInput(float thr, float roll, float pitch, float yaw, uint32_t nowUs);
  void onSessionStart(uint32_t nowUs);
  void onSessionStop();
  void tick(uint32_t nowUs);            // call often (timeouts, fixed-rate loop)

  // ---- commands (OLED menu, simulator, pilot codes)
  int8_t cmdArm(uint32_t nowUs);
  void cmdDisarm(uint8_t why);
  int8_t cmdTakeoff(uint32_t nowUs);
  void cmdLand();
  void cmdKill();
  bool startCalibration(uint8_t kind);   // SensorProcessor::CalKind; refused while armed (except compass)
  bool finishCompassCalibration();
  void resetEstimator();

  bool armed() const { return state != ST_DISARMED; }
  bool motorsReady = false;             // a new motor command is ready to send

  // ---- outputs / status (read by telemetry, OLED and the simulator)
  uint8_t state = ST_DISARMED, mode = MODE_ALT_HOLD, reason = R_NONE;
  int8_t lastArmResult = ARM_OK;
  bool airborne = false, crashed = false, geofence = false, sensorsOk = false;
  float motor[4] = { 0, 0, 0, 0 };
  float collective = 0.0f;
  Vec3 torque;
  float rollSp = 0, pitchSp = 0;
  float altTarget = 0, groundAlt = 0, takeoffGoal = 0;
  float tiltDeg = 0;
  float imuHz = 0, ctrlHz = 0;
  uint32_t loopUs = 0;                  // filled in by the app (time of one control step)
  float homeNorth = 0, homeEast = 0;    // GPS position relative to the first fix (m)
  bool gpsHome = false;
  // ---- GPS position hold
  PositionEstimator pos;                // position / speed in the GPS frame
  PositionController posCtl;
  bool posReady = false;                // GPS + compass good enough for position hold
  uint64_t lastImuTimestampNs() const { return lastTs_; }   // of the sensor packet the motor outputs answer
  bool posActive = false;               // position hold is flying the drone right now
  bool posFailed = false;               // runaway detected: switched off for 5 s, then it tries again
  uint8_t posState = POS_OFF;
  const char *posWhy = "";              // why position hold is not ready (OLED)
  float ceilingAbs = 1e9f;              // height ceiling (absolute altitude), huge when Alt limit is OFF
  bool atCeiling = false;
  DiffChain gpsChain;                   // GPS position -> velocity / acceleration (true north frame)
  DiffChain imuChain;                   // IMU acceleration integrated twice -> same chain (EKF heading frame)
  float aidYawOffset = 0.0f;            // learned angle between the GPS frame and the EKF heading frame (rad)
  float aidErrN = 0.0f, aidErrE = 0.0f; // slow horizontal acceleration error of the IMU path (= attitude error x g)
  float aidP[3][3];                     // covariance of [yaw offset, errN, errE]
  float aidPrevAccN = 0.0f, aidPrevAccE = 0.0f;
  uint8_t aidBigN_ = 0;                 // consecutive measurements of a big heading offset (3+ = real evidence)
  float aidBigAng_ = 0.0f;              // ... and their average (rad)
  bool gpsAidOk = false;

  Params *prm = 0;
  CalData *cal = 0;
  SensorProcessor sens;
  SensorFrame sf;
  AhrsEkf ekf;
  AltEkf altf;
  AttitudeController att;
  AltitudeController altc;
  MixerOut mix;
  PilotState pilot;

 private:
  void runControl(float dt, bool newImu, uint32_t nowUs);
  void flightLogic(float dt, uint32_t nowUs);
  void handleCode(int code, uint32_t nowUs);
  void enterFlying(uint8_t m);
  void setOutputsOff();

  uint64_t lastTs_ = 0;
  uint32_t lastImuUs_ = 0, lastCtrlUs_ = 0, disarmUs_ = 0;
  bool haveImu_ = false, newImu_ = false, altInit_ = false, holdCapture_ = false;
  bool pendingAutoTakeoff_ = false;
  uint32_t autoTakeoffUntil_ = 0;
  float idleTimer_ = 0, landTimer_ = 0, crashTimer_ = 0, takeoffTimer_ = 0;
  float descentSp_ = 0;     // speed the climb stick is asking to go down at (m/s), builds up gradually
  uint16_t imuCount_ = 0, ctrlCount_ = 0;
  uint32_t rateWinUs_ = 0;
  double lat0_ = 0, lon0_ = 0;
  uint32_t lastGpsUs_ = 0;
  double lastFixLat_ = 0, lastFixLon_ = 0;
  uint32_t lastFixUs_ = 0;
  float blendRoll_ = 0, blendPitch_ = 0, blendT_ = 1.0f;
  bool wasPosActive_ = false;
  float posFailT_ = 0.0f, posGoodT_ = 0.0f;   // since position hold failed / since it holds well
  uint8_t posFailCount_ = 0;
  float imuVelN_ = 0, imuVelE_ = 0, imuPosN_ = 0, imuPosE_ = 0;   // free integration of the IMU (drift is fine)
  float imuAccN_ = 0, imuAccE_ = 0;                                // current horizontal acceleration (EKF frame)
};
