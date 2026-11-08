#include "fc_app.h"
#include <string.h>

void FcApp::init() {
  paramsDefaults(params);
  calDefaults(cal);
  core.init(&params, &cal);
  parser.reset();
  session = SES_WAITING;
}

void FcApp::restoreDefaults() {
  paramsDefaults(params);
  calDefaults(cal);
  core.resetEstimator();
}

// ------------------------------------------------------------------ receive
void FcApp::feed(const uint8_t *data, size_t n, uint32_t nowUs) {
  hil::Packet pk;
  for (size_t i = 0; i < n; i++) {
    parser.push(data[i]);
    while (parser.next(pk)) handle(pk, nowUs);
  }
}

void FcApp::handle(const hil::Packet &pk, uint32_t nowUs) {
  lastRxUs = nowUs;
  const uint8_t *p = pk.payload;
  switch (pk.type) {
    case hil::SENSOR_IMU:
      if (pk.length < 24) break;
      rxCount[0]++;
      lastSensorUs = nowUs;
      {
        RawImu r;
        r.ax = hil::getF32(p); r.ay = hil::getF32(p + 4); r.az = hil::getF32(p + 8);
        r.gx = hil::getF32(p + 12); r.gy = hil::getF32(p + 16); r.gz = hil::getF32(p + 20);
        core.onImu(r, pk.timestampNs, nowUs);
      }
      break;
    case hil::SENSOR_BARO:
      if (pk.length < 4) break;
      rxCount[1]++;
      core.onBaro(hil::getF32(p));
      break;
    case hil::SENSOR_MAG:
      if (pk.length < 12) break;
      rxCount[2]++;
      core.onMag(hil::getF32(p), hil::getF32(p + 4), hil::getF32(p + 8));
      break;
    case hil::SENSOR_GPS:
      if (pk.length < 20) break;
      rxCount[3]++;
      core.onGps(hil::getF64(p), hil::getF64(p + 8), hil::getF32(p + 16));
      break;
    case hil::CONTROL_INPUT:
      if (pk.length < 16) break;
      rxCount[4]++;
      core.onControlInput(hil::getF32(p), hil::getF32(p + 4), hil::getF32(p + 8), hil::getF32(p + 12), nowUs);
      break;
    case hil::HELLO:
      rxCount[5]++;
      needHelloAck_ = true;
      if (session != SES_RUNNING) session = SES_HANDSHAKE;
      break;
    case hil::START:
      rxCount[6]++;
      session = SES_RUNNING;
      lastHbUs_ = nowUs;
      core.onSessionStart(nowUs);
      break;
    case hil::STOP:
      rxCount[7]++;
      session = SES_STOPPED;
      core.onSessionStop();
      break;
    default:
      break;   // HEARTBEAT, STATUS, ERROR, ACTUATOR echo: nothing to do
  }
}

// ------------------------------------------------------------------ timers
void FcApp::tick(uint32_t nowUs) {
  core.tick(nowUs);
}

// ------------------------------------------------------------------ transmit
size_t FcApp::emit(uint8_t *buf, size_t cap, uint8_t type, const uint8_t *payload, uint16_t len, uint32_t nowUs,
                   uint32_t &seq) {
  const size_t n = hil::build(buf, cap, type, seq, (uint64_t)nowUs * 1000ull, payload, len);
  if (n) seq++;
  return n;
}

size_t FcApp::nextUsbPacket(uint8_t *buf, size_t cap, uint32_t nowUs) {
  // Priority: HELLO_ACK > motor command > heartbeat > HELLO > STATUS.  Only whole packets are ever written.
  if (needHelloAck_) {
    const size_t n = emit(buf, cap, hil::HELLO_ACK, (const uint8_t *)hil::HELLO_TEXT, 10, nowUs, seq_);
    if (n) needHelloAck_ = false;
    return n;
  }
  if (core.motorsReady) {
    uint8_t pl[16];
    for (int i = 0; i < 4; i++) hil::putF32(pl + 4 * i, fcClamp(core.motor[i], 0.0f, 1.0f));
    // carries the timestamp of the sensor packet it answers, so the simulator can measure the control delay exactly
    // (the official bridge does not read it)
    const size_t n = hil::build(buf, cap, hil::ACTUATOR_COMMAND, seq_, core.lastImuTimestampNs(), pl, 16);
    if (n) { seq_++; core.motorsReady = false; txActuators++; }
    return n;
  }
  // When the sensors stopped for > 1 s nobody may be listening: unread data waits on the way and all arrives at once
  // when a bridge starts again, so then only the short HELLO is sent, once a second.
  const bool sensorsStopped = (uint32_t)(nowUs - lastSensorUs) > 1000000u;
  if (session == SES_RUNNING && !sensorsStopped && (uint32_t)(nowUs - lastHbUs_) >= 200000u) {
    const size_t n = emit(buf, cap, hil::HEARTBEAT, 0, 0, nowUs, seq_);
    if (n) { lastHbUs_ = nowUs; txHeartbeats++; }
    return n;
  }
  // Ask the simulator for a (new) session when we have none, or when the sensors stopped
  // (bridge failsafe, or this board was reset in the middle of a session).  Only while the bridge has been heard in the
  // last 3 s (it answers each HELLO): after a long wait a pile of old HELLOs could use up its 3 s wait for HELLO_ACK.
  // After being quiet, one HELLO goes out as soon as it is heard again, because a bridge that timed out ignores the
  // motor commands until it gets one.
  const bool heard = lastRxUs && (uint32_t)(nowUs - lastRxUs) < 3000000u;
  const bool wantHello = (session != SES_RUNNING || sensorsStopped) && session != SES_STOPPED;
  if (wantHello && !heard) wasQuiet_ = true;
  if ((wantHello || wasQuiet_) && heard && (uint32_t)(nowUs - lastHelloUs_) >= 1000000u) {
    const size_t n = emit(buf, cap, hil::HELLO, (const uint8_t *)hil::HELLO_TEXT, 10, nowUs, seq_);
    if (n) { lastHelloUs_ = nowUs; wasQuiet_ = false; }
    return n;
  }
  if (session == SES_RUNNING && !sensorsStopped && (uint32_t)(nowUs - lastStatusUs_) >= 50000u) {
    uint8_t pl[TELEMETRY_SIZE];
    telemetryPack(core, parser.crcErrors, pl, resetCode);
    const size_t n = emit(buf, cap, hil::STATUS, pl, (uint16_t)TELEMETRY_SIZE, nowUs, seq_);
    if (n) { lastStatusUs_ = nowUs; txStatus++; }
    return n;
  }
  return 0;
}

size_t FcApp::nextTelemetryPacket(uint8_t *buf, size_t cap, uint32_t nowUs) {
  if (params.telemetryPortOn < 0.5f || (uint32_t)(nowUs - lastTelemUs_) < 50000u) return 0;
  uint8_t pl[TELEMETRY_SIZE];
  telemetryPack(core, parser.crcErrors, pl, resetCode);
  const size_t n = emit(buf, cap, hil::STATUS, pl, (uint16_t)TELEMETRY_SIZE, nowUs, telemSeq_);
  if (n) lastTelemUs_ = nowUs;
  return n;
}

// ------------------------------------------------------------------ settings blob
//  u32 magic | u16 version | u16 size | Params | CalData | u32 crc32
size_t FcApp::settingsSize() const { return 8 + sizeof(Params) + sizeof(CalData) + 4; }

size_t FcApp::packSettings(uint8_t *out, size_t cap) const {
  const size_t total = settingsSize();
  if (cap < total) return 0;
  const uint32_t magic = SETTINGS_MAGIC;
  const uint16_t ver = PARAMS_VERSION, size = (uint16_t)total;
  memcpy(out, &magic, 4);
  memcpy(out + 4, &ver, 2);
  memcpy(out + 6, &size, 2);
  memcpy(out + 8, &params, sizeof(Params));
  memcpy(out + 8 + sizeof(Params), &cal, sizeof(CalData));
  const uint32_t crc = hil::crc32(out, total - 4);
  memcpy(out + total - 4, &crc, 4);
  return total;
}

bool FcApp::unpackSettings(const uint8_t *in, size_t n) {
  const size_t total = settingsSize();
  if (!in || n < total) return false;
  uint32_t magic, crc;
  uint16_t ver, size;
  memcpy(&magic, in, 4);
  memcpy(&ver, in + 4, 2);
  memcpy(&size, in + 6, 2);
  memcpy(&crc, in + total - 4, 4);
  if (magic != SETTINGS_MAGIC || ver != PARAMS_VERSION || size != total) return false;
  if (hil::crc32(in, total - 4) != crc) return false;
  memcpy(&params, in + 8, sizeof(Params));
  memcpy(&cal, in + 8 + sizeof(Params), sizeof(CalData));
  paramsSanitize(params);
  core.resetEstimator();
  return true;
}
