#include "telemetry.h"
#include "flight_core.h"
#include "hil_protocol.h"

static int16_t s16(float v) {
  if (!fcFinite(v)) return 0;
  if (v > 32767.0f) return 32767;
  if (v < -32768.0f) return -32768;
  return (int16_t)(v >= 0.0f ? v + 0.5f : v - 0.5f);
}
static uint16_t u16(float v) {
  if (!fcFinite(v) || v < 0.0f) return 0;
  if (v > 65535.0f) return 65535;
  return (uint16_t)(v + 0.5f);
}
static uint16_t heading100(float rad) {
  float d = rad * FC_RAD2DEG;
  while (d < 0.0f) d += 360.0f;
  while (d >= 360.0f) d -= 360.0f;
  return (uint16_t)(d * 100.0f) % 36000u;
}

void telemetryPack(const FlightCore &c, uint32_t crcErrors, uint8_t o[TELEMETRY_SIZE]) {
  for (size_t i = 0; i < TELEMETRY_SIZE; i++) o[i] = 0;
  o[0] = TELEMETRY_VERSION;
  o[1] = c.state;
  o[2] = c.mode;
  uint8_t f = 0;
  if (c.armed()) f |= TF_ARMED;
  if (c.airborne) f |= TF_AIRBORNE;
  if (c.ekf.magHealthy && c.prm->compassOn > 0.5f) f |= TF_COMPASS;
  if (c.att.headingLocked) f |= TF_HEADING_LOCK;
  if (c.state == ST_FAILSAFE) f |= TF_FAILSAFE;
  if (c.crashed) f |= TF_CRASHED;
  if (c.pilot.everReceived && !c.pilot.linkLost) f |= TF_PILOT_LINK;
  if (c.sensorsOk) f |= TF_SENSORS_OK;
  o[3] = f;
  hil::putI16(o + 4, s16(c.ekf.roll() * FC_RAD2DEG * 100.0f));
  hil::putI16(o + 6, s16(c.ekf.pitch() * FC_RAD2DEG * 100.0f));
  hil::putU16(o + 8, heading100(c.ekf.yaw()));
  hil::putI16(o + 10, s16(c.altf.alt * 100.0f));
  hil::putI16(o + 12, s16(c.altTarget * 100.0f));
  hil::putI16(o + 14, s16(c.altf.vz * 100.0f));
  hil::putI16(o + 16, s16(c.pilot.rawRoll * 1000.0f));
  hil::putI16(o + 18, s16(c.pilot.rawPitch * 1000.0f));
  hil::putI16(o + 20, s16(c.pilot.rawYaw * 1000.0f));
  hil::putI16(o + 22, s16(c.pilot.rawThr * 1000.0f));
  for (int i = 0; i < 4; i++) o[24 + i] = (uint8_t)u16(fcClamp(c.motor[i], 0.0f, 1.0f) * 250.0f);
  hil::putI16(o + 28, s16(c.rollSp * FC_RAD2DEG * 100.0f));
  hil::putI16(o + 30, s16(c.pitchSp * FC_RAD2DEG * 100.0f));
  hil::putU16(o + 32, heading100(c.att.headingTarget));
  hil::putU16(o + 34, u16((float)c.loopUs));
  o[36] = (uint8_t)fcClamp(c.imuHz, 0.0f, 255.0f);
  o[37] = c.reason;
  hil::putU16(o + 38, u16((float)crcErrors));
  hil::putU16(o + 40, u16(thrustToCmd(*c.prm, c.altc.integ) * 10000.0f));
  hil::putI16(o + 42, s16(c.sf.baroAlt * 100.0f));
  o[44] = c.pilot.rateHz;
  const uint8_t ck = (uint8_t)c.sens.calibrating();
  o[45] = (uint8_t)((ck << 4) | ((uint8_t)(fcClamp(c.sens.calProgress(), 0.0f, 1.0f) * 15.0f) & 0x0F));
  hil::putI16(o + 46, s16(c.collective * 10000.0f));
}
