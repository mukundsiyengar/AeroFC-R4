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

void telemetryPack(const FlightCore &c, uint32_t crcErrors, uint8_t o[TELEMETRY_SIZE], uint8_t resetCode) {
  for (size_t i = 0; i < TELEMETRY_SIZE; i++) o[i] = 0;
  const Params &p = *c.prm;
  o[0] = TELEMETRY_VERSION;
  o[1] = c.state;
  o[2] = c.mode;
  uint8_t f = 0;
  if (c.armed()) f |= TF_ARMED;
  if (c.airborne) f |= TF_AIRBORNE;
  if (c.ekf.magHealthy && p.compassOn > 0.5f) f |= TF_COMPASS;
  if (c.att.headingLocked) f |= TF_HEADING_LOCK;
  if (c.state == ST_FAILSAFE) f |= TF_FAILSAFE;
  if (c.crashed) f |= TF_CRASHED;
  if (c.pilot.everReceived && !c.pilot.linkLost) f |= TF_PILOT_LINK;
  if (c.sensorsOk) f |= TF_SENSORS_OK;
  o[3] = f;
  hil::putI16(o + 4, s16(c.ekf.roll() * FC_RAD2DEG * 100.0f));
  hil::putI16(o + 6, s16(c.ekf.pitch() * FC_RAD2DEG * 100.0f));
  hil::putU16(o + 8, heading100(c.ekf.yaw()));
  hil::putF32(o + 10, c.altf.alt);
  hil::putF32(o + 14, c.altTarget);
  hil::putI16(o + 18, s16(c.altf.vz * 100.0f));
  hil::putI16(o + 20, s16(c.pilot.rawRoll * 1000.0f));
  hil::putI16(o + 22, s16(c.pilot.rawPitch * 1000.0f));
  hil::putI16(o + 24, s16(c.pilot.rawYaw * 1000.0f));
  hil::putI16(o + 26, s16(c.pilot.rawThr * 1000.0f));
  for (int i = 0; i < 4; i++) o[28 + i] = (uint8_t)u16(fcClamp(c.motor[i], 0.0f, 1.0f) * 250.0f);
  hil::putI16(o + 32, s16(c.rollSp * FC_RAD2DEG * 100.0f));
  hil::putI16(o + 34, s16(c.pitchSp * FC_RAD2DEG * 100.0f));
  hil::putU16(o + 36, heading100(c.att.headingTarget));
  hil::putU16(o + 38, u16((float)c.loopUs));
  o[40] = (uint8_t)fcClamp(c.imuHz, 0.0f, 255.0f);
  o[41] = c.reason;
  hil::putU16(o + 42, u16((float)crcErrors));
  hil::putU16(o + 44, u16(thrustToCmd(p, c.altc.integ) * 10000.0f));
  hil::putI16(o + 46, s16(c.collective * 10000.0f));
  o[48] = c.pilot.rateHz;
  const uint8_t ck = (uint8_t)c.sens.calibrating();
  o[49] = (uint8_t)((ck << 4) | ((uint8_t)(fcClamp(c.sens.calProgress(), 0.0f, 1.0f) * 15.0f) & 0x0F));
  uint8_t f2 = 0;
  if (c.posActive) f2 |= TF2_POS_ACTIVE;
  if (c.posReady) f2 |= TF2_POS_READY;
  if (p.posHoldOn > 0.5f) f2 |= TF2_POS_ON;
  if (p.altLimitOn > 0.5f) f2 |= TF2_ALT_LIMIT;
  if (c.atCeiling) f2 |= TF2_AT_CEILING;
  if (c.pilot.linkLost) f2 |= TF2_LINK_LOST;
  if (c.gpsAidOk) f2 |= TF2_GPS_AID;
  if (p.posStick > 0.5f) f2 |= TF2_STICK_ANGLE;
  o[50] = f2;
  o[51] = (uint8_t)((c.posState & 0x0F) | (((uint8_t)(p.speedMode + 0.5f) & 0x0F) << 4));
  hil::putI16(o + 52, s16(c.pos.kn.p * 10.0f));
  hil::putI16(o + 54, s16(c.pos.ke.p * 10.0f));
  hil::putI16(o + 56, s16(c.pos.kn.v * 100.0f));
  hil::putI16(o + 58, s16(c.pos.ke.v * 100.0f));
  hil::putU16(o + 60, u16(c.posActive ? c.posCtl.dist * 100.0f : 0.0f));
  hil::putU16(o + 62, u16(p.altLimitOn > 0.5f ? p.maxAlt : 0.0f));
  hil::putI16(o + 64, s16(c.aidYawOffset * FC_RAD2DEG * 100.0f));
  o[66] = (uint8_t)((c.nav.kind << 4) | (c.nav.stage & 0x0F));
  o[67] = c.nav.wpIndex;
  hil::putI16(o + 68, s16(c.nav.targetN * 10.0f));
  hil::putI16(o + 70, s16(c.nav.targetE * 10.0f));
  hil::putI16(o + 72, s16(c.rthHomeN * 10.0f));
  hil::putI16(o + 74, s16(c.rthHomeE * 10.0f));
  hil::putU16(o + 76, u16(c.nav.dist * 10.0f));
  hil::putU16(o + 78, c.imuStaleCount);
  hil::putU16(o + 80, c.imuTimeBad);
  hil::putU16(o + 82, c.pilot.badPackets);
  o[84] = c.nav.markerFresh() ? 1 : 0;
  o[85] = c.pos.jumps;
  o[86] = resetCode;
}
