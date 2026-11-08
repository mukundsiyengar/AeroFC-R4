// telemetry.h - STATUS (0x25) payload, version 2, 66 bytes, little-endian.
// The official bridge ignores STATUS packets; our simulator, bridge_plus.py and the D1 port decode them for plots.
//
//  off type  field                         off type  field
//   0  u8   version (=2)                    32  i16  roll setpoint (centideg)
//   1  u8   state (FlightState)             34  i16  pitch setpoint (centideg)
//   2  u8   mode  (FlightMode)              36  u16  heading target (centideg 0..35999)
//   3  u8   flags (see TF_*)                38  u16  control step time (us)
//   4  i16  roll  (centideg)                40  u8   IMU packet rate (Hz)
//   6  i16  pitch (centideg)                41  u8   reason (last disarm / failsafe)
//   8  u16  yaw   (centideg 0..35999)       42  u16  CRC errors
//  10  f32  altitude (m)                    44  u16  learned hover command (x10000)
//  14  f32  altitude target (m)             46  i16  collective thrust (x10000)
//  18  i16  climb rate (cm/s)               48  u8   pilot packet rate (Hz)
//  20  i16  rc roll  (x1000)                49  u8   calibration: kind<<4 | progress 0..15
//  22  i16  rc pitch (x1000)                50  u8   flags2 (see TF2_*)
//  24  i16  rc yaw   (x1000)                51  u8   position-hold state (low 4 bits) | speed mode << 4
//  26  i16  rc throttle field (x1000)       52  i16  position north of home (dm)
//  28  u8x4 motor 1..4 commands (x250)      54  i16  position east of home (dm)
//                                           56  i16  speed north (cm/s)
//                                           58  i16  speed east (cm/s)
//                                           60  u16  distance to the hold point (cm)
//                                           62  u16  height ceiling (m above take-off), 0 = no limit
//                                           64  i16  learned compass/GPS heading offset (centideg)
// 66 u8 nav (kind << 4 | stage)   67 u8 mission point, 68 i16 target north x10, 70 i16 target east x10, 72 i16 home north x10
// 74 i16 home east x10, 76 u16 distance to target x10, 78 u16 stale IMU packets, 80 u16 bad IMU time stamps, 82 u16 bad pilot packets
// 84 u8 flags3: 1 = the camera sees the marker now    85 u8 spare
#pragma once
#include <stdint.h>
#include <stddef.h>

static const uint8_t TELEMETRY_VERSION = 2;
static const size_t TELEMETRY_SIZE = 87;

enum TelemetryFlags : uint8_t {
  TF_ARMED = 1, TF_AIRBORNE = 2, TF_COMPASS = 4, TF_HEADING_LOCK = 8,
  TF_FAILSAFE = 16, TF_CRASHED = 32, TF_PILOT_LINK = 64, TF_SENSORS_OK = 128
};
enum TelemetryFlags2 : uint8_t {
  TF2_POS_ACTIVE = 1, TF2_POS_READY = 2, TF2_POS_ON = 4, TF2_ALT_LIMIT = 8, TF2_AT_CEILING = 16,
  TF2_LINK_LOST = 32, TF2_GPS_AID = 64, TF2_STICK_ANGLE = 128
};

class FlightCore;
// resetCode: why the board last started, as the board's own code (255 = not known)
void telemetryPack(const FlightCore &c, uint32_t crcErrors, uint8_t out[TELEMETRY_SIZE], uint8_t resetCode = 255);
