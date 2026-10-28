// telemetry.h - STATUS (0x25) payload, version 1, 48 bytes, little-endian.
// The official bridge ignores STATUS packets; our simulator / ground station decodes them for live plots.
//
//  off type  field                      off type  field
//   0  u8   version (=1)                 24  u8x4 motor 1..4 (x250)
//   1  u8   state (FlightState)          28  i16  roll setpoint (centideg)
//   2  u8   mode  (FlightMode)           30  i16  pitch setpoint (centideg)
//   3  u8   flags (see TF_*)             32  u16  heading target (centideg 0..35999)
//   4  i16  roll  (centideg)             34  u16  control step time (us)
//   6  i16  pitch (centideg)             36  u8   IMU packet rate (Hz)
//   8  u16  yaw   (centideg 0..35999)    37  u8   reason (last disarm / failsafe)
//  10  i16  altitude (cm)                38  u16  CRC errors
//  12  i16  altitude target (cm)         40  u16  learned hover command (x10000)
//  14  i16  climb rate (cm/s)            42  i16  barometer altitude (cm)
//  16  i16  rc roll  (x1000)             44  u8   pilot packet rate (Hz)
//  18  i16  rc pitch (x1000)             45  u8   calibration: kind<<4 | progress 0..15
//  20  i16  rc yaw   (x1000)             46  i16  collective thrust (x10000)
//  22  i16  rc throttle field (x1000)
#pragma once
#include <stdint.h>
#include <stddef.h>

static const uint8_t TELEMETRY_VERSION = 1;
static const size_t TELEMETRY_SIZE = 48;

enum TelemetryFlags : uint8_t {
  TF_ARMED = 1, TF_AIRBORNE = 2, TF_COMPASS = 4, TF_HEADING_LOCK = 8,
  TF_FAILSAFE = 16, TF_CRASHED = 32, TF_PILOT_LINK = 64, TF_SENSORS_OK = 128
};

class FlightCore;
void telemetryPack(const FlightCore &c, uint32_t crcErrors, uint8_t out[TELEMETRY_SIZE]);
