// fc_app.h - connects the HIL serial byte stream to the flight core.  Platform independent:
// the Arduino sketch, the browser (WebAssembly) and the tests all drive the very same object.
//
//   feed(bytes)            everything received on USB
//   tick(now)              call as often as possible (timers)
//   nextUsbPacket(buf)     returns ONE complete packet to send if it fits in `cap` bytes (else 0, try later)
//   nextTelemetryPacket()  STATUS packets for the optional second port (Serial1, pins D0/D1)
#pragma once
#include "hil_protocol.h"
#include "flight_core.h"
#include "telemetry.h"

static const char FW_VERSION[] = "AeroFC-R4 3.0";
static const uint32_t SETTINGS_MAGIC = 0x34434641u;   // "AFC4"

enum Session : uint8_t { SES_WAITING = 0, SES_HANDSHAKE, SES_RUNNING, SES_STOPPED };

class FcApp {
 public:
  void init();                                   // defaults (call unpackSettings() afterwards to load saved ones)
  void feed(const uint8_t *data, size_t n, uint32_t nowUs);
  void tick(uint32_t nowUs);
  size_t nextUsbPacket(uint8_t *buf, size_t cap, uint32_t nowUs);
  size_t nextTelemetryPacket(uint8_t *buf, size_t cap, uint32_t nowUs);

  // ---- settings storage (the platform decides where the bytes go)
  size_t settingsSize() const;
  size_t packSettings(uint8_t *out, size_t cap) const;
  bool unpackSettings(const uint8_t *in, size_t n);    // false = invalid / old version -> nothing changed
  void restoreDefaults();
  bool saveRequested = false;                    // set by the menu, cleared by the platform after saving
  bool loadRequested = false;                    // set by the menu, cleared by the platform after loading
  int8_t lastSaveResult = 0;                     // 1 ok, -1 failed, -2 refused (armed)
  int8_t lastLoadResult = 0;                     // 1 ok, -1 nothing valid stored (defaults kept)
  bool bootFromDefaults = true;                  // true when no valid settings were found at power-up

  // ---- status
  uint8_t session = SES_WAITING;
  uint32_t lastRxUs = 0, lastSensorUs = 0;
  bool linkActive(uint32_t nowUs) const { return lastSensorUs && (uint32_t)(nowUs - lastSensorUs) < 300000u; }
  uint32_t rxCount[8] = { 0 };                   // IMU, BARO, MAG, GPS, CONTROL, HELLO, START, STOP
  uint32_t txActuators = 0, txHeartbeats = 0, txStatus = 0;

  Params params;
  CalData cal;
  FlightCore core;
  hil::Parser parser;

 private:
  void handle(const hil::Packet &pk, uint32_t nowUs);
  size_t emit(uint8_t *buf, size_t cap, uint8_t type, const uint8_t *payload, uint16_t len, uint32_t nowUs,
              uint32_t &seq);
  bool needHelloAck_ = false;
  uint32_t lastHbUs_ = 0, lastHelloUs_ = 0, lastStatusUs_ = 0, lastTelemUs_ = 0;
  uint32_t seq_ = 1, telemSeq_ = 1;
};
