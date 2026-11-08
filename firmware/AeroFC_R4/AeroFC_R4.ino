// =====================================================================================================
//  AeroFC-R4  -  flight controller for the Arduino UNO R4 Minima + UAV HIL simulator
// =====================================================================================================
//  Board:  Tools > Board > Arduino UNO R4 Boards > Arduino UNO R4 Minima.   No extra libraries needed.
//
//  What happens here (the flight logic itself lives in src/core and is the same code the PC simulator runs):
//    1. bytes from USB  -> FcApp (HIL protocol -> sensors -> EKF -> controllers -> motor commands)
//    2. packets to USB  -> only whole packets, only when the USB buffer has room (never blocks)
//    3. telemetry       -> Serial1 (pin D1) for an optional USB-TTL adapter, a few bytes per pass (never blocks long)
//    4. 4 buttons + OLED dashboard / menu with help texts, settings in flash, status LED
//    5. watchdog: started only when the drone is ARMED (see src/hal/reset_cause.h for why)
//  docs/16_other_boards.md says what to change for another board.
// =====================================================================================================
#include <Arduino.h>
#include <Wire.h>
#include "board_config.h"
#include "src/core/fc_app.h"
#include "src/core/ui_menu.h"
#include "src/hal/oled_ssd1306.h"
#include "src/hal/buttons.h"
#include "src/hal/storage_flash.h"
#include "src/hal/reset_cause.h"

static FcApp app;
static UiMenu ui;
static UiPlatform plat;
static Ssd1306 oled;
static Buttons buttons;
static uint8_t frame[GFX_BYTES];
static uint8_t settingsBuf[1024];
static uint8_t txBuf[160];
static uint8_t telBuf[160];
static size_t telLen = 0, telPos = 0;
static uint32_t lastFrameMs = 0;
static bool telemetryStarted = false;
static_assert(8 + sizeof(Params) + sizeof(CalData) + 4 <= sizeof(settingsBuf) - 16, "the settings no longer fit in a flash slot (src/hal/storage_flash.cpp)");

// On the Minima and Nano R4 the USB port is on the chip itself. The UNO R4 WiFi's goes through its ESP32 chip, so its
// Serial has no DTR line to read, and data nobody reads can't get stuck in the way described in loop().
#if defined(ARDUINO_UNOR4_WIFI)
static bool pcSetDtr() { return true; }
static void usbWrite(const uint8_t *buf, size_t len) { Serial.write(buf, len); }
#else
#include <tusb.h>
static bool pcSetDtr() { return Serial.dtr(); }
// Serial.write() waits for ever if the PC stops reading while it sends: when a bridge stopped in flight, the board hung
// there until the watchdog restarted it, and without the watchdog (never armed) it stayed hung with no USB at all.
// The USB stack itself never waits. loop() checks there is room for the whole packet first.
static void usbWrite(const uint8_t *buf, size_t len) {
  tud_cdc_write(buf, (uint32_t)len);
  tud_cdc_write_flush();
}
#endif

static void loadSettings(bool announce, uint32_t nowUs) {
  size_t len = 0;
  const bool ok = storage::load(settingsBuf, sizeof(settingsBuf), len) && app.unpackSettings(settingsBuf, len);
  app.bootFromDefaults = !ok;
  app.lastLoadResult = ok ? 1 : -1;
  if (announce) ui.toast(ok ? "Settings loaded" : "No saved settings", nowUs);
}

static void serviceStorage(uint32_t nowUs) {
  if (app.saveRequested) {
    app.saveRequested = false;
    if (app.core.armed()) {
      app.lastSaveResult = -2;
      ui.toast("Save refused: armed", nowUs);
    } else {
      const size_t n = app.packSettings(settingsBuf, sizeof(settingsBuf));
      const bool ok = n && storage::save(settingsBuf, n);
      app.lastSaveResult = ok ? 1 : -1;
      if (ok) app.bootFromDefaults = false;
      ui.toast(ok ? "Saved to flash" : "SAVE FAILED", nowUs);
    }
  }
  if (app.loadRequested) {
    app.loadRequested = false;
    if (!app.core.armed()) loadSettings(true, nowUs);
  }
}

// Telemetry on D1: Serial1.write() waits until every byte has left the pin, so send at most 16 bytes per loop pass.
static void serviceTelemetry(uint32_t nowUs) {
  if (app.params.telemetryPortOn < 0.5f) return;
  if (!telemetryStarted) { Serial1.begin(TELEMETRY_BAUD); telemetryStarted = true; }
  if (telPos >= telLen) {
    telLen = app.nextTelemetryPacket(telBuf, sizeof(telBuf), nowUs);
    telPos = 0;
  }
  if (telPos < telLen) {
    const size_t n = min((size_t)16, telLen - telPos);
    Serial1.write(telBuf + telPos, n);
    telPos += n;
  }
}

// LED: slow blink = waiting for the PC, double blink = simulator connected, solid = armed, fast = failsafe/crash
static void updateLed(uint32_t ms) {
  bool on;
  const FlightCore &c = app.core;
  if (c.crashed || c.state == ST_FAILSAFE) on = (ms / 100) & 1;
  else if (c.armed()) on = true;
  else if (app.linkActive(micros())) { const uint32_t ph = ms % 1000; on = ph < 80 || (ph > 200 && ph < 280); }
  else on = (ms / 500) & 1;
  digitalWrite(PIN_STATUS_LED, on ? HIGH : LOW);
}

void setup() {
  const ResetCause cause = resetinfo::readAndClear();   // first: why did we start?
  plat.resetCause = resetinfo::name(cause);
  app.resetCode = (uint8_t)cause;
  pinMode(PIN_STATUS_LED, OUTPUT);

  Serial.begin(115200);       // native USB: the number does not matter
  (void)pcSetDtr();           // keep working even if the PC program does not set DTR

  app.init();
  ui.init(&app, &plat);
  const uint8_t pins[4] = { PIN_BTN_UP, PIN_BTN_DOWN, PIN_BTN_SELECT, PIN_BTN_BACK };
  buttons.begin(pins);

  storage::begin();
  delay(30);                                        // let the button pull-ups settle
  const uint32_t now = micros();
  if (digitalRead(PIN_BTN_BACK) == LOW) {           // hold BACK while powering up = factory defaults
    app.bootFromDefaults = true;
    ui.bootMessage("FACTORY DEFAULTS", now);
  } else {
    loadSettings(false, now);
  }

  oled.begin(Wire, OLED_I2C_CLOCK);
  if (cause == RC_HANG_WDT) ui.bootMessage("WATCHDOG: hang!", now);
  else if (!app.bootFromDefaults) ui.bootMessage("Settings loaded", now);
  // NO watchdog here: it starts on the first arm (see src/hal/reset_cause.h)
}

void loop() {
  resetinfo::kick();
  const uint32_t now = micros();

  // 1) everything that arrived on USB
  if (Serial.available() > 0) {
    uint8_t rx[128];
    int n = 0;
    while (n < (int)sizeof(rx) && Serial.available() > 0) {
      const int b = Serial.read();
      if (b < 0) break;
      rx[n++] = (uint8_t)b;
    }
    const uint32_t t0 = micros();
    app.feed(rx, (size_t)n, now);
    if (app.core.motorsReady) app.core.loopUs = micros() - t0;   // time of the control step(s)
  }

  // 2) timers (fixed-rate loop, failsafes, rates)
  app.tick(now);
  if (app.core.armed()) resetinfo::armWatchdog(WATCHDOG_MS);   // first arm only; stays on afterwards

  // 3) send: one whole packet at a time, only if the USB buffer has room for all of it, and only while a program on the
  // PC has the port open (DTR set). Every serial program sets DTR on opening the port, and drops it on closing it, before
  // it stops reading. Sending to a PC that isn't reading is never safe on the R4: the data stays stuck and blocks the
  // Arduino IDE's upload ("get_status: LIBUSB_ERROR_TIMEOUT"), and the core's USB driver waits for ever once its buffers
  // are full. Up to v3 the board kept sending for 2 s after the last data came in: a bridge stopped in flight then
  // hung it, measured with a step marker and the watchdog's own reset report.
  const bool pcListening = pcSetDtr();
  for (int guard = 0; pcListening && guard < 6; guard++) {
    const int space = Serial.availableForWrite();
    if (space <= 0) break;
    const size_t len = app.nextUsbPacket(txBuf, (size_t)min(space, (int)sizeof(txBuf)), now);
    if (!len) break;
    usbWrite(txBuf, len);
  }

  // 4) optional telemetry on D1
  serviceTelemetry(now);

  // 5) buttons -> menu
  uint8_t btn, evt;
  const uint32_t ms = millis();
  while (buttons.poll(ms, btn, evt)) ui.onButton(btn, evt, now);

  // 6) settings save / load requested from the menu (blocks a few ms, only while disarmed)
  serviceStorage(now);

  // 7) OLED: draw when needed, send one small chunk per pass and only when no USB data is waiting
  plat.i2cErrors = oled.errors();
  plat.oledReinits = oled.reinits();
  plat.watchdogOn = resetinfo::watchdogRunning();
  if ((uint32_t)(ms - lastFrameMs) >= 40 && !oled.busy()) {
    lastFrameMs = ms;
    if (ui.render(frame, now)) oled.setFrame(frame);
  }
  oled.setPower(ui.displayOn());
  if (Serial.available() == 0) oled.service(ms);

  updateLed(ms);
}
