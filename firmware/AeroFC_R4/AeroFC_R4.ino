// =====================================================================================================
//  AeroFC-R4  -  flight controller for the Arduino UNO R4 Minima + Aerovidya UAV HIL simulator
// =====================================================================================================
//  Board:  Tools > Board > Arduino UNO R4 Boards > Arduino UNO R4 Minima.   No extra libraries needed.
//
//  What happens here (the flight logic itself lives in src/core and is the same code the PC simulator runs):
//    1. bytes from USB  -> FcApp (HIL protocol -> sensors -> EKF -> controllers -> motor commands)
//    2. packets to USB  -> only whole packets, only when the USB buffer has room (never blocks)
//    3. telemetry       -> Serial1 (pins D0/D1) for an optional USB-TTL adapter
//    4. 4 buttons + OLED dashboard / menu, settings in flash, watchdog, status LED
// =====================================================================================================
#include <Arduino.h>
#include <Wire.h>
#include <WDT.h>
#include "board_config.h"
#include "src/core/fc_app.h"
#include "src/core/ui_menu.h"
#include "src/hal/oled_ssd1306.h"
#include "src/hal/buttons.h"
#include "src/hal/storage_flash.h"

static FcApp app;
static UiMenu ui;
static Ssd1306 oled;
static Buttons buttons;
static uint8_t frame[GFX_BYTES];
static uint8_t settingsBuf[1024];
static uint8_t txBuf[160];
static bool watchdogReset = false;
static uint32_t lastFrameMs = 0;

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
  pinMode(PIN_STATUS_LED, OUTPUT);
  watchdogReset = R_SYSTEM->RSTSR1_b.WDTRF;   // was the last reset caused by the watchdog?

  Serial.begin(115200);       // native USB: the number does not matter
  (void)Serial.dtr();         // keep working even if the PC program does not set DTR
  Serial1.begin(TELEMETRY_BAUD);

  app.init();
  ui.init(&app);
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

  Wire.begin();
  Wire.setClock(OLED_I2C_CLOCK);
  Wire.setWireTimeout(3000, true);                  // never wait more than 3 ms for the display
  oled.begin(Wire);
  if (watchdogReset) ui.bootMessage("WATCHDOG RESET!", now);
  else if (!app.bootFromDefaults) ui.bootMessage("Settings loaded", now);

  WDT.begin(WATCHDOG_MS);
}

void loop() {
  WDT.refresh();
  const uint32_t now = micros();

  // 1) everything that arrived on USB
  int avail = Serial.available();
  if (avail > 0) {
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

  // 3) send: one whole packet at a time, only if the USB buffer has room for all of it
  for (int guard = 0; guard < 6; guard++) {
    const int space = Serial.availableForWrite();
    if (space <= 0) break;
    const size_t len = app.nextUsbPacket(txBuf, (size_t)min(space, (int)sizeof(txBuf)), now);
    if (!len) break;
    Serial.write(txBuf, len);
  }

  // 4) optional telemetry on D1 (20 Hz)
  const size_t tl = app.nextTelemetryPacket(txBuf, sizeof(txBuf), now);
  if (tl) Serial1.write(txBuf, tl);

  // 5) buttons -> menu
  uint8_t btn, evt;
  const uint32_t ms = millis();
  while (buttons.poll(ms, btn, evt)) ui.onButton(btn, evt, now);

  // 6) settings save / load requested from the menu (blocks a few ms, only while disarmed)
  serviceStorage(now);

  // 7) OLED: draw 5 times a second, send small chunks only when no USB data is waiting
  if (oled.present()) {
    if ((uint32_t)(ms - lastFrameMs) >= UI_FRAME_MS && !oled.busy()) {
      lastFrameMs = ms;
      if (ui.render(frame, now)) oled.setFrame(frame);
    }
    if (Serial.available() == 0) oled.pushChunk();
  }

  updateLed(ms);
}
