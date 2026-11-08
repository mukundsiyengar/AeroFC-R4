// oled_ssd1306.h - small SSD1306 128x64 I2C driver that never blocks the flight loop for long and repairs itself.
//
// A full screen is 1 KB = ~25 ms of I2C, far too long to send at once (a sensor packet arrives every 20 ms).
// So the driver only sends pages that changed, one 32-byte chunk (~0.8 ms) per call to pushChunk().
//
// Self-repair (v2):
//  * any I2C error -> Wire.end() at once (the R4's Wire does NOT abort a timed-out transfer by itself; letting the
//    next chunk overwrite its buffer is what garbled / froze the display in v1), pause, bus clear, re-init, full redraw
//  * every 5 s the init sequence is sent again (a display that browned out comes back by itself)
//  * every 10 s the whole screen is redrawn (clears any corrupted pixels)
//  * display missing -> probed once per second (plug it in while running)
#pragma once
#include <Arduino.h>
#include <Wire.h>

class Ssd1306 {
 public:
  void begin(TwoWire &wire, uint32_t clockHz);
  bool present() const { return ok_; }
  uint8_t address() const { return addr_; }
  void setFrame(const uint8_t *fb);   // compares with what is on screen and marks the changed pages
  void setPower(bool on) { wantOn_ = on; }   // false really switches the panel and its charge pump off (under 1 mA instead of about 20 mA)
  bool panelOn() const { return isOn_; }
  void service(uint32_t nowMs);       // call every loop: sends one chunk or does the housekeeping
  bool busy() const { return dirty_ != 0 || inPage_; }
  uint16_t errors() const { return errors_; }
  uint16_t reinits() const { return reinits_; }

 private:
  bool probe();
  bool sendInit();
  bool cmd(const uint8_t *c, uint8_t n);
  void fail(uint32_t nowMs);
  void restartBus();
  TwoWire *w_ = nullptr;
  uint32_t clock_ = 400000;
  bool ok_ = false;
  bool wantOn_ = true, isOn_ = true;
  uint8_t addr_ = 0x3C;
  uint8_t shadow_[1024];              // what the display shows
  uint8_t next_[1024];                // what it should show
  uint8_t dirty_ = 0;                 // one bit per page
  bool inPage_ = false;
  uint8_t page_ = 0, col_ = 0;
  uint16_t errors_ = 0, reinits_ = 0;
  uint32_t pauseUntil_ = 0, lastProbe_ = 0, lastInit_ = 0, lastFull_ = 0;
  bool pendingInit_ = false;
};
