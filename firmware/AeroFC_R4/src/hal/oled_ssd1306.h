// oled_ssd1306.h - small SSD1306 128x64 I2C driver that never blocks the flight loop for long.
// A full screen is 1 KB = ~25 ms of I2C at 400 kHz, far too long to send at once (a sensor packet arrives every 20 ms).
// So the driver only sends pages that changed, and only one small chunk (<= 1.5 ms) per call to pushChunk().
#pragma once
#include <Arduino.h>
#include <Wire.h>

class Ssd1306 {
 public:
  bool begin(TwoWire &wire);          // finds the display at 0x3C or 0x3D; false = no display (that is fine)
  bool present() const { return ok_; }
  uint8_t address() const { return addr_; }
  void setFrame(const uint8_t *fb);   // compares with what is on screen and marks the changed pages
  bool pushChunk();                   // sends one chunk; returns true while more is pending
  bool busy() const { return dirty_ != 0 || inPage_; }

 private:
  bool cmd(const uint8_t *c, uint8_t n);
  TwoWire *w_ = nullptr;
  bool ok_ = false;
  uint8_t addr_ = 0x3C;
  uint8_t shadow_[1024];              // what the display shows
  uint8_t next_[1024];                // what it should show
  uint8_t dirty_ = 0;                 // one bit per page
  bool inPage_ = false;
  uint8_t page_ = 0, col_ = 0;
  uint8_t errors_ = 0;
};
