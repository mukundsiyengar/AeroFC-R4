// buttons.h - 4 push buttons with debounce, short press, long press and auto-repeat.
//   UP / DOWN : SHORT as soon as pressed, then REPEAT while held (fast value changes)
//   SELECT / BACK : SHORT when released quickly, LONG when held 0.8 s
#pragma once
#include <Arduino.h>

class Buttons {
 public:
  void begin(const uint8_t pins[4]);
  // returns true and fills btn / evt (0..3 / UiEvent) when something happened
  bool poll(uint32_t nowMs, uint8_t &btn, uint8_t &evt);
  bool isDown(uint8_t b) const { return stable_[b]; }

 private:
  uint8_t pins_[4];
  bool raw_[4], stable_[4], longSent_[4];
  uint32_t changeMs_[4], downMs_[4], repeatMs_[4];
};
