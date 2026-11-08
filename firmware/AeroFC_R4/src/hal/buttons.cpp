#include "buttons.h"

static const uint32_t DEBOUNCE_MS = 25;
static const uint32_t LONG_MS = 800;
static const uint32_t REPEAT_DELAY_MS = 450;
static const uint32_t REPEAT_MS = 110;
enum { EV_SHORT = 1, EV_LONG = 2, EV_REPEAT = 3 };   // same values as UiEvent

void Buttons::begin(const uint8_t pins[4]) {
  for (uint8_t i = 0; i < 4; i++) {
    pins_[i] = pins[i];
    pinMode(pins_[i], INPUT_PULLUP);
    raw_[i] = stable_[i] = longSent_[i] = false;
    changeMs_[i] = downMs_[i] = repeatMs_[i] = 0;
  }
}

bool Buttons::poll(uint32_t now, uint8_t &btn, uint8_t &evt) {
  for (uint8_t i = 0; i < 4; i++) {
    const bool r = digitalRead(pins_[i]) == LOW;   // pressed = connected to GND
    if (r != raw_[i]) { raw_[i] = r; changeMs_[i] = now; }
    const bool isUpDown = (i <= 1);
    if (r != stable_[i] && (uint32_t)(now - changeMs_[i]) >= DEBOUNCE_MS) {
      stable_[i] = r;
      if (r) {                                       // pressed
        downMs_[i] = now;
        repeatMs_[i] = now + REPEAT_DELAY_MS;
        longSent_[i] = false;
        if (isUpDown) { btn = i; evt = EV_SHORT; return true; }
      } else if (!isUpDown && !longSent_[i]) {       // released before the long-press time
        btn = i; evt = EV_SHORT; return true;
      }
    }
    if (stable_[i]) {
      if (isUpDown && (int32_t)(now - repeatMs_[i]) >= 0) {
        repeatMs_[i] = now + REPEAT_MS;
        btn = i; evt = EV_REPEAT; return true;
      }
      if (!isUpDown && !longSent_[i] && (uint32_t)(now - downMs_[i]) >= LONG_MS) {
        longSent_[i] = true;
        btn = i; evt = EV_LONG; return true;
      }
    }
  }
  return false;
}
