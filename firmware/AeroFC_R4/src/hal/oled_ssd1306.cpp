#include "oled_ssd1306.h"

static const uint8_t CHUNK = 32;            // data bytes per I2C transaction (~0.8 ms at 400 kHz)
static const uint32_t PAUSE_MS = 100;       // after an error: let the bus settle before trying again
static const uint32_t REINIT_MS = 5000;     // re-send the init sequence (recovers a display that reset itself)
static const uint32_t FULL_MS = 10000;      // redraw everything (clears corrupted pixels)
static const uint32_t PROBE_MS = 1000;      // look for a missing display

static const uint8_t kInit[] = {
  0xAE,              // display off
  0xD5, 0x80,        // clock divide
  0xA8, 0x3F,        // multiplex 64
  0xD3, 0x00,        // display offset 0
  0x40,              // start line 0
  0x8D, 0x14,        // charge pump on (needed for the usual 3.3/5 V modules)
  0x20, 0x02,        // page addressing mode
  0xA1,              // segment remap (column 127 = SEG0)
  0xC8,              // COM scan direction: remapped
  0xDA, 0x12,        // COM pins for 128x64
  0x81, 0xCF,        // contrast
  0xD9, 0xF1,        // pre-charge
  0xDB, 0x40,        // VCOM detect
  0xA4,              // display follows RAM
  0xA6,              // normal (not inverted)
  0x2E,              // no scrolling
  0xAF               // display on
};

static const uint8_t kOff[] = { 0xAE, 0x8D, 0x10 };   // display off, charge pump off

bool Ssd1306::cmd(const uint8_t *c, uint8_t n) {
  w_->beginTransmission(addr_);
  w_->write((uint8_t)0x00);          // control byte: commands follow
  w_->write(c, n);
  return w_->endTransmission() == 0;
}

// Frees a stuck bus: if a device holds SDA low (it was cut off in the middle of a byte), clock SCL until it lets go.
void Ssd1306::restartBus() {
  w_->end();
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);
  delayMicroseconds(5);
  for (int i = 0; i < 9 && digitalRead(SDA) == LOW; i++) {
    pinMode(SCL, OUTPUT);
    digitalWrite(SCL, LOW);
    delayMicroseconds(5);
    pinMode(SCL, INPUT_PULLUP);
    delayMicroseconds(5);
  }
  // STOP condition: SDA low -> high while SCL is high
  pinMode(SDA, OUTPUT);
  digitalWrite(SDA, LOW);
  delayMicroseconds(5);
  pinMode(SDA, INPUT_PULLUP);
  delayMicroseconds(5);
  w_->begin();                       // note: begin() resets the clock to 100 kHz
  w_->setClock(clock_);
  w_->setWireTimeout(10000, false);  // 10 ms: a 32-byte chunk needs ~0.8 ms
}

bool Ssd1306::probe() {
  const uint8_t candidates[2] = { 0x3C, 0x3D };
  for (uint8_t a : candidates) {
    w_->beginTransmission(a);
    if (w_->endTransmission() == 0) { addr_ = a; return true; }
  }
  return false;
}

bool Ssd1306::sendInit() {
  if (!cmd(kInit, sizeof(kInit))) return false;
  isOn_ = true;
  return true;
}

void Ssd1306::begin(TwoWire &wire, uint32_t clockHz) {
  w_ = &wire;
  clock_ = clockHz;
  restartBus();
  ok_ = probe() && sendInit();
  memset(shadow_, 0xFF, sizeof(shadow_));   // force a full first frame
  memset(next_, 0, sizeof(next_));
  dirty_ = 0xFF;
  inPage_ = false;
  lastInit_ = lastFull_ = lastProbe_ = millis();
}

void Ssd1306::fail(uint32_t nowMs) {
  errors_++;
  // abort the transfer that may still be running in the background, then wait before touching the bus again
  w_->end();
  inPage_ = false;
  pendingInit_ = true;
  pauseUntil_ = nowMs + PAUSE_MS;
}

void Ssd1306::setFrame(const uint8_t *fb) {
  memcpy(next_, fb, sizeof(next_));
  for (uint8_t p = 0; p < 8; p++)
    if (memcmp(next_ + p * 128, shadow_ + p * 128, 128) != 0) dirty_ |= (uint8_t)(1u << p);
}

void Ssd1306::service(uint32_t nowMs) {
  if (!w_) return;
  if ((int32_t)(nowMs - pauseUntil_) < 0) return;
  if (!ok_) {                                        // not found: look again once per second
    if ((uint32_t)(nowMs - lastProbe_) < PROBE_MS) return;
    lastProbe_ = nowMs;
    restartBus();
    if (probe() && sendInit()) {
      ok_ = true;
      reinits_++;
      memset(shadow_, 0xFF, sizeof(shadow_));
      dirty_ = 0xFF;
      lastInit_ = lastFull_ = nowMs;
    }
    return;
  }
  if (pendingInit_) {                                // recover after an error
    pendingInit_ = false;
    restartBus();
    if (!probe() || !sendInit()) { ok_ = false; lastProbe_ = nowMs; return; }
    reinits_++;
    memset(shadow_, 0xFF, sizeof(shadow_));
    dirty_ = 0xFF;
    lastInit_ = lastFull_ = nowMs;
    return;
  }
  if (!inPage_) {
    if (wantOn_ != isOn_) {                                  // switch the panel on or off between pages
      if (wantOn_) {
        if (!sendInit()) { fail(nowMs); return; }
        memset(shadow_, 0xFF, sizeof(shadow_));
        dirty_ = 0xFF;
        lastInit_ = lastFull_ = nowMs;
      } else {
        if (!cmd(kOff, sizeof(kOff))) { fail(nowMs); return; }
        isOn_ = false;
      }
      return;
    }
    if (!isOn_) return;
    if ((uint32_t)(nowMs - lastInit_) >= REINIT_MS) {        // keep-alive: display on, addressing, charge pump
      lastInit_ = nowMs;
      if (!sendInit()) fail(nowMs);
      return;
    }
    if ((uint32_t)(nowMs - lastFull_) >= FULL_MS) {          // periodic full redraw
      lastFull_ = nowMs;
      memset(shadow_, 0xFF, sizeof(shadow_));
      dirty_ = 0xFF;
    }
    if (!dirty_) return;
    page_ = 0;
    while (!(dirty_ & (1u << page_))) page_++;
    dirty_ &= (uint8_t)~(1u << page_);
    const uint8_t setPage[3] = { (uint8_t)(0xB0 | page_), 0x00, 0x10 };   // page, column 0
    if (!cmd(setPage, 3)) { dirty_ |= (uint8_t)(1u << page_); fail(nowMs); return; }
    col_ = 0;
    inPage_ = true;
    return;
  }
  const uint8_t *src = next_ + page_ * 128 + col_;
  w_->beginTransmission(addr_);
  w_->write((uint8_t)0x40);          // control byte: data follows
  w_->write(src, CHUNK);
  if (w_->endTransmission() != 0) {
    dirty_ |= (uint8_t)(1u << page_);
    fail(nowMs);
    return;
  }
  memcpy(shadow_ + page_ * 128 + col_, src, CHUNK);
  col_ += CHUNK;
  if (col_ >= 128) inPage_ = false;
}
