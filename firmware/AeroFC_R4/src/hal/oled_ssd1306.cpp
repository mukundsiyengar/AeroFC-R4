#include "oled_ssd1306.h"

static const uint8_t CHUNK = 64;   // data bytes per I2C transaction (~1.5 ms at 400 kHz)

bool Ssd1306::cmd(const uint8_t *c, uint8_t n) {
  w_->beginTransmission(addr_);
  w_->write((uint8_t)0x00);          // control byte: commands follow
  w_->write(c, n);
  return w_->endTransmission() == 0;
}

bool Ssd1306::begin(TwoWire &wire) {
  w_ = &wire;
  ok_ = false;
  const uint8_t candidates[2] = { 0x3C, 0x3D };
  for (uint8_t a : candidates) {
    w_->beginTransmission(a);
    if (w_->endTransmission() == 0) { addr_ = a; ok_ = true; break; }
  }
  if (!ok_) return false;
  static const uint8_t init[] = {
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
  ok_ = cmd(init, sizeof(init));
  memset(shadow_, 0xFF, sizeof(shadow_));   // force a full first frame
  memset(next_, 0, sizeof(next_));
  dirty_ = 0xFF;
  return ok_;
}

void Ssd1306::setFrame(const uint8_t *fb) {
  if (!ok_) return;
  memcpy(next_, fb, sizeof(next_));
  for (uint8_t p = 0; p < 8; p++)
    if (memcmp(next_ + p * 128, shadow_ + p * 128, 128) != 0) dirty_ |= (uint8_t)(1u << p);
}

bool Ssd1306::pushChunk() {
  if (!ok_) return false;
  if (!inPage_) {
    if (!dirty_) return false;
    page_ = 0;
    while (!(dirty_ & (1u << page_))) page_++;
    dirty_ &= (uint8_t)~(1u << page_);
    const uint8_t setPage[3] = { (uint8_t)(0xB0 | page_), 0x00, 0x10 };   // page, column 0
    if (!cmd(setPage, 3)) { if (++errors_ > 20) ok_ = false; return false; }
    col_ = 0;
    inPage_ = true;
    return true;
  }
  const uint8_t *src = next_ + page_ * 128 + col_;
  w_->beginTransmission(addr_);
  w_->write((uint8_t)0x40);          // control byte: data follows
  w_->write(src, CHUNK);
  if (w_->endTransmission() != 0) {
    if (++errors_ > 20) ok_ = false; // display unplugged: stop trying, the flight controller keeps running
    inPage_ = false;
    dirty_ |= (uint8_t)(1u << page_);
    return false;
  }
  memcpy(shadow_ + page_ * 128 + col_, src, CHUNK);
  col_ += CHUNK;
  if (col_ >= 128) inPage_ = false;
  return busy();
}
