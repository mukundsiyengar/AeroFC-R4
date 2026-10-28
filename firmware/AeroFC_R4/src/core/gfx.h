// gfx.h - tiny monochrome graphics for a 128 x 64 framebuffer in SSD1306 layout
// (8 pages of 128 bytes; bit 0 of a byte is the top pixel of that 8-pixel tall page).
#pragma once
#include <stdint.h>
#include <stddef.h>

static const int GFX_W = 128;
static const int GFX_H = 64;
static const size_t GFX_BYTES = GFX_W * GFX_H / 8;

class Gfx {
 public:
  explicit Gfx(uint8_t *buffer) : fb(buffer) {}
  void clear();
  void pixel(int x, int y, bool on = true);
  void hline(int x, int y, int w, bool on = true);
  void vline(int x, int y, int h, bool on = true);
  void rect(int x, int y, int w, int h);
  void fillRect(int x, int y, int w, int h, bool on = true);
  void invertRect(int x, int y, int w, int h);
  void line(int x0, int y0, int x1, int y1);
  int text(int x, int y, const char *s, bool on = true);   // 6 px per character, returns the end x
  void bar(int x, int y, int w, int h, float frac);        // progress bar 0..1
  uint8_t *fb;
};

// number -> text without printf (the R4 printf has no float support)
char *fmtFloat(char *out, float v, int decimals);   // returns out
char *fmtInt(char *out, long v);
