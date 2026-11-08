#include "gfx.h"
#include "font5x7.h"
#include <string.h>

void Gfx::clear() { memset(fb, 0, GFX_BYTES); }

void Gfx::pixel(int x, int y, bool on) {
  if (x < 0 || y < 0 || x >= GFX_W || y >= GFX_H) return;
  uint8_t &b = fb[(y >> 3) * GFX_W + x];
  const uint8_t m = (uint8_t)(1u << (y & 7));
  if (on) b |= m; else b &= (uint8_t)~m;
}

void Gfx::hline(int x, int y, int w, bool on) { for (int i = 0; i < w; i++) pixel(x + i, y, on); }
void Gfx::vline(int x, int y, int h, bool on) { for (int i = 0; i < h; i++) pixel(x, y + i, on); }

void Gfx::rect(int x, int y, int w, int h) {
  hline(x, y, w);
  hline(x, y + h - 1, w);
  vline(x, y, h);
  vline(x + w - 1, y, h);
}

void Gfx::fillRect(int x, int y, int w, int h, bool on) {
  for (int j = 0; j < h; j++) hline(x, y + j, w, on);
}

void Gfx::invertRect(int x, int y, int w, int h) {
  for (int j = 0; j < h; j++)
    for (int i = 0; i < w; i++) {
      const int px = x + i, py = y + j;
      if (px < 0 || py < 0 || px >= GFX_W || py >= GFX_H) continue;
      fb[(py >> 3) * GFX_W + px] ^= (uint8_t)(1u << (py & 7));
    }
}

void Gfx::line(int x0, int y0, int x1, int y1) {
  int dx = x1 > x0 ? x1 - x0 : x0 - x1, sx = x0 < x1 ? 1 : -1;
  int dy = y1 > y0 ? y0 - y1 : y1 - y0, sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;
  for (int guard = 0; guard < 512; guard++) {
    pixel(x0, y0);
    if (x0 == x1 && y0 == y1) break;
    const int e2 = 2 * err;
    if (e2 >= dy) { err += dy; x0 += sx; }
    if (e2 <= dx) { err += dx; y0 += sy; }
  }
}

int Gfx::text(int x, int y, const char *s, bool on) {
  for (; s && *s; s++) {
    unsigned char c = (unsigned char)*s;
    if (c < 0x20 || c > 0x7F) c = '?';
    const uint8_t *g = kFont5x7[c - 0x20];
    for (int col = 0; col < 5; col++)
      for (int row = 0; row < 7; row++)
        if (g[col] & (1u << row)) pixel(x + col, y + row, on);
    x += 6;
  }
  return x;
}

void Gfx::bar(int x, int y, int w, int h, float frac) {
  if (frac < 0.0f) frac = 0.0f;
  if (frac > 1.0f) frac = 1.0f;
  rect(x, y, w, h);
  fillRect(x + 1, y + 1, (int)((float)(w - 2) * frac + 0.5f), h - 2);
}

char *fmtInt(char *out, long v) {
  char tmp[12];
  int n = 0;
  bool neg = v < 0;
  unsigned long u = neg ? (unsigned long)(-v) : (unsigned long)v;
  do { tmp[n++] = (char)('0' + (u % 10)); u /= 10; } while (u && n < 11);
  int k = 0;
  if (neg) out[k++] = '-';
  while (n) out[k++] = tmp[--n];
  out[k] = 0;
  return out;
}

char *fmtFloat(char *out, float v, int decimals) {
  if (!(v == v)) { strcpy(out, "nan"); return out; }
  if (decimals < 0) decimals = 0;
  if (decimals > 4) decimals = 4;
  long scale = 1;
  for (int i = 0; i < decimals; i++) scale *= 10;
  const bool neg = v < 0.0f;
  float a = neg ? -v : v;
  if (a > 2.0e9f / (float)scale) a = 2.0e9f / (float)scale;
  long whole = (long)(a * (float)scale + 0.5f);
  long ip = whole / scale, fp = whole % scale;
  int k = 0;
  if (neg && whole != 0) out[k++] = '-';
  fmtInt(out + k, ip);
  k = (int)strlen(out);
  if (decimals > 0) {
    out[k++] = '.';
    for (long d = scale / 10; d >= 1; d /= 10) { out[k++] = (char)('0' + (fp / d) % 10); }
  }
  out[k] = 0;
  return out;
}
