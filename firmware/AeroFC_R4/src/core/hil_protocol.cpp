#include "hil_protocol.h"
#include <string.h>

namespace hil {

// 16-entry ("nibble") table for the reflected IEEE polynomial 0xEDB88320: small and still fast.
static const uint32_t kCrcNibble[16] = {
  0x00000000u, 0x1DB71064u, 0x3B6E20C8u, 0x26D930ACu, 0x76DC4190u, 0x6B6B51F4u, 0x4DB26158u, 0x5005713Cu,
  0xEDB88320u, 0xF00F9344u, 0xD6D6A3E8u, 0xCB61B38Cu, 0x9B64C2B0u, 0x86D3D2D4u, 0xA00AE278u, 0xBDBDF21Cu
};

uint32_t crc32(const uint8_t *data, size_t len, uint32_t crc) {
  crc = ~crc;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    crc = (crc >> 4) ^ kCrcNibble[crc & 0x0Fu];
    crc = (crc >> 4) ^ kCrcNibble[crc & 0x0Fu];
  }
  return ~crc;
}

void Parser::push(uint8_t b) {
  if (n_ >= MAX_PACKET) drop(1);   // cannot happen with the checks in next(), but never overflow
  buf_[n_++] = b;
}

void Parser::drop(size_t k) {
  if (k >= n_) { n_ = 0; return; }
  memmove(buf_, buf_ + k, n_ - k);
  n_ -= k;
}

bool Parser::next(Packet &out) {
  while (n_ > 0) {
    if (buf_[0] != MAGIC0) { drop(1); continue; }
    if (n_ < 2) return false;
    if (buf_[1] != MAGIC1) { drop(1); continue; }
    if (n_ < HEADER_SIZE) return false;
    const uint16_t len = (uint16_t)(buf_[4] | (buf_[5] << 8));
    if (buf_[2] != VERSION || len > MAX_PAYLOAD) { framingErrors++; drop(1); continue; }
    const size_t total = HEADER_SIZE + len + CRC_SIZE;
    if (n_ < total) return false;
    const uint32_t rx = (uint32_t)buf_[total - 4] | ((uint32_t)buf_[total - 3] << 8) |
                        ((uint32_t)buf_[total - 2] << 16) | ((uint32_t)buf_[total - 1] << 24);
    if (crc32(buf_, HEADER_SIZE + len) != rx) { crcErrors++; drop(1); continue; }

    out.type = buf_[3];
    out.length = len;
    out.sequence = (uint32_t)buf_[6] | ((uint32_t)buf_[7] << 8) | ((uint32_t)buf_[8] << 16) | ((uint32_t)buf_[9] << 24);
    uint64_t ts = 0;
    for (int i = 7; i >= 0; i--) ts = (ts << 8) | buf_[10 + i];
    out.timestampNs = ts;
    if (len) memcpy(out.payload, buf_ + HEADER_SIZE, len);
    goodPackets++;
    drop(total);
    return true;
  }
  return false;
}

size_t build(uint8_t *out, size_t cap, uint8_t type, uint32_t seq, uint64_t timestampNs,
             const uint8_t *payload, uint16_t len) {
  const size_t total = HEADER_SIZE + len + CRC_SIZE;
  if (len > MAX_PAYLOAD || total > cap) return 0;
  out[0] = MAGIC0;
  out[1] = MAGIC1;
  out[2] = VERSION;
  out[3] = type;
  out[4] = (uint8_t)(len & 0xFF);
  out[5] = (uint8_t)(len >> 8);
  for (int i = 0; i < 4; i++) out[6 + i] = (uint8_t)(seq >> (8 * i));
  for (int i = 0; i < 8; i++) out[10 + i] = (uint8_t)(timestampNs >> (8 * i));
  if (len) memcpy(out + HEADER_SIZE, payload, len);
  const uint32_t c = crc32(out, HEADER_SIZE + len);
  for (int i = 0; i < 4; i++) out[HEADER_SIZE + len + i] = (uint8_t)(c >> (8 * i));
  return total;
}

float getF32(const uint8_t *p) { float v; memcpy(&v, p, 4); return v; }
double getF64(const uint8_t *p) { double v; memcpy(&v, p, 8); return v; }
void putF32(uint8_t *p, float v) { memcpy(p, &v, 4); }
void putU16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v & 0xFF); p[1] = (uint8_t)(v >> 8); }
void putI16(uint8_t *p, int16_t v) { putU16(p, (uint16_t)v); }
uint16_t getU16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
int16_t getI16(const uint8_t *p) { return (int16_t)getU16(p); }

}  // namespace hil
