#include "storage_flash.h"
#include <DataFlashBlockDevice.h>
#include "../core/hil_protocol.h"   // crc32

namespace storage {

static const uint32_t SLOT_MAGIC = 0x534C4641u;   // "AFLS"
static const uint32_t SLOT_SIZE = 1024;
static const uint32_t SLOT_ADDR[2] = { 0, 1024 };
static const size_t HEADER = 16;

struct Header {
  uint32_t magic, seq, len, crc;
};

static DataFlashBlockDevice &dev() { return DataFlashBlockDevice::getInstance(); }
static bool ready = false;

bool begin() {
  ready = dev().init() == 0;
  return ready;
}

static bool readSlot(int s, Header &h, uint8_t *buf, size_t cap) {
  if (dev().read(&h, SLOT_ADDR[s], sizeof(h)) != 0) return false;
  if (h.magic != SLOT_MAGIC || h.len == 0 || h.len > SLOT_SIZE - HEADER || h.len > cap) return false;
  if (dev().read(buf, SLOT_ADDR[s] + HEADER, h.len) != 0) return false;
  return hil::crc32(buf, h.len) == h.crc;
}

bool load(uint8_t *buf, size_t cap, size_t &len) {
  if (!ready) return false;
  static uint8_t tmp[SLOT_SIZE];
  Header h[2];
  bool ok[2];
  ok[0] = readSlot(0, h[0], buf, cap);
  ok[1] = readSlot(1, h[1], tmp, sizeof(tmp));
  if (!ok[0] && !ok[1]) return false;
  const int best = (!ok[0] || (ok[1] && h[1].seq > h[0].seq)) ? 1 : 0;
  if (best == 1) memcpy(buf, tmp, h[1].len);
  len = h[best].len;
  return true;
}

bool save(const uint8_t *data, size_t len) {
  if (!ready || len == 0 || len > SLOT_SIZE - HEADER) return false;
  // which slot is newer?  write into the other one
  static uint8_t tmp[SLOT_SIZE];
  Header h[2];
  const bool ok0 = readSlot(0, h[0], tmp, sizeof(tmp));
  const bool ok1 = readSlot(1, h[1], tmp, sizeof(tmp));
  uint32_t seq = 1;
  int target = 0;
  if (ok0 && (!ok1 || h[0].seq >= h[1].seq)) { target = 1; seq = h[0].seq + 1; }
  else if (ok1) { target = 0; seq = h[1].seq + 1; }

  // header + data, padded to a multiple of 4 bytes with 0xFF (erased flash value)
  memset(tmp, 0xFF, sizeof(tmp));
  Header nh = { SLOT_MAGIC, seq, (uint32_t)len, hil::crc32(data, len) };
  memcpy(tmp, &nh, sizeof(nh));
  memcpy(tmp + HEADER, data, len);
  const size_t total = (HEADER + len + 3) & ~(size_t)3;
  if (dev().erase(SLOT_ADDR[target], SLOT_SIZE) != 0) return false;
  if (dev().program(tmp, SLOT_ADDR[target], total) != 0) return false;
  // verify by reading back
  Header vh;
  return readSlot(target, vh, tmp, sizeof(tmp)) && vh.seq == seq;
}

}  // namespace storage
