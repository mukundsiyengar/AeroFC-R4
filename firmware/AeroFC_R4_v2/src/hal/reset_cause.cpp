#include "reset_cause.h"
#include <WDT.h>

// Marker "our watchdog was running" kept in the battery-backup registers (they survive resets; normal RAM may be
// overwritten by the bootloader).  The board core itself uses VBTBKR[0..3]; we use [8..15].
static const uint32_t MARKER = 0x57444741u;   // "AGDW"
static const uint16_t PRCR_KEY = 0xA500u;
static bool running = false;
extern bool is_watchdog_reset_in_progress_for_upload;   // set by the board core on an upload request (SerialUSB.cpp)

static void writeMarker(uint32_t v) {
  R_SYSTEM->PRCR = (uint16_t)(PRCR_KEY | 0x2u);     // unlock (PRC1)
  for (int i = 0; i < 4; i++) {
    R_SYSTEM->VBTBKR[8 + i] = (uint8_t)(v >> (8 * i));
    R_SYSTEM->VBTBKR[12 + i] = (uint8_t)(~v >> (8 * i));
  }
  R_SYSTEM->PRCR = (uint16_t)PRCR_KEY;              // lock
}

static bool markerSet() {
  uint32_t v = 0, inv = 0;
  for (int i = 0; i < 4; i++) {
    v |= (uint32_t)R_SYSTEM->VBTBKR[8 + i] << (8 * i);
    inv |= (uint32_t)R_SYSTEM->VBTBKR[12 + i] << (8 * i);
  }
  return v == MARKER && inv == (uint32_t)~MARKER;
}

namespace resetinfo {

ResetCause readAndClear() {
  const uint8_t r0 = R_SYSTEM->RSTSR0;
  const uint16_t r1 = R_SYSTEM->RSTSR1;
  const bool warm = R_SYSTEM->RSTSR2_b.CWSF;
  const bool ourWdt = markerSet();
  ResetCause c;
  if (r0 & 0x01u) c = RC_POWER_ON;                          // PORF
  else if (!warm) c = RC_POWER_ON;                          // cold start
  else if (r0 & 0x0Eu) c = RC_LOW_VOLTAGE;                  // LVD0/1/2
  else if (r1 & 0x02u) c = ourWdt ? RC_HANG_WDT : RC_UPLOAD_WDT;   // WDTRF
  else if (r1 & 0x01u) c = RC_OTHER;                        // IWDT
  else if (r1 & 0x04u) c = RC_SOFTWARE;                     // SWRF
  else c = RC_RESET_PIN;
  // clear: flags are cleared by writing 0 after reading 1; CWSF is set to 1 = "next reset is a warm start"
  if (r0) R_SYSTEM->RSTSR0 = 0;
  if (r1) R_SYSTEM->RSTSR1 = 0;
  R_SYSTEM->RSTSR2_b.CWSF = 1;
  writeMarker(0);
  return c;
}

const char *name(ResetCause c) {
  switch (c) {
    case RC_POWER_ON: return "power-on";
    case RC_RESET_PIN: return "reset btn";
    case RC_SOFTWARE: return "software";
    case RC_UPLOAD_WDT: return "upload/USB";
    case RC_HANG_WDT: return "WDT HANG!";
    case RC_LOW_VOLTAGE: return "low volt";
    default: return "other";
  }
}

void armWatchdog(uint32_t ms) {
  if (running) return;
  if (WDT.begin(ms)) {
    running = true;
    writeMarker(MARKER);
  }
}

bool watchdogRunning() { return running; }

void kick() {
  if (!running) return;
  // The Arduino IDE asked for an upload: the core has set its "go to the bootloader" mark and expects its watchdog to
  // restart the board. Ours already runs (after a flight), so the core's cannot start: restart here. No hang mark.
  if (is_watchdog_reset_in_progress_for_upload) {
    writeMarker(0);
    NVIC_SystemReset();
  }
  WDT.refresh();
}

}  // namespace resetinfo
