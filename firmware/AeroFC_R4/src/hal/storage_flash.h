// storage_flash.h - keeps the settings in the R4's 8 KB data flash (the "EEPROM").
//
// Why not the EEPROM library?  On the R4 its put() erases and rewrites a whole 1 KB flash block for EVERY changed
// byte, which takes seconds for our ~300 byte settings.  Here we use the data flash directly:
//   two 1 KB slots, each  {magic, sequence number, length, crc32, settings...}
//   save  = erase the OLDER slot and write there (one erase + one write, ~tens of ms)
//   load  = take the newest slot whose CRC is correct
// So a power cut during a save can never destroy the last good settings.
#pragma once
#include <Arduino.h>

namespace storage {
bool begin();
bool load(uint8_t *buf, size_t cap, size_t &len);   // newest valid settings
bool save(const uint8_t *buf, size_t len);
}
