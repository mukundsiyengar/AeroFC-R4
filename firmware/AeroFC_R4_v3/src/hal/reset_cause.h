// reset_cause.h - why did the board (re)start, and a watchdog that only runs while armed.
//
// Why the watchdog is no longer started at power-up (the v1 "reboot loop" with the Arduino IDE open):
//   The UNO R4 Minima is uploaded with dfu-util, which asks the running sketch to "detach".  The board core
//   then reboots itself by starting the hardware watchdog.  If the sketch had ALREADY started the watchdog,
//   the core cannot, and it waits forever inside the USB interrupt until the sketch's watchdog fires:
//   OLED frozen, then "WATCHDOG RESET!", USB disconnect / reconnect.  Starting the watchdog only when the drone
//   is armed avoids that completely on the bench, and still protects a flying drone against a program hang.
#pragma once
#include <Arduino.h>

enum ResetCause : uint8_t {
  RC_POWER_ON = 0,   // power applied
  RC_RESET_PIN,      // reset button
  RC_SOFTWARE,       // software reset (e.g. 1200-baud touch / bootloader request)
  RC_UPLOAD_WDT,     // watchdog started by the core for an upload / DFU detach (normal)
  RC_HANG_WDT,       // OUR watchdog fired: the program stopped while armed
  RC_LOW_VOLTAGE,    // supply dropped too low
  RC_OTHER
};

namespace resetinfo {
ResetCause readAndClear();          // call first thing in setup()
const char *name(ResetCause c);     // short text for the OLED (<= 12 characters)
void armWatchdog(uint32_t ms);      // first arm: start the hardware watchdog (cannot be stopped afterwards)
bool watchdogRunning();
void kick();                        // call every loop
}
