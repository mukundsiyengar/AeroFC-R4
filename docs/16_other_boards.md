# 16. Which boards can run this firmware

The firmware was written for the **Arduino UNO R4 Minima**, and that is the board it has been flown on. This page separates what was actually flown, what only builds, and what would need work, so nothing here is a guess dressed up as a result.

| Board | Status | What that means |
|---|---|---|
| **Arduino UNO R4 Minima** | **Flown** | Every test in [docs/11](11_test_report.md) ran on this board. Use this one. |
| Arduino Nano R4 | Builds, never flown | Same RA4M1 chip and the same native USB. It builds with no changes, using 124 KB of flash and 17.6 KB of RAM. Its pin names match `board_config.h`. |
| Arduino UNO R4 WiFi | Builds, never flown | Same chip, but its USB goes through an ESP32 at 115200 baud. The sketch allows for that, and it builds, using 137 KB of flash and 20.5 KB of RAM. The link is slower than native USB, so check the control delay first. |
| Arduino UNO Q | Flown, with a different board layer | A port of this flight code flew all the same tests on a UNO Q. It needs its own board layer and a small program on the Q's Linux side, and neither is in this repository. It shows how portable `src/core` is, but this sketch won't build for the Q as it is. |
| Other 32-bit boards with a floating-point unit | Needs a port, never tried | For example Teensy 4.x, an ESP32-S3 or an STM32F4. See below. |
| Arduino UNO R3, classic Nano, Mega | **Can't run it** | The firmware needs 16 KB of RAM, and an UNO R3 has 2 KB. These chips also have no floating-point unit. |

The simulator's bridge treats the board as a serial port, so any board that speaks the protocol works with it.

## What the firmware needs

| Need | Measured on the R4 Minima | Why |
|---|---|---|
| Flash | 124 KB | The program |
| RAM | 16.5 KB, plus a few KB of stack | The program's data |
| Speed | One control step takes about 0.85 ms, worst 1.06 ms, out of the 20 ms between sensor bursts | At least the R4's 48 MHz is comfortable |
| Floating point | Hardware floating-point unit | The filters do a lot of floating-point maths |
| USB serial | Native USB is best | The simulator sends 7.4 KB/s and expects an answer within the delay limit of about 80 ms |
| I2C, 4 inputs | For the OLED and the buttons | Optional: it flies without them |

## Porting to another board

The flight code in `firmware/AeroFC_R4/src/core` is plain C++ with no Arduino functions. It already runs in three places: on the board, in a browser as WebAssembly, and on Linux as a normal program. So a port only has to replace these files:

| File | What it does | On the R4 it uses |
|---|---|---|
| `AeroFC_R4.ino` | Moves bytes between USB and `FcApp`, runs the loop | `Serial`, including `Serial.dtr()` |
| `board_config.h` | Pin numbers | Your wiring |
| `src/hal/oled_ssd1306.*` | OLED over I2C | `Wire` and its timeout |
| `src/hal/buttons.*` | The four buttons | `pinMode`, `digitalRead` |
| `src/hal/storage_flash.*` | Saved settings | The RA4M1's data flash (`DataFlashBlockDevice`) |
| `src/hal/reset_cause.*` | Why it restarted, and the watchdog | RA4M1 registers and `WDT.h` |

The two you will have to rewrite are `storage_flash` and `reset_cause`, because they depend on the chip. To begin with they can do nothing: `storage::load` can return false, so the board starts with default settings, and the watchdog can stay off while you get the rest working.

**How to check a port:**

1. Build it, and run `hil_bridge_v1.py --port <your port>`. The board must answer `HELLO_ACK`, and the drone must take off.
2. On the OLED's SYSTEM page, or in the bridge's own output, check the sensor rate and the control delay. Over about 80 ms of delay the drone wobbles.
3. Run `RUN_ALL_TESTS.bat`, or `tools/flight_tests/*_run.sh board` on Linux. The same tests that passed here then judge your board against Gazebo's own true position.

If you get it flying on another board, a pull request adding it to this table would be very welcome.
