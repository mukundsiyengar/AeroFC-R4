# 3. Upload the firmware and first run

## A. Upload

**Easiest:** press **Stop** on the simulator page, then double-click **`UPLOAD_FIRMWARE.bat`**. It compiles and uploads, with no Arduino IDE window needed. It also works when the board is stuck.

**Or with the Arduino IDE:**

1. Plug the R4 Minima into the PC with the USB-C cable.
2. Open **Arduino IDE 2**, then **File → Open** → `AeroFC_R4_v3\firmware\AeroFC_R4_v3\AeroFC_R4_v3.ino`. All the `src` files come with it automatically. The IDE title shows `AeroFC_R4_v3`, so you can see it is the new version.
3. **Tools → Board → Arduino UNO R4 Boards → Arduino UNO R4 Minima**.
4. **Tools → Port** → the COM port that appears when you plug the board in (for example COM5).
5. Press **Upload** (the → arrow). The IDE should finish with "Done uploading".
   - No libraries are needed: the OLED driver, the font and the storage code are all inside the project.
   - **If the upload fails:**
     - Close the simulator page first (only one program can use the COM port).
     - Is the board attached to Ubuntu (usbipd)? Run `gazebo\6_ARDUINO_back_to_Windows.bat`.
     - If it ends with `get_status: LIBUSB_ERROR_TIMEOUT` or `Lost device after RESET?`, the board is still running an older v2 firmware. Use `UPLOAD_FIRMWARE.bat` once; after that, IDE uploads work too.
     - If it still fails, **double-press RESET** quickly (the LED "breathes"), select the new port and upload again.
6. v2 has new settings, so the first start uses the defaults (the SYSTEM page says `dflt`). Choose **MENU → Storage → Save settings** once.

## B. First checks on the board

| Check | Expected |
|---|---|
| OLED | Shows **STATUS** and **PC: WAIT PC**. The bottom line explains what you see. |
| **SYSTEM page** (press DOWN until "SYSTEM") | **Reset: power-on** (or "upload/USB" right after an upload), **WDT: off**, **I2C err: 0**. |
| Arduino IDE open, Serial Monitor closed, wait 2 minutes | The OLED keeps running. **No** Windows USB sounds, **no** "WATCHDOG" message. (This is the v1 bug, see below.) |
| L LED | Slow blink. |
| Button DOWN | Next page (ATTITUDE, ALTITUDE, POSITION, MOTORS, RC INPUT, SENSORS, SYSTEM). |
| Button SELECT | Opens the MENU. BACK returns. **Hold BACK** in a menu: a help page about the selected item. |
| Storage | **MENU → Storage → Save settings** shows "Saved to flash". Unplug, plug in again: **MENU → Storage** shows "Boot: saved". |

## C. Fly the board in the loop (Windows simulator)

1. Double-click **`START_SIMULATOR.bat`**. Keep its black window open; Edge opens the page.
2. Press **Connect Arduino R4 (USB)**, pick the board's COM port, then **Connect**.
3. The **Bridge log** must show the same lines as the official bridge:
   ```
   Sending HELLO to Arduino...
   HELLO_ACK received from Arduino
   Sending START to Arduino...
   HIL session started successfully
   ```
4. The drone takes off to 1.5 m by itself and hovers. The OLED shows **PC: RUNNING** and **TAKEOFF**, then **FLYING**. Position hold takes over (POSITION page: **HOLDING**).
5. Fly with the keyboard or gamepad ([04_flying_controls.md](04_flying_controls.md)). Watch the live plots. Press **L** to land; it disarms itself on the ground.
6. **SYSTEM page:** IMU about 50 Hz. **Step** = microseconds per control step (a few hundred µs). **CRC err** should stay 0.
7. **Status card on the page:**
   - **Control delay:** normally under 40 ms. The page never lets the physics run more than 70 ms ahead of the board: if USB or the PC is slow, the simulation slows down instead (**sim speed** below 1.00x), and the drone stays steady.
   - **Estimate vs truth:** normally a few cm and under 1 deg.

## Why v1 kept restarting (fixed in v2)

What you saw with v1, with the Arduino IDE open:
- the OLED froze
- Windows played the USB connect/disconnect sound again and again
- the OLED said **WATCHDOG RESET!**

The cause:

1. The R4 Minima is uploaded with `dfu-util`. When the IDE checks the board (it does this in the background while it is open), it can send a "detach" request.
2. The R4 core answers that request by **restarting the board through the watchdog timer**.
3. v1 had already started the watchdog in `setup()`, so the core's restart failed. The core then waits forever inside the USB interrupt (`while(1)`). The program stops, so the OLED freezes.
4. Our own watchdog fired 1–2 s later ("WATCHDOG RESET!"). USB disconnected and connected again (the sounds), and it could happen again.

Two more upload problems were found and fixed by testing on a real board:
- **`get_status: LIBUSB_ERROR_TIMEOUT`:** the firmware sent packets even when no program on the PC was listening. Unread data stuck in the USB buffer blocked the IDE's upload request. Now it only sends when a PC program has the port open (DTR on) or has sent something in the last 2 s.
- **`Lost device after RESET?` after a flight:** the flight watchdog keeps running after a flight, and the core couldn't restart the board for the upload. Now the firmware sees the upload request and restarts straight into the bootloader.

The fixes in v2:

| Fix | Effect |
|---|---|
| The watchdog starts **only when you arm** (it is a flight safety feature) | On the bench the IDE's restart works normally: no freeze, no loop. |
| The true reset cause is shown on the SYSTEM page | `power-on`, `reset btn`, `upload/USB`, `software`, `WDT HANG!` (only if our program really hung while armed), `low volt`. |
| The OLED driver checks every I2C transfer | If one fails, it pauses 0.1 s, restarts the I2C bus, re-initialises the display and redraws it. It also refreshes the display settings every 5 s, so a glitched OLED repairs itself. |
| The telemetry port D1 is **off** by default | v1 spent about 6 ms per packet writing it even with nothing connected. Switch it on in Features → Telem D1 only if you use a USB-TTL adapter. |

## D. Re-check the code without a board

- `tools\check_firmware.bat`: compiles the firmware for the R4 and prints flash and RAM use.
- `tools\run_tests_headless.bat`: runs all automated flight tests.
- **Upload the firmware again after every update of this folder.** The page and the firmware belong together.
- After changing anything in `firmware/AeroFC_R4_v3/src/core`:
  - run `python tools\build_wasm.py` (the simulator's software FC)
  - run `python tools\sitl\build_sitl.py` (the Linux SITL), then `gazebo\1_UPDATE_tools_in_Ubuntu.bat`
