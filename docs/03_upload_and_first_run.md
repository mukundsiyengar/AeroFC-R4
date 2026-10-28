# 3. Upload the firmware and first run (when the board arrives)

## A. Upload

1. Plug the R4 Minima into the PC with the USB-C cable.
2. Open **Arduino IDE 2**, then **File → Open** → `AeroFC_R4\firmware\AeroFC_R4\AeroFC_R4.ino`. All the `src` files come with it automatically.
3. **Tools → Board → Arduino UNO R4 Boards → Arduino UNO R4 Minima**.
4. **Tools → Port** → the COM port that appears when you plug the board in (for example COM5).
5. Press **Upload** (the → arrow). The first compile takes about 1 minute. The IDE should finish with "Done uploading".
   - No libraries are needed: the OLED driver, the font and the storage code are all inside the project.
   - **If the upload fails:** close the simulator page first (only one program can use the COM port), then try again. If it still fails, **double-press the board's RESET button quickly**. The LED starts "breathing" (bootloader mode), a new COM port may appear; select it and upload again.

## B. First checks on the board (no PC program yet)

| Check | Expected |
|---|---|
| OLED | Shows **STATUS 1/7** and **PC: WAIT PC**. |
| L LED | Slow blink. |
| Button DOWN | Next page (ATTITUDE, ALTITUDE, MOTORS, RC INPUT, SENSORS, SYSTEM). |
| Button SELECT | Opens the MENU. BACK returns. |
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
4. The drone takes off to 1.5 m by itself and hovers. The OLED shows **PC: RUNNING** and **TAKEOFF**, then **FLYING ALTHOLD**.
5. Fly with the keyboard or gamepad ([04_flying_controls.md](04_flying_controls.md)). Watch the live plots. Press **L** to land; it disarms itself on the ground.
6. Look at **SYSTEM** (page 7/7) on the OLED: **IMU ~50 Hz**, **Step** = microseconds the R4 needs per control step (expect a few hundred µs), **CRC err** should stay 0.

## D. If something does not work

See [10_troubleshooting.md](10_troubleshooting.md). Report what the OLED and the bridge log say and we fix it together.

## E. Re-check the code without a board

- `tools\check_firmware.bat` compiles the firmware for the R4 and prints the flash and RAM use.
- `tools\run_tests_headless.bat` runs all automated flight tests.
- After changing anything in `firmware/AeroFC_R4/src/core`, run `python tools\build_wasm.py` so the simulator's software FC uses the new code.
