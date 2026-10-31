# 10. Troubleshooting

| Problem | Cause / fix |
|---|---|
| `START_SIMULATOR.bat` opens a page that says it can't load fc.wasm | The page must come from the local server, not a double-clicked HTML file. Use the .bat and keep its black window open. v3 uses **port 8767** (v2: 8766, v1: 8765). |
| The page behaves like the old version | Make sure the address is `localhost:8767` (v3) and press Ctrl+F5 once. The v2 server never lets the browser keep old copies. |
| "Connect Arduino" does nothing or isn't supported | Use **Edge or Chrome** (Web Serial). Close the Arduino IDE Serial Monitor; only one program can open the COM port. |
| No COM port appears | Use a data USB-C cable. If the R4 is attached to Ubuntu (usbipd), run `gazebo\6_ARDUINO_back_to_Windows.bat`. |
| Upload fails (for example `get_status: LIBUSB_ERROR_TIMEOUT` or `Lost device after RESET?`) | Use `UPLOAD_FIRMWARE.bat`: it works even with an older firmware on the board. Otherwise close the simulator page and the Serial Monitor. **Double-press RESET** (the LED "breathes") and upload again. |
| Bridge log stops at "Sending HELLO" / `HELLO_ACK timeout` | Wrong port, or old firmware. Re-upload, press **Stop**, then Connect again. |
| OLED stays black | Check GND/VCC and SDA→A4, SCL→A5 (pin order differs between modules). The board flies without the OLED (LED shows status). |
| OLED shows garbage or is shifted by 2 pixels | It is an SH1106 (usually 1.3") rather than an SSD1306. Tell us and we change the column offset in `oled_ssd1306.cpp`. |
| A button does nothing | Use diagonal legs of the button; the other leg goes to GND. |
| "ARM: not level" / "EKF not ready" / "no sensors" | Wait 1 s after the simulator starts (the first packets are zeros). Arming needs sensor data, under 15° tilt, and no calibration running. |
| **Drone flies fine, then starts to drift in one direction** | Look at the page while it drifts. **Card 3 (Pilot):** with your hands off, `roll` and `pitch` must be 0.00. If not, or if a red "Stick not centred" appears, the gamepad is the cause: use the keyboard, or unplug the gamepad. **Position hold card:** MOVING = it sees a stick push; OFF / FAILED / NOT READY = position hold isn't flying it (the POSITION page says why). **Status card:** see the next rows. Still no idea? Press **Record log**, fly until it drifts, and send the CSV. |
| Status card: **Control delay** above 60 ms, or **sim speed** well below 1.00x | The PC is busy, or USB is slow. The simulation slows down on purpose instead of letting the drone wobble (it gets unstable above about 90 ms of delay). Close other programs (Arduino IDE compile, OneDrive sync), keep the page visible. |
| Keys press but the drone doesn't react, or KILL / sticks appear out of nowhere | Two pilots: a simulator page left open is connected to the same bridge. Close the extra page. |
| Gyro noise or drift sliders in the simulator | Normal results with position hold ON: noise 2 deg/s moves it about 0.3 m, drift 3 deg/s about 2 m for some seconds while the filter learns the drift. With position hold OFF nothing brings it back, so it travels (about 90 m for drift 3 deg/s). Keep position hold ON. |
| Status card: **Estimate vs truth** large (over 1 m or 3 deg) | The flight controller's estimate is wrong. Send a log. |
| Status card: **Heading offset** far from 0 | Normal only if the compass really is off. In the simulator and Gazebo it should stay within a few degrees of 0. Send a log if it doesn't. |
| Organisers' joystick (not our page) drifts the drone | Their stick may rest off-centre. Raise **MENU → Input → Deadband** (default 0.02, up to 0.20). |
| Drone drifts away after letting go of the stick | Position hold is off or not ready: the POSITION page and the bottom line say why (OFF, NO GPS, NO COMPASS, FAILED). Switch it on: Flight → Pos hold (or P). With it off, the X3 drifts (no drag): counter-steer. |
| POSITION page: FAILED | The safety check switched position hold off (the drone kept getting further from its spot for 3 s). It tries again by itself after 5 s; after 3 failures in a row it waits for the next arm. Usually very strong wind. Otherwise check Estimator → Mag decl (90.8 for Gazebo). |
| Won't climb above some height | Flight → Alt limit is ON; Max alt is the limit above the take-off point. |
| Forward/back reversed with the organisers' tool | MENU → Input → **Inv pitch** (their keyboard tool's `w` sends +pitch, the protocol says + = backward). |
| Lands after 2 s without input | Pilot-link failsafe: our 25 Hz pilot stream stopped (browser tab closed or "Cut RC link" pressed). Set Failsafe → RC lost → HOLD or NONE if you prefer. |
| "CRASH" and motors off | Tilt over 70° for 0.2 s, or an impact while tilted. Switch off in Features → Crash det if needed. |
| SYSTEM page: IMU much below 50 Hz | The simulator PC is slow (Gazebo GUI in WSL). Run Gazebo headless. |
| v1 problem: OLED freezes, Windows USB sounds repeat, "WATCHDOG RESET!" with the Arduino IDE open | Fixed in v2 (docs/03, "Why v1 kept restarting"). Upload v2. |
| OLED message "WATCHDOG: hang!" / SYSTEM "Reset: WDT HANG!" | The program really hung for 1 s **while armed** (v2 only runs the watchdog then). Note what you were doing and report it. |
| OLED garbled for a moment, then fine | An I2C glitch the driver repaired (SYSTEM: I2C err counts them). If it happens often, shorten the OLED wires. |
| Gazebo: the page shows no data | The page retries by itself while Gazebo starts. The Ubuntu window must say `bridge_plus ... http://127.0.0.1:8770`. If needed: More connections → Gazebo in WSL. |
| **Gazebo: the drone hovers at 1.5 m and my keys do nothing** | Read the banner at the top of the page, and the black Ubuntu window (docs/08). A red banner `Gazebo is running...` means this page isn't flying Gazebo: press **Connect to Gazebo**. An orange banner `Click on this page once` means the keys are going to another window. |
| Commands feel slow in Gazebo | The address must be `http://127.0.0.1:8770`, not `localhost`. With `localhost`, Windows tries IPv6 first, and that added 320 ms to every command. |
| OLED says `BRIDGE TOO SLOW` and the drone stays on the ground | Their bridge is in its slow mode (it prints `Gazebo Transport: Subprocess Fallback`), with about one motor update a second. Nothing can fly on that. Fix on that PC: `sudo apt install python3-gz-transport13 python3-gz-msgs10`, then start the bridge again. |
| Motors always show about 66 % | That is correct: the same weight needs the same thrust at any height (docs/04). Watch **Throttle stick**, **Climb speed** and **Power above hover** on the page. |
| Gazebo: "No /dev/ttyACM0" | Run `gazebo\2_ARDUINO_to_Ubuntu.bat` first and keep its window open. |
| Settings lost after power off | Save with MENU → Storage → Save (refused while armed). After a firmware update with new parameters, defaults are used ("dflt" on SYSTEM). |
| Gamepad not detected | Press a button on it once while the page has focus. Xbox-style pads work best. |
