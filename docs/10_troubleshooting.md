# 10. Troubleshooting

| Problem | Cause / fix |
|---|---|
| `START_SIMULATOR.bat` opens a page that says it can't load fc.wasm | The page must come from the local server, not a double-clicked HTML file. Use the .bat and keep its black window open. |
| "Connect Arduino" does nothing or isn't supported | Use **Edge or Chrome** (Web Serial). Close the Arduino IDE Serial Monitor; only one program can open the COM port. |
| No COM port appears | Use a data USB-C cable. If the R4 is attached to WSL (usbipd), run `usbipd detach --busid …`. |
| Upload fails | Close the simulator page and the Serial Monitor. **Double-press RESET** (the LED "breathes") and upload again. |
| Bridge log stops at "Sending HELLO" / `HELLO_ACK timeout` | Wrong port, or old firmware. Re-upload, press **Stop**, then Connect again. |
| OLED stays black | Check GND/VCC and SDA→A4, SCL→A5 (pin order differs between modules). The board flies without the OLED (LED shows status). |
| OLED shows garbage or is shifted by 2 pixels | It is an SH1106 (usually 1.3") rather than an SSD1306. Tell us and we change the column offset in `oled_ssd1306.cpp`. |
| A button does nothing | Use diagonal legs of the button; the other leg goes to GND. |
| "ARM: not level" / "EKF not ready" / "no sensors" | Wait 1 s after the simulator starts (the first packets are zeros). Arming needs sensor data, under 15° tilt, and no calibration running. |
| Drone drifts away after letting go of the stick | Normal for the X3 (no drag, ATTI-mode behaviour). Counter-steer. |
| Forward/back reversed with the organisers' tool | MENU → Input → **Inv pitch** (their keyboard tool's `w` sends +pitch, the protocol says + = backward). |
| Lands after 2 s without input | Pilot-link failsafe: our 25 Hz pilot stream stopped (browser tab closed). Set Failsafe → FS action → HOLD or NONE if you prefer. |
| "CRASH" and motors off | Tilt over 70° for 0.2 s, or an impact while tilted. Switch off in Features → Crash det if needed. |
| SYSTEM page: IMU much below 50 Hz | The simulator PC is slow (Gazebo GUI in WSL). Run Gazebo headless. |
| OLED: "WATCHDOG RESET!" | The program hung for 2 s and the board restarted itself. Note what you were doing and report it. |
| Settings lost after power off | Save with MENU → Storage → Save (refused while armed). After a firmware update with new parameters, defaults are used ("dflt" on SYSTEM). |
| Gamepad not detected | Press a button on it once while the page has focus. Xbox-style pads work best. |
