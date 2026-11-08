# 9. Demo checklist

## The night before

- [ ] `tools\check_firmware.bat` says COMPILE OK. Upload the final firmware with `UPLOAD_FIRMWARE.bat`.
- [ ] `RUN_ALL_TESTS.bat quick`, or at least `tools\run_tests_headless.bat`, says ALL PASS.
- [ ] Fly once in the real Gazebo: `START_ALL.bat`. See [docs/08](08_gazebo_wsl_setup.md).
- [ ] **MENU > Storage > Save settings**, and check `Boot: saved` after unplugging and plugging back in.
- [ ] Pack the USB-C data cable, a spare cable, the board with its OLED and buttons, and the gamepad if you use one.

## On a laptop that has only the simulator

Nothing of ours is installed there. **Bring [the sheet in docs/12](12_hand_over_sheet.md).**

1. **Plug the Arduino in first**, then start Gazebo and the bridge (`hil_bridge_v1.py --port /dev/ttyACM0`). The bridge says HELLO only once at start-up.
2. The OLED shows `PC:HANDSHAKE`, then `PC:RUNNING`. The bridge prints `HELLO_ACK received`, then `HIL session started`.
3. The drone **takes off to 1.5 m by itself** and hovers.
4. Fly with `control_input.py`:
   - roll, pitch and yaw both ways
   - the drone holds its height and heading when the sticks are centred
   - `w` goes forward in their keyboard tool, and `--pitch -0.2` also goes forward. If you want it the other way: **MENU > Input > Inv pitch > ON** (and roll or yaw likewise). No upload is needed.
5. Let go: it brakes and **holds its position** (POSITION page: `HOLDING`, and the distance).
6. Show return to home: `python3 control_input.py --throttle 22 --duration 0.2`.
7. Show the OLED pages: **PLOTS** (UP once from the first page: live RC input, attitude and altitude), ATTITUDE, ALTITUDE, POSITION, MOTORS, RC INPUT, SYSTEM.
8. Land, any of these ways: hold SELECT on the dashboard, **MENU > Flight > Land**, `j` twice in their tool, or `python3 control_input.py --throttle 13 --duration 0.2`.

## Live plots

| Option | What you need |
|---|---|
| **The OLED's PLOTS page** | Nothing else. It runs on the R4. |
| Our page as a plotter | A **USB-TTL adapter**: R4 D1 to adapter RX, GND to GND. On the page: More connections > **Telemetry only**, then pick the adapter's port. It works alongside their unmodified bridge. |
| Our `bridge_plus.py` | The same as their bridge, plus live plots in the browser ([docs/08](08_gazebo_wsl_setup.md)). |
| No extras | The OLED shows live attitude, height, RC input and motors. Their bridge also writes a CSV with height and motors. |
