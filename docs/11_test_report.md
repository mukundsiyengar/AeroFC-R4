# 11. Test report (closed-loop, software-in-the-loop)

**How:**
- The flight code in `firmware/AeroFC_R4/src/core` is compiled unchanged to WebAssembly.
- It flies a replica of the Gazebo `betaflight_x3`:
  - mass 1.52 kg, inertia from `model.sdf`
  - the MulticopterMotorModel (k·ω², motor time constants 12.5/25 ms, yaw moment 0.016, rotor drag, thrust drop with axial speed)
  - 1 ms physics, ground contact
  - Gazebo-style sensors
- The drone sits behind a replica of `hil_bridge_v1.py`:
  - handshake, 4-packet sensor burst every 20 ms
  - zero packets before data arrives
  - 1.5 s failsafe
  - re-handshake on HELLO

Run the suite with `tools\run_tests_headless.bat` or open `simulator/tests.html`. It runs faster than real time (about 3 s in total).

**Result: 12 / 12 scenarios passed.** Firmware build for the UNO R4 Minima: 92 556 B flash (35 %), 13 956 B RAM (42 %).

| # | Scenario | Key results |
|---|---|---|
| 1 | Protocol | CRC32 = zlib. 11 packet types byte-identical to the official `hil_protocol.py`. Resync after garbage and a CRC error. The firmware answers HELLO with HELLO_ACK "UAV_HIL_V1". |
| 2 | Handshake | HELLO → ACK → START. 0.99 motor packets per sensor burst. Heartbeat gap max 0.201 s. STATUS telemetry received. |
| 3 | Auto take-off + hover | FLYING after 3.1 s, peak 1.61 m (target 1.5 m). Hover 1.498 m ± 0.001 m, tilt < 0.03°. Learned hover command 0.660. |
| 4 | Directions (mixer sign test) | Roll +0.4 → +9.7° and moves right. Pitch −0.4 → −9.7° (nose down) and moves forward. Yaw +0.5 → +71° in 2 s (clockwise). Heading drift after release 2.5°. |
| 5 | Step response + estimator | Roll step to 14.8°: rise 0.32 s, overshoot 2.8 %, settled 0.46 s. EKF error during a 5 s 15° tilt: roll RMS 0.76°, pitch RMS 0.58° (max 1.4°). |
| 6 | Climb stick | +1.94 m in 2 s full climb. Holds the new height (−4 mm in 3 s). Descends with the stick. |
| 7 | Stock `control_input.py` | 0.66 → manual, holds about 1.50 m. 0 → altitude hold at the current height. Sticky +0.1 roll → 2.05°. No false link-loss failsafe. |
| 8 | Landing | 1.5 m → touch-down → auto-disarm in 6.2 s, reason "landed". TAKEOFF works again. |
| 9 | Failsafes | FC silent → bridge stops the motors after 1.5 s → FC HELLO → bridge re-ACK → session running. STOP → disarm. KILL → off. Pilot stream lost → failsafe landing → disarmed. |
| 10 | Crash detection | Flip → motors off (reason CRASH). With crash detection OFF, the motors are not cut. |
| 11 | Robustness (each: take-off, roll/pitch/yaw manoeuvres, hover) | Stable with all of: +30 ms delay; 40 Hz bursts with ±3 ms jitter; inertia ×1.3 and ×0.7; mass +15 %; gyro noise 0.6 °/s + bias 1.1 °/s + accel/baro/compass noise; wind 3 m/s + gusts; 800 rad/s world (Motor max 0.8); motors twice as slow. Max tilt 12–14° (commanded 12.5°), altitude 1.44–1.52 m. |
| 12 | OLED menu + settings | Dashboard draws. Menu edit changes Roll P by 2 steps. The settings blob restores on another board. A corrupted blob is rejected by CRC. Arming is refused without sensor data. |

**Extra measurement (GPS-aided attitude):** over a 40 s sequence of aggressive manoeuvres in different directions, the attitude-estimate error is **0.95° RMS / 2.2° max with GPS aid**, against **3.35° RMS / 5.4° max without**.

**Limits of this validation:**
- The replica is built from the simulator's `model.sdf` and plugin parameters, not from Gazebo itself. A run in the real Gazebo (docs/08) is still the final check.
- The real board's OLED, buttons, flash and USB timing have not been tested (the board arrives tomorrow).
