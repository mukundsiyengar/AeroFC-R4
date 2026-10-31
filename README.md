# AeroFC-R4 v3: flight controller for the Arduino UNO R4 Minima

This is a quadcopter flight controller for the **Aerovidya 2026** hackathon. The **Arduino UNO R4 Minima** is the real flight controller. The drone itself (body, motors, sensors) is simulated by the organisers' **UAV HIL simulator** (Gazebo). They talk over USB using the official `"HI"` packet protocol.

This folder is **version 3**. Versions 1 and 2 (`..\AeroFC_R4\`, `..\AeroFC_R4_v2\`) are unchanged.

## New in v3: everything on the R4

At the event, the organisers plug the R4 into their laptop and use only their simulator from GitHub. Nothing of ours is installed there. **Print [docs/12](docs/12_hand_over_sheet.md) and hand it over with the board.**

- **Their keyboard tool works as labelled.** `W` goes forward, while a typed `--pitch -0.2` also goes forward as their protocol says. Only a typed `--pitch 0.1` or `-0.1` flies reversed, because it looks like a key press.
- **Land and take off again with only their tool.** `j` comes down and the motors stop by themselves after touch-down; `u` takes off again. `--throttle 13` lands, `--throttle 12` takes off.
- **Live plots on the board:** the OLED's new PLOTS page shows RC input, attitude and altitude.
- **Won't fly away on their bridge's slow mode.** Without Gazebo's Python bindings, their bridge updates the motors about once a second, and any flight controller flies away (their own fake Arduino reached over 1000 m). The R4 now stays on the ground and shows `BRIDGE TOO SLOW`. The first take-off waits about 4 s for a steady sensor rate.

## New in v2

- **Fixed: commands not reaching the drone in Gazebo.** The launchers opened the page without its Gazebo mode, so the keys flew the page's built-in copy. A banner on the page and a line in the black window now say whether commands really arrive.
- **Fixed: every command in Gazebo arriving about 320 ms late** (`localhost` tries IPv6 first; the page now uses `127.0.0.1`).
- **Gentle landing with the stick:** it slows down near the ground, comes straight down, and the motors stop on touch-down. Measured on the real R4 in Gazebo: 5.2 s from 1.5 m, touching at 0.28 m/s, 0.0 m sideways.
- **New card "Throttle and motors":** the stick, climb speed and power above hover, also on the OLED. About 66 % at any height is correct (docs/04).
- **Fixed: board restarting, OLED freezing, Windows USB sounds repeating** while the Arduino IDE was open.
  - Cause: the Arduino IDE asks the board to restart through the watchdog timer. v1 had already started the watchdog at power-up, so the board hung inside the USB code, the watchdog reset it ("WATCHDOG RESET!"), and it reconnected again and again.
  - Fix: the watchdog now starts only when you arm. The OLED driver also recovers by itself after any I2C glitch, and the SYSTEM page shows the true reset cause.
  - Details: [docs/03](docs/03_upload_and_first_run.md#why-v1-kept-restarting-fixed-in-v2).
- **Fixed: gyro drift crashing the drone.** A gyro that starts drifting in flight (3 °/s) used to tilt the drone over and crash it. Now it moves 1.7 m and the filter recovers (measured on a real R4).
- **Fixed: Arduino IDE uploads failing** (`LIBUSB_ERROR_TIMEOUT`, `Lost device after RESET?`). New: **`UPLOAD_FIRMWARE.bat`** uploads with one double-click.
- **Fixed: drone starting to drift in one direction after flying a while.** Four causes were found and fixed:
  - A gamepad stick resting off-centre was read as a push (deadzone now 25 %, with a red warning on the page).
  - With a real board, USB delay plus a busy page could get close to the point where the drone gets unstable. The physics now waits for the board.
  - The learned heading offset could go wrong after fast flying (up to 31°, now below 1°).
  - Position hold stayed off for ever after its safety check tripped. Now it tries again by itself.

  See [docs/10](docs/10_troubleshooting.md) for what to check if it ever happens again.
- **Position hold (GPS), ON at power-up.** Let go of the sticks and the drone brakes, stops and stays on its spot, fighting wind (8 m/s wind: it moves less than 0.7 m). Switch it on/off from the OLED, the simulator page or the keyboard (P).
- **Two stick styles:**
  - **SPEED** (like DJI, default): the stick sets the speed.
  - **ANGLE**: the stick sets the tilt, as in v1.
- **Speed modes ECO / NORMAL / SPORT.**

  | | ECO | NORMAL | SPORT |
  |---|---|---|---|
  | Top speed | 2 m/s | 5 m/s | 10 m/s |
  | Max tilt | 15° | 25° | 35° |
  | Climb | 1 m/s | 2 m/s | 4 m/s |
  | Descend | 0.7 m/s | 1 m/s | 2.5 m/s |
  | Turn rate | 45°/s | 90°/s | 150°/s |

- **Height up to 1000 m**, with an optional **height limit** (OFF by default; on the OLED: Flight → Alt limit / Max alt).
- **RC link test** in the simulator: "Cut RC link". After 2 s the failsafe lands the drone on the spot, or hovers if you choose HOLD.
- **Help on the OLED:**
  - The bottom line of every menu explains the selected item.
  - Hold **BACK** for a full help page.
  - **MENU → Help** has 15 topics.
- **New dashboard pages:**
  - **POSITION** page.
  - **MOTORS** page redrawn as the drone seen from above (FL FR / RL RR, spin direction, %). It was checked against what the simulator receives.
- **Gazebo made easy:**
  - Double-click launchers in [`gazebo\`](gazebo/).
  - A **software flight controller for Linux (SITL)** flies the real Gazebo without the board.

## Start here

| When | Do this | Guide |
|---|---|---|
| **No board** | Double-click **`START_SIMULATOR.bat`**, press **Fly the software FC**, fly with the keyboard (W A S D, Q E, R F). | [docs/04](docs/04_flying_controls.md) |
| **Board** | Double-click **`UPLOAD_FIRMWARE.bat`**, then in the simulator press **Connect Arduino R4**. | [docs/03](docs/03_upload_and_first_run.md) |
| **Real Gazebo** | Double-click the files in `gazebo\` (in number order). | [docs/08](docs/08_gazebo_wsl_setup.md) |
| **Event day** | Follow the checklist. | [docs/09](docs/09_event_day_checklist.md) |

## What is tested, and what is not yet

**Tested:**
- **Your real R4 with v3 and only the organisers' files, freshly unpacked from their zip:** their unmodified bridge, every key of their keyboard tool, their timed commands, land and take off again, restarts and wrong start order, and 10 minutes of random flying, then it holds within 0.02 m (docs/11, section 0).
- **On a real R4 Minima in the real Gazebo, through launcher 3 and the organisers' bridge: over 10 minutes of flying, then holds within 0.14 m,** and an RC-loss landing 0.00 m from the spot. 9 of my 11 checks are inside their limits; the other two are small misses, explained in docs/11 (0.30 m settling after a stop against under 0.3 m, and a brief 7.4° heading offset against under 5°). Control delay: typically 5 ms, worst 64 ms.
- **On a real R4 Minima in the real Gazebo through launcher 3:** commands from the page, take-off again and stick landings from 1.5, 7.8 and 22 m (docs/11).
- **On a real R4 Minima with the simulator's physics:** wind 8 m/s moves it 0.78 m, gyro noise 0.56 m, gyro drift 1.67 m. Control step 946 µs at most, 0 CRC errors.
- **Compiles** for the UNO R4 Minima with 0 warnings in our code: 114 KB flash (43 %), 16 KB RAM (48 %). Check it with `tools\check_firmware.bat`.
- **38/38 automated flight tests** in the browser replica. They run the same flight code, compiled to WebAssembly. They cover all v1 tests plus:
  - position hold in calm air and in 5 and 8 m/s wind
  - SPEED and ANGLE sticks
  - the speed modes
  - compass errors from −30° to +60° in wind, and −76° and 180° learned in flight
  - far from home and a full turn
  - take-off and landing in wind
  - RC loss with LAND / HOLD / link restored
  - GPS stuck, compass off, noisy and delayed GPS
  - smooth mode switching
  - height limit at 20 m and a climb to 1000 m
  - the OLED help and motor readings
  - a resting gamepad stick, and keyboard + gamepad together
  - 10-minute flights
  - slow USB and a busy page (the real firmware behind a simulated USB link)
  - position hold recovering after FAILED
  - gyro noise and drift switched on in flight
  - landing with the stick, a wobbling left stick, and what the page says about the command link
  - the organisers' keyboard tool, landing with only their tool, and a bridge too slow to fly on

  A 2-hour simulated flight also ran. See [docs/11](docs/11_test_report.md).
- **9/9 checks in the real Gazebo** (Ubuntu 22.04 in WSL, Gazebo Harmonic), flown by the same firmware code built for Linux:
  - take-off, then hover within 0.00 m
  - forward and let go: stops 2.3 m later, then holds within 0.1 m
  - height limit 20.05 m
  - RC loss: failsafe after 2.0 s, lands 0.00 m from where the link was lost

**Not tested by me (needs you):**
- The OLED and buttons on the real board (I can't see them). First check: the SYSTEM page, and that the OLED no longer resets with the Arduino IDE open.
- Your real gamepad (tested with a fake one).
- The page's **Connect Arduino R4** button (it needs your click in the browser; I used a test relay).
- The banner and keys in your real Edge window (I flew from the app's browser pane with the same page), and launcher 2's permission step (the board was already shared).

## Folder map

```
AeroFC_R4_v3/
  START_SIMULATOR.bat          simulator / ground station for practice at home (Edge, port 8767)
  UPLOAD_FIRMWARE.bat          compile and upload to the board, no Arduino IDE window needed
  gazebo/                      double-click launchers for Ubuntu + Gazebo (docs/08)
  firmware/AeroFC_R4_v3/       the Arduino sketch (open AeroFC_R4_v3.ino); no libraries to install
    src/core/                  flight code (EKF, position hold, controllers, protocol, menu + help); also runs in the simulator and SITL
    src/hal/                   Arduino-only parts: OLED driver, buttons, flash, reset cause / watchdog
  simulator/                   index.html (fly), tests.html (automated tests), fc.wasm
  tools/                       build_wasm.py, check_firmware.bat, run_tests_headless.bat, serve.py, upload_firmware.ps1
    gazebo_wsl/                Ubuntu setup, bridge_plus.py, run_hil.sh, run_sitl.sh, sitl_check.py, stop_all.sh
    sitl/                      aerofc_sitl (the firmware as a Linux program) and its build script
  docs/                        guides, numbered in reading order
```

## Docs

1. [What you need](docs/01_what_you_need.md)
2. [Wiring](docs/02_wiring.md)
3. [Upload and first run](docs/03_upload_and_first_run.md)
4. [Flying: keyboard, gamepad, position hold, speed modes, RC test](docs/04_flying_controls.md)
5. [OLED dashboard, menu and help](docs/05_oled_menu_map.md)
6. [How it works](docs/06_how_it_works.md)
7. [Tuning](docs/07_tuning.md)
8. [Gazebo on Windows: launchers and PowerShell commands](docs/08_gazebo_wsl_setup.md)
9. [Event-day checklist](docs/09_event_day_checklist.md)
10. [Troubleshooting](docs/10_troubleshooting.md)
11. [Test report](docs/11_test_report.md)
12. [Hand-over sheet for the organisers' laptop](docs/12_hand_over_sheet.md)
