# AeroFC-R4: flight controller for the Arduino UNO R4 Minima

This is a quadcopter flight controller for the **Aerovidya 2026** hackathon. The **Arduino UNO R4 Minima** is the real flight controller. The drone itself (body, motors, sensors) is simulated by the organisers' **UAV HIL simulator** (Gazebo). They talk over USB using the official `"HI"` packet protocol.

What it does:

- **EKF attitude estimation (AHRS):** roll, pitch and yaw.
  - Gyro bias is estimated.
  - The accelerometer corrects tilt.
  - The compass corrects heading.
  - GPS is used so that accelerating or braking doesn't fool the horizon.
- **Kalman altitude filter:** altitude and climb rate.
- **Control loops:**
  - Attitude: angle, then rate (PID).
  - Altitude: height, then climb rate, then thrust.
- **Flight modes:** heading hold, altitude hold, manual throttle, automatic take-off and automatic landing.
- **Safety:** failsafes (lost pilot, lost sensors, estimator fault), crash detection (on/off), arming checks and a height ceiling.
- **On the board:** a 0.96" OLED dashboard and menu with 4 buttons.
  - Live data, PID tuning, feature switches, calibration and failsafe settings.
  - Settings are saved in flash.
- **Live telemetry:** a STATUS packet carrying RC input, attitude, altitude and motors, used for real-time plots and CSV logs.
- **A Windows simulator and ground station.** It runs in your browser with nothing to install.
  - Fly the firmware without the board, or fly the real board in the loop.
  - Keyboard or gamepad.
  - 3D view, live plots, virtual OLED.
  - Automated flight tests.

## Start here

| When | Do this | Guide |
|---|---|---|
| **Today, no board** | Double-click **`START_SIMULATOR.bat`**, press **Fly the software FC**, and fly with the keyboard (W A S D, Q E, R F). | [docs/04_flying_controls.md](docs/04_flying_controls.md) |
| **Board arrives** | Wire the OLED and buttons, upload `firmware/AeroFC_R4/AeroFC_R4.ino`, then in the simulator press **Connect Arduino R4**. | [docs/02_wiring.md](docs/02_wiring.md), [docs/03_upload_and_first_run.md](docs/03_upload_and_first_run.md) |
| **Before the event** | Install Ubuntu 22.04 in WSL with Gazebo and test against the *real* official simulator. | [docs/08_gazebo_wsl_setup.md](docs/08_gazebo_wsl_setup.md) |
| **Event day** | Follow the checklist. | [docs/09_event_day_checklist.md](docs/09_event_day_checklist.md) |

New to all this? Read [docs/01_what_you_need.md](docs/01_what_you_need.md), then [docs/06_how_it_works.md](docs/06_how_it_works.md).

## What is tested, and what is not yet

**Tested:**
- The firmware compiles for the UNO R4 Minima with 0 warnings in our code: 92.5 KB flash (35 %) and 14 KB RAM (42 %). Run `tools\check_firmware.bat`.
- The **exact same flight code**, compiled to WebAssembly, passes **12/12 automated flight tests** against a replica of the Gazebo X3 drone. The tests cover:
  - the protocol, byte-identical to the official `hil_protocol.py`
  - the handshake
  - take-off and hover
  - the direction of every stick
  - step responses
  - climb
  - the stock `control_input.py` behaviour
  - landing
  - failsafes and crash detection
  - robustness: delay, 40 Hz sensors, ±30 % inertia, noise and bias, wind, the 800 rad/s world, slow motors
  - the menu and settings

  Results are in [docs/11_test_report.md](docs/11_test_report.md). Run them again with `tools\run_tests_headless.bat`, or open `simulator/tests.html`.

**Not yet tested:**
- The physical OLED, buttons, flash saving and USB link on a real board (it arrives tomorrow; see the first checks in docs/03).
- The real Gazebo, which needs the Ubuntu 22.04 setup from docs/08.

## Folder map

```
AeroFC_R4/
  START_SIMULATOR.bat          start the simulator / ground station (opens Edge)
  firmware/AeroFC_R4/          the Arduino sketch (open AeroFC_R4.ino); no libraries to install
    board_config.h             pins (buttons D2-D5, OLED on A4/A5)
    src/core/                  flight code (EKF, controllers, state machine, protocol, menu); also runs in the simulator
    src/hal/                   Arduino-only parts: OLED driver, buttons, flash storage
  simulator/                   browser simulator: index.html (fly), tests.html (automated tests), fc.wasm
  tools/                       build_wasm.py, check_firmware.bat, run_tests_headless.bat, golden_vectors.py
    gazebo_wsl/                Ubuntu setup script, bridge_plus.py (official bridge + live plots), relay.py, run_hil.sh
  tests/golden_vectors.json    packets made by the official hil_protocol.py, used to check our protocol code
  docs/                        guides, numbered in reading order
```

## Docs

1. [What you need](docs/01_what_you_need.md)
2. [Wiring](docs/02_wiring.md)
3. [Upload and first run](docs/03_upload_and_first_run.md)
4. [Flying: keyboard, gamepad, modes](docs/04_flying_controls.md)
5. [OLED menu map](docs/05_oled_menu_map.md)
6. [How it works (EKF, PID, mixer, protocol)](docs/06_how_it_works.md)
7. [Tuning](docs/07_tuning.md)
8. [Gazebo on Windows (WSL Ubuntu 22.04)](docs/08_gazebo_wsl_setup.md)
9. [Event-day checklist](docs/09_event_day_checklist.md)
10. [Troubleshooting](docs/10_troubleshooting.md)
11. [Test report](docs/11_test_report.md)
