# 11. Test report (v3)

Everything below flies the **unchanged flight code** in `firmware/AeroFC_R4_v2/src/core`. There are three levels, from fastest to most real.

## 0. With only the organisers' files (v3)

**How it runs:** your **real R4 with the v3 firmware**, and their `uav-hil-simulator-main.zip` freshly unpacked with none of our tools, using the commands in their README and manual. Their files are not edited. Truth comes from Gazebo's own odometry topic, and keys go through their real `control_input.py --interactive`, with Gazebo's window open. On this PC the board reaches Ubuntu through usbipd.

| Their command | Result on the real R4 |
|---|---|
| Bridge starts | Handshake, `Gazebo Transport: Native (High Perf)`, 42 motor updates a second |
| Take-off by itself | About 5 s after the bridge starts; hovers at 1.5 m, within 0.01 m sideways |
| `w` 3 times, 3 s | 3.64 m forward |
| `s` 3 times, 3 s | 3.79 m back |
| `a` / `d` 3 times, 3 s | 3.52 m left / 3.59 m right |
| `q` / `e` 3 times, 3 s | turns 71° left / 70° right, no height change |
| `u` 4 times, 2.5 s, then `SPACE` | climbs 8.1 m, then holds height |
| `j` 3 times, 2 s, then `SPACE` | descends 7.6 m, then holds height |
| `--roll 0.2 --duration 2.0` | 1.08 m right |
| `--pitch -0.2 --duration 2.0` | 0.94 m forward |
| `--yaw 0.3 --duration 2.0` | turns 41° right |
| `--throttle 0.70 --duration 2.0` | climbs 1.94 m |
| `--throttle 13 --duration 0.2` | lands in 10.4 s, moved 0.01 m, motors 0 |
| `--throttle 12 --duration 0.2` | back above 1.2 m in 1.4 s |
| `j` twice from 1.5 m | touches at about 1.5 m/s, level, motors stop 3.2 s later |
| `SPACE`, then `u` | takes off again, holds at 1.5 m |

Largest tilt in the whole flight: 11.4°.

**Restarts and wrong order, on the real R4:**

| What the operator does | Result |
|---|---|
| Ctrl+C on the bridge in flight | Their bridge stops the motors, so the drone falls to the ground |
| Start the bridge again, R4 still plugged in | Flies again 5.5 s later |
| Stop the bridge and restart it within 1 s | Stays in the air: rises briefly to 2.5 m, then back to 1.5 m |
| Stop Gazebo and the bridge, start both again | Flies again 4.9 s later |
| USB disconnected in flight (usbipd detach) | Their bridge stops with an error (`termios.error`), motors stop |
| Start the bridge again after that | Flies again 5.1 s later |
| Bridge started 25 s before Gazebo | Takes off 2.1 s after Gazebo comes up |

A physical unplug and replug wasn't tried: the board stayed powered during the USB test. The firmware's cold start was tested only by the restart after each upload.

**10-minute flight on the real R4 with their keyboard tool:** 233 random key presses, with the Gazebo window open. Largest tilt 18.7°. After `SPACE`, it held its spot within 0.02 m and its height within 0.001 m for 20 s.

**The same R4 through our test tool** (`bridge_plus` + `sitl_check.py`, no Gazebo window): 8 of 8 checks. Hover within 0.00 m; forward 2.99 m/s; let go and it brakes in 1.4 s, stops 2.3 m later and holds within 0.10 m; RC link cut, failsafe after 2.1 s and landing 0.00 m from the spot.

**Their bridge's slow mode** (no Gazebo Python bindings; forced without editing their file):
- Motors update about once a second. What the board sees is about 1 sensor burst a second, with short runs of 8–16.
- **Their own fake Arduino flew away to over 1000 m.** Our firmware did too, before this version.
- v3 on the real R4 stays disarmed, with the drone on the ground, for the whole 45 s run.
- A standard `gz-harmonic` install includes the bindings, so their laptop should use fast mode.

**Software flight controller:** the same flight with v3 built for Linux gave matching results, for example `w` 3.32 m forward and motors stopping 3.3 s after a `j` landing.

Sections A0–A3 below were measured on the real R4 with the last v2 firmware, whose flight code v3 keeps; v3 adds only the items in the README.

## A. Real UNO R4 Minima (your board)

### A0. Commands and stick landings: real R4 + real Gazebo, through the launchers

**How it runs:** `gazebo\3_FLY_ARDUINO_in_Gazebo.bat` with the Gazebo window open, the board attached with usbipd as in launcher 2 (it was already shared, so the permission step wasn't needed), and the simulator page in Gazebo mode sending real key events. A stand-in for Edge recorded the address the launcher asked for: `index.html?mode=gazebo`.

**What was wrong before:**
- The launchers handed `?mode=gazebo` to a batch file, which splits its arguments at `=`. The page opened as `index.html?mode` and flew its own built-in copy, so the R4 received no commands at all (`rc_thr = 0.0` in all three logs of that morning).
- Each command is a new connection, and `localhost` first tries IPv6, which Ubuntu's bridge doesn't listen on: 315 ms per command. `127.0.0.1` takes about 12 ms. In the software FC run, a key press now reaches the flight controller and comes back in telemetry in 56 ms.

| Command from the page | Real R4 in Gazebo |
|---|---|
| W, 3 s (0.6 stick) | 2.99 m/s, then brakes to HOLDING in 1.5 s, holds within 0.12 m |
| E, 2 s | turns from 90° to 192° |
| R, 3 s | climbs from 1.49 to 7.77 m |
| T | takes off again after every landing, about 3 s to 1.5 m |

**Landing with the stick held down** (all end with motors off, reason: landed):

| From | Time | Touch-down speed | Fastest | Sideways | Heading change | Largest tilt |
|---|---|---|---|---|---|---|
| 7.8 m, NORMAL | 11.2 s | 0.21 m/s | 1.13 m/s | 0.1 m | 0.0° | 0.1° |
| 1.5 m, NORMAL | 5.2 s | 0.28 m/s | 0.72 m/s | 0.0 m | 0.0° | 0.0° |
| 22 m, SPORT | 15.6 s | 0.18 m/s | 2.80 m/s | 0.0 m | 0.0° | 0.1° |
| 1.5 m, gamepad with a wobbling thumb (fake pad: sideways 0.4, down 1.0) | 5.5 s | 0.26 m/s | 0.75 m/s | 0.0 m | 0.0° | 0.0° |

Sideways comes from telemetry in 0.1 m steps; the distance from the hold point (0.01 m steps) stayed at or below 0.02 m. The commands and the first two landings are from the final firmware. The other flights in this section (SPORT, the fake gamepad, the L key and the RC cut) used the firmware just before the heading fix in A1, which doesn't touch the way down.

Before the change, in the browser replica: full stick from 1.5 m reached the ground in 1.3 s with a 0.12 m bounce, and the motors never stopped. After: 5.5 s, 0.03 m bounce, motors stop.

The L key from 1.5 m took 5.9 s (touch-down 0.26 m/s). Cut RC link: failsafe lands on the spot in 8.0 s, reason pilot link.

Throughout: control step at most 1014 µs, **0 CRC errors**. Through usbipd the board gets 37–44 sensor bursts per second instead of 50, with Gazebo at real time (real-time factor 1.0); the flight stays steady. The page's commands arrive at 17–25 per second with Gazebo's 3D window open on this PC. A real gamepad and the real Edge window are still to be checked by you.

### A1. Real R4 + real Gazebo + the organisers' bridge: 10-minute check, 9 of 11 inside my limits

**How it runs:** launcher 3's path (usbipd, Gazebo window open) with the final firmware. `sitl_check.py --long 10` flies it with pilot packets for over 10 minutes.

| Check | Limit | Result |
|---|---|---|
| Auto take-off to 1.5 m | | done |
| Position hold active after take-off | | HOLDING |
| Hover: stays on its spot | under 0.3 m | 0.01 m |
| Forward stick 0.6 (3 m/s) | moves | 2.99 m/s |
| Let go: brakes, switches to HOLDING | | 2.0 s, stops 3.3 m after release |
| **Then holds its spot** | under 0.3 m | **0.30 m: missed** |
| After 10 min of flying it still holds its spot | under 0.5 m | 0.14 m |
| **Heading offset during the long flight** | under 5° | **worst 7.4°: missed** |
| Still flying with position hold active | | yes |
| RC link cut: failsafe starts | | after 2.0 s |
| Failsafe lands on the spot, disarms (reason pilot link) | | 5.9 s, 0.00 m from where the link was lost |

**The two misses are small, and I left the limits as they were:**
- **0.30 m:** telemetry reads position in 0.1 m steps, and this is the drone still settling in the first seconds after a stop. After the 10-minute flight it held within 0.14 m.
- **7.4°:** the heading offset was above 5° for 24 s of the 11 minutes, and it corrected itself.

**Why this path is softer than my earlier test relay:** the same check through a relay with no Gazebo window scored 11/11 on the older firmware (worst offset 1.1°, stopped in 1.4 s). Through usbipd with the window open, the board gets 37–44 sensor bursts per second instead of 50. The software FC with no window still brakes in 1.3 s (section B).

**Safe margin measured:** control delay typically 5 ms, worst 64 ms, below the 80 ms limit (`bridge_plus` now prints this in the black window).

**A real fault this test found and fixed:** the first run scored 10/11, with the heading offset jumping 27° in one step (worst 21°). The IMU and GPS accelerations are a little apart in time, so while the acceleration turns, its angle looks wrong without any compass error. Such samples now count for nothing unless the angle is over 35°, and the fine filter doesn't learn during a fast turn. True compass errors still get corrected: 60° in wind 1.14 m, 76° and 180° learned (scenarios 17 and 18).

### A2. Real R4 + the simulator's physics, with wind and gyro faults

**How it runs:** the page's real-time physics behind a test relay to COM8, in place of the browser's serial permission click, with the page's wait-for-board limit on.

| Phase | Moved | Largest tilt | Notes |
|---|---|---|---|
| Hover, calm | 0.00 m | 0.0° | |
| Forward stick 3 s, then let go | stops 2.1 m later | | HOLDING |
| **Wind 8 m/s** | 0.78 m | 9.0° | comes back |
| **Gyro noise 2 °/s** | 0.56 m | 1.5° | |
| **Gyro drift 3 °/s, switched on in flight** | 1.67 m | 3.4° | attitude error peaks at 8.7°, then fully recovers |
| RC link cut | lands 0.18 m from the spot | | disarmed, reason pilot link |

Throughout: control step at most 946 µs, **0 CRC errors**, control delay at most 19 ms, simulation at 1.00× real time. (Measured earlier the same day, before the stick-landing and heading changes.)

### A3. Uploading to the real board

| Situation | Arduino IDE's route | `UPLOAD_FIRMWARE.bat` |
|---|---|---|
| Fresh boot, nobody listening for 20 s | works (failed before the fix) | |
| A program talked, then closed | works | |
| Idle for 60 s | works | |
| **After a flight (watchdog running)** | works (failed before the fix) | |
| Any of the above | | works |

## B. Real Gazebo, firmware built for Linux (no board): 9 / 9 checks

`run_sitl.sh` + `sitl_check.py` with the height limit ON at 20 m:
- take-off, then hover within 0.01 m
- forward and let go: brakes in 1.3 s, stops 2.3 m later, holds within 0.10 m
- **height limit 20 m with full climb stick: highest 20.05 m**
- RC loss from 19.9 m: failsafe after 2.0 s, lands in 36.9 s, 0.00 m from the spot

## C. Browser replica: 38 / 38 scenarios

**How it runs:** the flight code is compiled to WebAssembly and flies a replica of the Gazebo `betaflight_x3`, with Gazebo's real magnetometer field, behind a replica of `hil_bridge_v1.py`. Faster than real time. Run it with `tools\run_tests_headless.bat` or open `simulator/tests.html`.

| # | Scenario | Key results |
|---|---|---|
| 1 | Protocol: CRC32 + framing = official `hil_protocol.py` | byte-identical, resync after garbage |
| 2 | Handshake, motors per burst, heartbeat | 0.993 motor packets per burst, heartbeat gap 0.201 s |
| 3 | Auto take-off to 1.5 m, hover | 3.1 s, peak 1.62 m, hover ±1 mm |
| 4 | Stick directions | roll +0.4 → 9.6° right, pitch −0.4 → 9.5° nose down |
| 5 | Attitude step + estimator | rise 0.32 s, overshoot 2 %, EKF error RMS 0.14° |
| 6 | Climb / descend | ~2 m/s (NORMAL), drift after release −0.03 m |
| 7 | Organisers' `control_input.py` semantics | 0.66 = hover, sticky +0.1 = 2° |
| 8 | Land, touch-down, auto-disarm | 5.3 s from 1.5 m |
| 9 | Failsafes: FC silent, STOP, KILL, pilot link | all OK |
| 10 | Crash detection on / off | OK |
| 11 | Robustness: +30 ms, 40 Hz, inertia ±30 %, mass +15 %, noise + bias, wind, 800 rad/s, slow motors | all stable, max tilt 13.9° |
| 12 | Settings, OLED menu, buttons | menu edit, settings blob + CRC |
| 13 | Position hold: calm, wind 5 and 8 m/s | calm 0.00 m; 5 m/s peak 0.51 m; 8 m/s peak 0.42 m → 0.14 m |
| 14 | SPEED sticks: full stick, release | 4.89 m/s; stops in 1.7 s; then within 0.14 m |
| 15 | ANGLE sticks with position hold | 0.6 stick = 15.3°; holds within 0.31 m |
| 16 | Speed modes ECO / NORMAL / SPORT | 1.97 / 4.87 / 9.77 m/s; 14.8 / 24.6 / 34.5° |
| 17 | Compass error −30…+30° in wind; 60° | within 0.17 m; 60°: 1.14 m |
| 18 | Big compass error −76° and 180° | learned 76.1° / 179.9°, then holds within 0.11 m |
| 19 | 49 m from home + full 360° turn | max error 0.17 m |
| 20 | Take-off and landing in 5 m/s wind | take-off drift 0.73 m, lands 0.10 m from the hold point |
| 21 | RC link lost: LAND / HOLD / link back | failsafe after 1.99 s, lands 0.01 m from the spot; HOLD drift 0.03 m |
| 22 | GPS stuck / compass off / noisy GPS | falls back in 0.46 s; noisy 0.3 m: 0.61 m; + 150 ms delay: 0.95 m |
| 23 | Switching position hold / stick style in flight | largest set-point step 0.9° |
| 24 | Height limit 20 m; OFF → 1000 m | stops at 20.19 m; 1000 m reached, telemetry 1001.2 m |
| 25 | **Gamepad:** resting stick, real push, keys, hidden page | resting at 0.22: 0 m in 60 s; a real push flies; keys override the pad; hidden page = neutral |
| 26 | **10-minute flights** in wind and gusts (NORMAL, SPORT, noisy) | then holds within 0.25 m; heading offset at most 3.1° |
| 27 | **Long flight with 70 ms delay + jitter** | holds within 0.12 m |
| 28 | **Real board link** (real firmware behind a simulated USB link and page) | see below |
| 29 | **FAILED recovery** | wind 40 m/s trips it; tries again after 4.8 s; holds within 0.26 m; gives up after 3 in a row |
| 30 | **Heading offset after hard flying** | ANGLE + SPORT brake from 26 m/s: 0.13°; 15 min of SPORT: 0.5° |
| 31 | **Gyro noise and drift switched on in flight** | see below |
| 32 | OLED help + motor readings | 106 help texts fit the display; OLED motor % = simulator's |
| 33 | **Landing with the stick** | ECO, NORMAL and SPORT from 1.5 m, SPORT from 20 m, half stick, wind 3 m/s, position hold off: touches at 0.19–0.25 m/s, 0.17 m sideways at most (wind), bounce 0.03 m, lowest motor 60 %, motors stop; stick released at 0.6 m: keeps flying |
| 34 | **Left stick with a wobbling thumb** | heading 0.0°, 0.00 m sideways; a real diagonal still does both |
| 36 | **Organisers' keyboard tool** | `w` steps: 7.1° nose down, forward; `--pitch -0.2`: nose down too; `--pitch 0.2`: nose up; smooth joystick values and our page follow the protocol |
| 37 | **Landing and taking off with only their tool** | `j` to the ground: motors stop, reason landed; `u` flies again; after KILL, throttle does not restart |
| 38 | **A bridge too slow to fly on** | one burst every 1 s or 1.6 s: never arms, height 0 m after 70 s; 25 bursts a second flies; PLOTS page draws all 4 graphs |
| 35 | **Page says if commands reach the flight controller** | every banner message; launchers open Gazebo mode; `127.0.0.1`; throttle card from real telemetry; command count goes to 0 when commands stop |

**Scenario 28, control delay:** the drone is steady up to about 80 ms of delay. At 100 ms it wobbles (42° of tilt), and above that it crashes. The page therefore never lets the physics run more than 70 ms ahead of the board:

| USB delay, drawing time | Tilt | Hold | Simulation speed |
|---|---|---|---|
| 5 ms, 10 ms | 2.5° | 0.14 m | 1.00× |
| 5 ms, 60 ms | 2.6° | 0.09 m | 1.00× |
| 50 ms, 30 ms | 2.5° | 0.08 m | 1.00× |
| 100 ms, 10 ms | 2.6° | 0.12 m | 0.61× |
| 150 ms, 30 ms | 2.8° | 0.19 m | 0.41× |
| 300 ms, 30 ms | 3.2° | 0.30 m | 0.20× |

**Scenario 31, gyro faults switched on in flight (moved in 60 s):**

| Fault | Hold ON: before fix → now | Hold OFF: before fix → now |
|---|---|---|
| Noise 2 °/s | 0.4 → **0.2 m** | 9.8 → **2.8 m** |
| Drift 1 °/s | 1.3 → **0.5 m** | 209 → **30 m** |
| Drift 3 °/s | **crash, 708 m** → **1.6 m** | 541 → **90 m** |

With position hold OFF nothing brings the drone back, so it still travels while the filter learns the drift (about 10 s). Keep position hold ON.

**2-hour flight** (random flying, wind 3 m/s + gusts, sensor noise and gyro bias): holds within 0.36 m at every check, never FAILED, no worsening with time. Heading offset: below 1° after the first half hour, 4.3° at worst during it.

## D. Firmware build

`tools\check_firmware.bat`: compiles for the UNO R4 Minima with **0 warnings** in our code, about 114.4 KB flash (43 %) and 16 KB RAM (48 %).

## Not tested by me (needs you)

- **The OLED and buttons.** I can't see your display. The drawing code is tested on the virtual OLED, and the real board runs without errors. Please look at it: SYSTEM page, POSITION page, MOTORS page, hold BACK for help.
- **Your real gamepad.** Tested with a fake gamepad. If a stick rests beyond 0.25, card 3 shows a red warning.
- **The page's "Connect Arduino R4" button.** Web Serial needs your click in the browser's port chooser, so I used a test relay with the same page code path. Please try it once.
- **`gazebo\2_ARDUINO_to_Ubuntu.bat`'s permission step.** The board was already shared, so I ran only its attach command. Launcher 3 is tested (A0).
- **The banner and keys in your real Edge window,** in split screen. I flew from the app's browser pane with the same page. It's only for practice now, because the event doesn't use our page.
- **A physical unplug and replug,** and the organisers' actual laptop (there the board connects directly, without usbipd).
