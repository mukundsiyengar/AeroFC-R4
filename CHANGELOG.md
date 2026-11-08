# What changed in each version

Each version is a tag in this repository, so you can open or download exactly what it was: use the branch and tag menu at the top of the file list.

## v4.0: flies by itself, and survives broken sensors

**New**

- Return to home like a DJI drone, by command or when the RC link is lost, then a hover at 2 m. A short reconnect can't grab control: only a moved stick can.
- RC-loss detection covers silence, too few packets and invalid packets.
- Fly to a marked position, by clicking a map, dragging a marker in Gazebo or typing a command.
- Missions of up to 16 points, then home.
- Stops the motors if someone holds the drone at home (simulation only).
- Landing on an ArUco marker with a simulated down camera.
- Optional gestures and object detection on the laptop's cameras, in the page.
- The page's keys work with the Gazebo window active, so you don't have to click the page first.
- One-click start and stop (`START_ALL.bat`, `STOP_ALL.bat`), and every test with a dated report (`RUN_ALL_TESTS.bat`).
- The OLED turns itself off on the ground after a set time.
- Builds for the Nano R4 and UNO R4 WiFi too (never flown on them: [docs/16](docs/16_other_boards.md)).

**Stability fixes.** Each was first seen as a real failure, by breaking things on purpose:

- **The board hung when a bridge stopped while it was flying.** Up to v3 it kept sending for 2 s after the PC's last data, and the R4 core's USB driver waits for ever once its buffers are full and nobody reads them. In flight the watchdog then restarted the board and the USB connection dropped. On the bench, with the watchdog not yet running, the board stayed hung with no USB at all until RESET was double-pressed. It now sends only while the PC has the port open. The telemetry also reports why the board last started, which is how this was found.

- **A stuck compass could send the drone 400 m away.** The filter took real turns for gyro drift, then the learned heading offset went stale when the compass recovered. It now spots a stuck compass, flies on the gyro, and keeps the offset right when the compass comes back.
- **Speed that keeps rising while braking** now stops position hold, levels the drone and tries again 2 s later, whatever the cause.
- **A GPS that jumps 50 m** is no longer believed: the difference is taken off later fixes, so the hold point and home stay valid.
- **Evidence of a big heading error that stopped coming** used to switch off GPS help for as long as the drone hovered, and a small gyro drift then tilted the estimate by 19°. It is now forgotten after 0.5 s.
- **On the ground, a wrong speed estimate with an unchanging GPS** let the position run away by hundreds of metres, and the next arm recorded home at that wrong place. The GPS is now always used on the ground, and home is only recorded when the GPS and the estimate agree.
- **A repeated sensor reading from a stalling simulator** no longer spoils the estimates, and a really stuck IMU stops the motors.
- **After a landing with a dead GPS,** the GPS coming back no longer looks like a jump.
- **The state of the RC link is forgotten when a new bridge starts,** and a single command no longer counts as a working link. Before this, a command sent after a stream had ended could send the drone home 2 s later.
- **GPS positions use the WGS84 ellipsoid.** A round 111320 m per degree was 0.25 m off on a 40 m trip.

## v3.0: everything lives on the board

- Their keyboard tool works as labelled: `w` goes forward, while a typed `--pitch -0.2` also goes forward.
- Land and take off again with only their tool: `j` and `u`, or `--throttle 13` and `12`.
- The OLED's PLOTS page shows RC input, attitude and height, live on the board.
- Won't take off on the bridge's slow mode, where any flight controller flies away.

## v2.0: position hold, and working in the real Gazebo

- GPS and IMU position hold, which learns the wind. Speed modes ECO, NORMAL and SPORT.
- Heading learned from GPS when the compass is wrong. Gazebo's compass is 90.8° from GPS north.
- Gentle landing with the stick: slows near the ground, and the motors stop on touch-down.
- Fixed: commands never reached the drone in Gazebo, because the launchers opened the page without its Gazebo mode.
- Fixed: every command arrived about 320 ms late, because `localhost` tries IPv6 first.
- Fixed: the R4 restarted in a loop with the Arduino IDE open, because the watchdog ran all the time. It now starts at the first arm.
- Fixed: uploads failing with `LIBUSB_ERROR_TIMEOUT`, because data nobody read stayed stuck in the USB buffer.

## v1.0: first flight

- Attitude EKF with gyro-bias estimation, height filter, cascaded PID control and a motor mixer.
- The simulator's HIL protocol, with CRC32 checks.
- OLED menu and four buttons, with help texts and settings saved in flash.
- A browser replica of the drone, running the same flight code as the board, with automated flight tests.
