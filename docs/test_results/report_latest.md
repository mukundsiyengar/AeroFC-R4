# Test report 2026-11-05 2230

**1 FAILED: 230 of 231 checks.**

## Keys: which windows count

| Check | Result | Measured |
|---|---|---|
| window rule | pass | all pass |

## Browser tests: the same flight code in a replica

| Check | Result | Measured |
|---|---|---|
| Protocol: CRC32 + framing match zlib / official bridge | pass |  |
| Handshake, motor packet per burst, heartbeat 5 Hz | pass |  |
| Auto take-off to 1.5 m after START, then hover | pass |  |
| Directions: roll + right, pitch - forward, yaw + clockwise (mixer sign test) | pass |  |
| Attitude step response + estimator accuracy | pass |  |
| Climb / descend stick in altitude hold | pass |  |
| Stock control_input.py semantics (throttle 0 = hold, 0.66 = manual, sticky +-0.1) | pass |  |
| Land command: descent, touch-down detection, auto-disarm | pass |  |
| Failsafes: FC silent -> bridge stop + HELLO recovery; STOP; KILL; pilot link loss | pass |  |
| Crash detection ON / OFF | pass |  |
| Robustness: latency, 40 Hz bursts, inertia +-30 %, noise + bias, wind, 800 rad/s world | pass |  |
| Settings, OLED menu and buttons (virtual board) | pass |  |
| Position hold: calm hover, then wind steps 5 and 8 m/s | pass |  |
| SPEED sticks: 5 m/s forward, release -> brakes and holds | pass |  |
| ANGLE sticks with position hold: tilt like v1, release -> brakes and holds | pass |  |
| Speed modes ECO / NORMAL / SPORT | pass |  |
| Compass error -30 .. +30 deg in wind, runaway guard at 60 deg | pass |  |
| Big compass error (-76 deg, 180 deg): learned on the first move, then holds | pass |  |
| Far from home, then a full 360 deg turn while holding | pass |  |
| Take-off and landing in 5 m/s wind stay on the spot | pass |  |
| RC link lost: LAND lands on the spot, HOLD hovers, link back = control back | pass |  |
| GPS stuck / compass off / noisy, delayed GPS | pass |  |
| Switching position hold / stick style in flight is smooth | pass |  |
| Height limit ON stops at 20 m; OFF climbs to 1000 m (telemetry v2) | pass |  |
| Gamepad: a resting stick holds, a real push moves, keys override, hidden page = neutral | pass |  |
| Long flight: 10 minutes of flying in wind and gusts, then it still holds its spot | pass |  |
| Long flight with 70 ms delay + jitter (no waiting, like a busy Gazebo bridge), then it still holds its spot | pass |  |
| Real board link: slow USB and a busy page don't shake the drone (the physics waits for the board) | pass |  |
| Position hold recovers: FAILED tries again by itself; gives up after 3 in a row | pass |  |
| Heading offset stays right after hard flying (and telemetry reports it) | pass |  |
| Gyro noise and drift switched on in flight (the simulator's sliders) | pass |  |
| OLED help texts and motor readings | pass |  |
| Landing with the stick: slows near the ground, comes straight down, motors stop on touch-down | pass |  |
| Left stick: pulling down with a wobbling thumb does not turn the drone | pass |  |
| Organisers' keyboard tool: W goes forward, typed commands and our page follow their protocol | pass |  |
| Landing and taking off again with only the organisers' tool (u, j, SPACE, codes 12 and 13) | pass |  |
| A bridge too slow to fly on: stays on the ground and says so; PLOTS page draws | pass |  |
| Display off: after the set time on the ground, never while armed; the first press only wakes; alarms wake it | pass |  |
| A restarted world after a turn: takes off again and position hold works (the compass fault itself needs the real board) | pass |  |
| With nobody listening the board stays quiet: nothing waits on the way for the next bridge | pass |  |
| Page says if commands really reach the flight controller; launchers open it in Gazebo mode | pass |  |
| Keys pressed in the Gazebo window work like keys on the page, with no click | pass |  |
| Return to home: RC lost from any direction, home at 2 m, stays there | pass |  |
| Return to home: climbs first if low, keeps its height if high, turns to face home | pass |  |
| RC-loss detection: silence, too few packets, invalid packets; a reconnect doesn't grab control | pass |  |
| Return to home: navigation fails on the way, it lands where it is | pass |  |
| Hovering at home: someone holding it stops the motors; wind doesn't | pass |  |
| Go to a marked position: reaches it within 0.2 m, also from the ground; a stick takes over | pass |  |
| Mission: flies the points in order, then returns home; RC lost on the way goes home at once | pass |  |
| A stalling simulator: repeated sensor readings don't spoil the estimates | pass |  |
| A stuck IMU stops the motors; a paused simulator doesn't | pass |  |
| The GPS jumps 50 m: the drone stays where it is, and still gets home | pass |  |
| A compass that sticks, or turns wrong and comes right again: no fly-away | pass |  |
| Telemetry and the OLED show what the drone is doing by itself | pass |  |
| Gestures: only a clear gesture held on purpose is a command, and none can stop the motors | pass |  |
| Landing on a marker: lines up, comes down, lands on it; marker lost or never seen | pass |  |
| Laptop cameras: the real models load and work on real pictures | pass |  |
| The map: clicks become metres from the take-off point; the fault panel says what is wrong | pass |  |

## Real Gazebo, no board: phases 2 to 5

| Check | Result | Measured |
|---|---|---|
| Phase 2: sensor rate steady near 50 a second | pass | lowest 48, highest 48 a second |
| Phase 2: time stamps always move forward | pass | 0 out of order or repeated since start-up |
| Phase 2: readings steady while hovering | pass | height moves 0.003 m, roll 0.00 deg (standard deviation) |
| Phase 2: no damaged packets to speak of | pass | 0 failed their CRC check |
| Phase 3: roll right: the estimate follows Gazebo's truth | pass | tilted to 18.8 deg, error 0.23 deg (root mean square), 0.28 deg before taking off the 5 ms the numbers take to get here |
| Phase 3: roll left: the estimate follows Gazebo's truth | pass | tilted to 19.2 deg, error 0.38 deg (root mean square), 0.39 deg before taking off the 5 ms the numbers take to get here |
| Phase 3: pitch forward: the estimate follows Gazebo's truth | pass | tilted to 18.9 deg, error 0.21 deg (root mean square), 0.26 deg before taking off the 5 ms the numbers take to get here |
| Phase 3: pitch back: the estimate follows Gazebo's truth | pass | tilted to 18.9 deg, error 0.31 deg (root mean square), 0.30 deg before taking off the 5 ms the numbers take to get here |
| Phase 3: turning: the heading follows Gazebo's truth | pass | error 0.87 deg over a 6 s turn |
| Phase 3: heading after the turn, held still | pass | error 0.38 deg |
| Phase 4: height estimate follows a climb and descent | pass | moved 8.7 m, error 0.050 m (root mean square) |
| Phase 4: position estimate follows a fast move | pass | error 0.073 m (root mean square) |
| Phase 4: sensor health stays good | pass | compass True, sensors True, position True |
| Phase 5: sideways push 4 N: corrected, no unstable behaviour | pass | pushed 2.07 m sideways and 0.01 m in height, tilted at most 19.2 deg, ended 0.28 m from the start |
| Phase 5: push the other way 4 N: corrected, no unstable behaviour | pass | pushed 2.01 m sideways and 0.01 m in height, tilted at most 19.4 deg, ended 0.38 m from the start |
| Phase 5: push down 4 N: corrected, no unstable behaviour | pass | pushed 0.27 m sideways and 0.54 m in height, tilted at most 0.0 deg, ended 0.27 m from the start |
| Phase 5: push up 3 N: corrected, no unstable behaviour | pass | pushed 0.03 m sideways and 0.38 m in height, tilted at most 0.0 deg, ended 0.03 m from the start |
| Phase 5: twist in roll 0.3 N m: corrected, no unstable behaviour | pass | pushed 0.24 m sideways and 0.03 m in height, tilted at most 4.3 deg, ended 0.00 m from the start |
| Phase 5: twist in yaw 0.3 N m: corrected, no unstable behaviour | pass | pushed 0.11 m sideways and 0.01 m in height, tilted at most 1.8 deg, ended 0.01 m from the start |

## Real Gazebo, no board: the red pole

| Check | Result | Measured |
|---|---|---|
| fly to the red pole in Gazebo | pass | stopped 0.048 m from it |

## Real Gazebo, no board: return to home, go-to, missions, RC loss, holding it

| Check | Result | Measured |
|---|---|---|
| their tool: --throttle 23 goes to 10 m north, 5 m east, 3 m up | pass | missed by 0.080 m, height 3.00 m |
| their tool: --throttle 22 returns home and hovers at 2 m | pass | 0.048 m from home, 0.044 m from 2 m |
| their tool: its silence doesn't count as a lost link | pass | False |
| RC lost from 8 directions, 30 m away: home within 0.5 m, 2 m within 0.1 m | pass | worst 0.040 m from home, 0.041 m from 2 m, lost after at most 2.05 s, slowest trip 21.5 s  |
| low: climbs to 5 m before moving | pass | moved 0.13 m sideways, height 4.80 m after the climb |
| high: stays at its height until it is over home | pass | lowest 11.98 m on the way |
| then comes down to 2 m over home | pass | 0.010 m from home, 0.061 m from 2 m |
| 3 packets a second count as a lost link | pass | RTH after 2.7 s |
| the rate comes back: it keeps going home | pass | RTH |
| packets that aren't numbers count as a lost link | pass | RTH after 2.0 s, 48 bad packets |
| a single command doesn't bring a lost link back | pass | link came back: False, RTH |
| 35 s at home with nobody holding it: keeps hovering | pass | AUTO |
| pulled down with 6 N: motors stop within 2 s | pass | 1.25 s, reason held |
| pushed up with 6 N: motors stop within 2 s | pass | 1.23 s, reason held |
| 20 random targets up to 50 m away: within 0.2 m of each | pass | worst 0.094 m, height 0.029 m |
| mission: visits the four points in order | pass | [0.03, 0.091, 0.12, 0.091] |
| mission: then goes home by itself | pass | RTH |
| mission: ends hovering at home | pass | 0.059 m from home, 0.043 m from 2 m |

## Real Gazebo, no board: phases 8 and 9: the camera, landing on the marker

| Check | Result | Measured |
|---|---|---|
| Phase 8: hovering over the pad at 3 m: the camera's position of the pad is right | pass | 59 pictures, worst 0.008 m off, height from the picture off by 0.074 m |
| Phase 8: hovering over the pad at 6 m: the camera's position of the pad is right | pass | 60 pictures, worst 0.012 m off, height from the picture off by 0.145 m |
| Phase 8: moving past the pad: the camera still has it right | pass | 7 pictures, worst 0.257 m off |
| Phase 8: with no marker in the picture, nothing is reported | pass | seen: False |
| Phase 9: from +1.0, +1.0 m, 5 m up: lands on the pad | pass | 0.001 m from the centre, landed |
| Phase 9: from -1.5, +0.5 m, 6 m up: lands on the pad | pass | 0.002 m from the centre, landed |
| Phase 9: from +0.5, -0.6 m, 3 m up: lands on the pad | pass | 0.002 m from the centre, landed |
| Phase 9: marker lost on the way down: holds its height, then lands normally | pass | waited 3.0 s, height moved 0.16 m |
| Phase 9: and it lands | pass | landed |
| Phase 9: no marker in view: the command is refused | pass | FLYING |

## Real Gazebo, no board: phase 11: faults put in on purpose

| Check | Result | Measured |
|---|---|---|
| damaged bytes on the line for 8 s: thrown away, flight not disturbed | pass | moved at most 0.008 m, 0 packets failed their check, state FLYING |
| barometer frozen for 15 s while hovering: holds its height | pass | moved at most 0.10 m |
| gyro drift 3 deg/s switched on in flight: recovers | pass | moved at most 0.92 m, state FLYING |
| RC lost during CLIMB: it carries on home | pass | 0.026 m from home, height 2.01 m |
| RC lost during TURN: it carries on home | pass | 0.024 m from home, height 2.01 m |
| RC lost during CRUISE: it carries on home | pass | 0.019 m from home, height 2.01 m |
| RC lost during DESCEND: it carries on home | pass | 0.018 m from home, height 2.01 m |
| RC lost during HOVER: it carries on home | pass | 0.012 m from home, height 2.01 m |
| GPS frozen during the cruise: stops navigating, says why | pass | after 0.54 s, reason no navigation |
| and lands where it is, upright | pass | 15.7 m from where it happened, reason no navigation |
| GPS jumps 50 m: the drone stays where it is | pass | moved at most 0.03 m, tilt 0.5 deg, state FLYING, 1 jump seen |
| GPS jumps back: it still stays where it is | pass | moved at most 0.03 m, tilt 0.5 deg, 2 jumps seen |
| GPS jumps 50 m on the way home: it still gets there | pass | 0.380 m from home, state AUTO, height 2.03 m |
| and stays there when the GPS jumps back | pass | 0.393 m from home, state AUTO |
| compass fault mag_freeze on the way home: gets home, or lands and says why | pass | 23.5 m from home when it happened, at most 23.5 m, 0.38 m at the end, top speed 3.2 m/s, state AUTO, reason -, compass used False |
| after compass fault mag_freeze: the next trip goes where it is told | pass | 0.45 m off, top speed 3.1 m/s, state FLYING, reason - |
| compass fault mag_zero on the way home: gets home, or lands and says why | pass | 23.4 m from home when it happened, at most 23.4 m, 0.38 m at the end, top speed 3.2 m/s, state AUTO, reason -, compass used False |
| after compass fault mag_zero: the next trip goes where it is told | **FAIL** | 0.52 m off, top speed 3.1 m/s, state FLYING, reason - |
| compass turning wrong, then right again: it never gets away | pass | 41.4 m from where the 40 m trip started, top speed 3.4 m/s, state FAILSAFE, reason no navigation, heading offset 107 deg |
| after a compass turning wrong: the next trip goes where it is told | pass | 0.17 m off, top speed 3.1 m/s, state FLYING, reason - |
| IMU stuck in flight (the world goes on): motors stop, says why | pass | after 2.31 s, reason sensors, height 3.93 m |
| Phase 10: the faults are in the flight log | pass | 10312 rows: reasons ['no navigation', 'pilot link', 'sensors'], 902 rows flying with no compass, 4 GPS jumps, 1396 rows with the RC link lost |

## Real Gazebo, no board: phase 12: the whole system, a long mixed flight

| Check | Result | Measured |
|---|---|---|
| Phase 12: the whole mixed flight, without one failure | pass | 10.0 minutes: 15 trips, 5 stick moves, 5 climbs and descents, 5 turns, 5 returns home, 5 lost links, 5 missions |
| Phase 12: no failsafe, no landing that nobody asked for | pass | 12195 samples in the air |
| Phase 12: every trip arrives within 0.5 m | pass | 15 trips, worst 0.165 m |
| Phase 12: every return home arrives within 0.5 m | pass | 10 returns, worst 0.051 m |
| Phase 12: never tilts or speeds out of control | pass | worst tilt 21.4 deg, top speed 3.5 m/s |
| Phase 12: motor outputs always valid | pass | 0 invalid of 12195, between 0.052 and 1.000 |
| Phase 12: no motor stuck at its limit for over 1 s | pass | longest 0.05 s |
| Phase 12: one control step takes under 10 ms | pass | typical 0 us, worst 0 us (sensors come every 20 ms) |
| Phase 12: sensor rate near 50 a second | pass | typical 48.0 a second, lowest 47 |
| Phase 12: pilot stream arrives | pass | typical 24.0 a second |
| Phase 12: control delay stays under the 80 ms limit | pass | 0 of 29878 motor packets over 80 ms, worst 28 ms, average 3.5 ms |
| Phase 12: delays of seconds are Ubuntu's clock moving, not the link | pass | 0 motor packets looked over 1 s late, and the clock moved 0 times |
| Phase 12 safety: LAND comes down and stops the motors | pass | after 7.1 s, reason landed |
| Phase 12 safety: KILL stops the motors at once | pass | after 0.06 s, reason KILL |
| Phase 12 safety: flies again after a KILL | pass | flying after 2.6 s |
| Phase 12 safety: a lost link still starts the flight home | pass | after 2.0 s |

## Real Gazebo, real R4 Minima: phases 2 to 5

| Check | Result | Measured |
|---|---|---|
| Phase 2: sensor rate steady near 50 a second | pass | lowest 43, highest 44 a second |
| Phase 2: time stamps always move forward | pass | 0 out of order or repeated since start-up |
| Phase 2: readings steady while hovering | pass | height moves 0.003 m, roll 0.01 deg (standard deviation) |
| Phase 2: no damaged packets to speak of | pass | 0 failed their CRC check |
| Phase 3: roll right: the estimate follows Gazebo's truth | pass | tilted to 19.0 deg, error 0.42 deg (root mean square), 0.41 deg before taking off the 10 ms the numbers take to get here |
| Phase 3: roll left: the estimate follows Gazebo's truth | pass | tilted to 18.6 deg, error 0.42 deg (root mean square), 0.52 deg before taking off the 10 ms the numbers take to get here |
| Phase 3: pitch forward: the estimate follows Gazebo's truth | pass | tilted to 18.5 deg, error 0.39 deg (root mean square), 0.43 deg before taking off the 10 ms the numbers take to get here |
| Phase 3: pitch back: the estimate follows Gazebo's truth | pass | tilted to 18.8 deg, error 0.44 deg (root mean square), 0.43 deg before taking off the 10 ms the numbers take to get here |
| Phase 3: turning: the heading follows Gazebo's truth | pass | error 1.02 deg over a 6 s turn |
| Phase 3: heading after the turn, held still | pass | error 0.45 deg |
| Phase 4: height estimate follows a climb and descent | pass | moved 8.6 m, error 0.063 m (root mean square) |
| Phase 4: position estimate follows a fast move | pass | error 0.111 m (root mean square) |
| Phase 4: sensor health stays good | pass | compass True, sensors True, position True |
| Phase 5: sideways push 4 N: corrected, no unstable behaviour | pass | pushed 1.96 m sideways and 0.01 m in height, tilted at most 19.3 deg, ended 0.32 m from the start |
| Phase 5: push the other way 4 N: corrected, no unstable behaviour | pass | pushed 1.99 m sideways and 0.01 m in height, tilted at most 19.3 deg, ended 0.40 m from the start |
| Phase 5: push down 4 N: corrected, no unstable behaviour | pass | pushed 0.27 m sideways and 0.53 m in height, tilted at most 0.0 deg, ended 0.27 m from the start |
| Phase 5: push up 3 N: corrected, no unstable behaviour | pass | pushed 0.03 m sideways and 0.39 m in height, tilted at most 0.0 deg, ended 0.03 m from the start |
| Phase 5: twist in roll 0.3 N m: corrected, no unstable behaviour | pass | pushed 0.23 m sideways and 0.03 m in height, tilted at most 4.2 deg, ended 0.00 m from the start |
| Phase 5: twist in yaw 0.3 N m: corrected, no unstable behaviour | pass | pushed 0.10 m sideways and 0.01 m in height, tilted at most 1.8 deg, ended 0.02 m from the start |

## Real Gazebo, real R4 Minima: the red pole

| Check | Result | Measured |
|---|---|---|
| fly to the red pole in Gazebo | pass | stopped 0.054 m from it |

## Real Gazebo, real R4 Minima: return to home, go-to, missions, RC loss, holding it

| Check | Result | Measured |
|---|---|---|
| their tool: --throttle 23 goes to 10 m north, 5 m east, 3 m up | pass | missed by 0.054 m, height 3.00 m |
| their tool: --throttle 22 returns home and hovers at 2 m | pass | 0.031 m from home, 0.041 m from 2 m |
| their tool: its silence doesn't count as a lost link | pass | False |
| RC lost from 8 directions, 30 m away: home within 0.5 m, 2 m within 0.1 m | pass | worst 0.030 m from home, 0.046 m from 2 m, lost after at most 2.05 s, slowest trip 25.2 s  |
| low: climbs to 5 m before moving | pass | moved 0.14 m sideways, height 4.80 m after the climb |
| high: stays at its height until it is over home | pass | lowest 11.98 m on the way |
| then comes down to 2 m over home | pass | 0.007 m from home, 0.061 m from 2 m |
| 3 packets a second count as a lost link | pass | RTH after 3.1 s |
| the rate comes back: it keeps going home | pass | RTH |
| packets that aren't numbers count as a lost link | pass | RTH after 2.0 s, 47 bad packets |
| a single command doesn't bring a lost link back | pass | link came back: False, RTH |
| 35 s at home with nobody holding it: keeps hovering | pass | AUTO |
| pulled down with 6 N: motors stop within 2 s | pass | 1.21 s, reason held |
| pushed up with 6 N: motors stop within 2 s | pass | 1.21 s, reason held |
| 20 random targets up to 50 m away: within 0.2 m of each | pass | worst 0.085 m, height 0.024 m |
| mission: visits the four points in order | pass | [0.014, 0.092, 0.113, 0.12] |
| mission: then goes home by itself | pass | RTH |
| mission: ends hovering at home | pass | 0.025 m from home, 0.052 m from 2 m |

## Real Gazebo, real R4 Minima: phases 8 and 9: the camera, landing on the marker

| Check | Result | Measured |
|---|---|---|
| Phase 8: hovering over the pad at 3 m: the camera's position of the pad is right | pass | 59 pictures, worst 0.006 m off, height from the picture off by 0.069 m |
| Phase 8: hovering over the pad at 6 m: the camera's position of the pad is right | pass | 60 pictures, worst 0.012 m off, height from the picture off by 0.134 m |
| Phase 8: moving past the pad: the camera still has it right | pass | 8 pictures, worst 0.220 m off |
| Phase 8: with no marker in the picture, nothing is reported | pass | seen: False |
| Phase 9: from +1.0, +1.0 m, 5 m up: lands on the pad | pass | 0.001 m from the centre, landed |
| Phase 9: from -1.5, +0.5 m, 6 m up: lands on the pad | pass | 0.001 m from the centre, landed |
| Phase 9: from +0.5, -0.6 m, 3 m up: lands on the pad | pass | 0.002 m from the centre, landed |
| Phase 9: marker lost on the way down: holds its height, then lands normally | pass | waited 3.0 s, height moved 0.14 m |
| Phase 9: and it lands | pass | landed |
| Phase 9: no marker in view: the command is refused | pass | FLYING |

## Real Gazebo, real R4 Minima: phase 11: faults put in on purpose

| Check | Result | Measured |
|---|---|---|
| damaged bytes on the line for 8 s: thrown away, flight not disturbed | pass | moved at most 0.008 m, 0 packets failed their check, state FLYING |
| barometer frozen for 15 s while hovering: holds its height | pass | moved at most 0.10 m |
| gyro drift 3 deg/s switched on in flight: recovers | pass | moved at most 0.92 m, state FLYING |
| RC lost during CLIMB: it carries on home | pass | 0.024 m from home, height 2.01 m |
| RC lost during TURN: it carries on home | pass | 0.021 m from home, height 2.01 m |
| RC lost during CRUISE: it carries on home | pass | 0.017 m from home, height 2.01 m |
| RC lost during DESCEND: it carries on home | pass | 0.022 m from home, height 2.01 m |
| RC lost during HOVER: it carries on home | pass | 0.011 m from home, height 2.01 m |
| GPS frozen during the cruise: stops navigating, says why | pass | after 0.46 s, reason no navigation |
| and lands where it is, upright | pass | 12.9 m from where it happened, reason no navigation |
| GPS jumps 50 m: the drone stays where it is | pass | moved at most 0.06 m, tilt 0.8 deg, state FLYING, 1 jump seen |
| GPS jumps back: it still stays where it is | pass | moved at most 0.06 m, tilt 0.8 deg, 2 jumps seen |
| GPS jumps 50 m on the way home: it still gets there | pass | 0.238 m from home, state AUTO, height 2.02 m |
| and stays there when the GPS jumps back | pass | 0.251 m from home, state AUTO |
| compass fault mag_freeze on the way home: gets home, or lands and says why | pass | 23.6 m from home when it happened, at most 23.6 m, 0.21 m at the end, top speed 3.0 m/s, state AUTO, reason -, compass used False |
| after compass fault mag_freeze: the next trip goes where it is told | pass | 0.32 m off, top speed 3.1 m/s, state FLYING, reason - |
| compass fault mag_zero on the way home: gets home, or lands and says why | pass | 23.6 m from home when it happened, at most 23.6 m, 0.24 m at the end, top speed 3.1 m/s, state AUTO, reason -, compass used False |
| after compass fault mag_zero: the next trip goes where it is told | pass | 0.32 m off, top speed 3.1 m/s, state FLYING, reason - |
| compass turning wrong, then right again: it never gets away | pass | 39.8 m from where the 40 m trip started, top speed 3.1 m/s, state FLYING, reason -, heading offset 98 deg |
| after a compass turning wrong: the next trip goes where it is told | pass | 0.30 m off, top speed 4.5 m/s, state FLYING, reason - |
| IMU stuck in flight (the world goes on): motors stop, says why | pass | after 1.55 s, reason sensors, height 3.69 m |
| Phase 10: the faults are in the flight log | pass | 10379 rows: reasons ['no navigation', 'pilot link', 'sensors'], 1174 rows flying with no compass, 4 GPS jumps, 1401 rows with the RC link lost |

## Real Gazebo, real R4 Minima: phase 12: the whole system, a long mixed flight

| Check | Result | Measured |
|---|---|---|
| Phase 12: the whole mixed flight, without one failure | pass | 10.0 minutes: 16 trips, 6 stick moves, 5 climbs and descents, 5 turns, 5 returns home, 5 lost links, 5 missions |
| Phase 12: no failsafe, no landing that nobody asked for | pass | 12139 samples in the air |
| Phase 12: every trip arrives within 0.5 m | pass | 16 trips, worst 0.226 m |
| Phase 12: every return home arrives within 0.5 m | pass | 10 returns, worst 0.040 m |
| Phase 12: never tilts or speeds out of control | pass | worst tilt 20.4 deg, top speed 3.4 m/s |
| Phase 12: motor outputs always valid | pass | 0 invalid of 12139, between 0.052 and 1.000 |
| Phase 12: no motor stuck at its limit for over 1 s | pass | longest 0.10 s |
| Phase 12: one control step takes under 10 ms | pass | typical 850 us, worst 1055 us (sensors come every 20 ms) |
| Phase 12: sensor rate near 50 a second | pass | typical 44 a second, lowest 43 |
| Phase 12: pilot stream arrives | pass | typical 24 a second |
| Phase 12: control delay stays under the 80 ms limit | pass | 0 of 27655 motor packets over 80 ms, worst 46 ms, average 5.1 ms |
| Phase 12: delays of seconds are Ubuntu's clock moving, not the link | pass | 0 motor packets looked over 1 s late, and the clock moved 0 times |
| Phase 12 safety: LAND comes down and stops the motors | pass | after 11.0 s, reason landed |
| Phase 12 safety: KILL stops the motors at once | pass | after 0.04 s, reason KILL |
| Phase 12 safety: flies again after a KILL | pass | flying after 2.6 s |
| Phase 12 safety: a lost link still starts the flight home | pass | after 2.0 s |
