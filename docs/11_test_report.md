# 11. Test report

Everything flown is judged by Gazebo's own true position, not by what the flight controller believes. These are the raw results from `RUN_ALL_TESTS.bat`, kept in [docs/test_results](test_results).

## Summary

| Test group | On the real R4 Minima | The same flight code on the PC, no board |
|---|---|---|
| Phases 2 to 5: sensors, estimates, stabilisation | 19 of 19 | 19 of 19 |
| Flying to a marked pole | 1 of 1 | 1 of 1 |
| Return to home, go-to, missions, RC loss, holding it | 18 of 18 | 18 of 18 |
| The camera: phases 8 and 9 | 10 of 10 | 10 of 10 |
| Faults: phases 10 and 11 | 22 of 22 | 21 of 22 |
| The whole system: phase 12 | 16 of 16 | 16 of 16 |
| Browser replica: the same flight code, faster than real time | | 58 of 58 |

## Every check, on the real R4 Minima

### Phases 2 to 5: sensors, estimates, stabilisation

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

### Flying to a marked pole

| Check | Result | Measured |
|---|---|---|
| Fly to the red pole in Gazebo | pass | stopped 0.054 m from it |

### Return to home, go-to, missions, RC loss, holding it

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

### The camera: phases 8 and 9

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

### Faults: phases 10 and 11

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

### The whole system: phase 12

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

## Not tested

- **A real airframe.** The drone is simulated throughout, and Gazebo's GPS has no noise, so a real drone would be far less exact.
- **A real gamepad.** Only the page's own tests cover it.
- **A laptop running Ubuntu directly.** All of this ran on Ubuntu inside Windows, reaching the board through usbipd.
- **Other boards.** The Nano R4 and UNO R4 WiFi only build: see [docs/16](16_other_boards.md).
- **Power use,** which needs a USB meter.
