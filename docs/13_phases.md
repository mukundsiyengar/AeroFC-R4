# 13. The development phases and their evidence

The project was built in these phases, each with a test that had to pass. The measurements are in [docs/11](11_test_report.md), in the group named here.

| Phase | What was done | How it was checked | Test group |
|---|---|---|---|
| 1. Hardware | Board, OLED and buttons wired and written up | The board's own start-up and display checks, by hand | [docs/02](02_wiring.md) |
| 2. Sensor acquisition | IMU, barometer, compass and GPS over the protocol, with counters for late and out-of-order data | Steady rate, time stamps always forward, steady readings at rest, damaged packets thrown away | Phases 2 to 5 |
| 3. AHRS and calibration | Attitude EKF, gyro-bias learning, heading, calibration menus | Estimate against Gazebo's true attitude during known moves | Phases 2 to 5 |
| 4. EKF | Height and position filters, GPS aiding, sensor health | Estimates against Gazebo's true height and position during known moves | Phases 2 to 5 |
| 5. Stabilisation and height hold | Cascaded control and mixer | Real forces and twists applied in Gazebo, corrected with no unstable behaviour | Phases 2 to 5 |
| 6. RC-loss detection | Silence, too few packets, invalid packets | Each one starts the flight home, and a reconnect doesn't take control | Return to home |
| 7. Return to home | Climb, turn, cruise, descend, hover at 2 m | From 30 m in 8 directions, judged by Gazebo's true position | Return to home |
| 8. Camera processing | ArUco marker found in a simulated down camera | The camera's position of the pad against where the pad really is, hovering and moving | The camera |
| 9. Visual landing | Lines up and lands on the marker, and what happens if it is lost | Landed distance from the pad's centre | The camera |
| 10. Ground station | Telemetry, fault panel, map, CSV flight log | Faults show on the page and are in the log | Faults |
| 11. Fault injection | Sensor failures, communication loss and navigation failures, on purpose | Each one gives the expected response | Faults |
| 12. Whole system | A 10-minute mixed flight, timing, motor outputs, safety checks again | Every trip arrives, no failsafe, delay and motor limits | The whole system |

**Not measured:** power use, which needs a USB meter.
