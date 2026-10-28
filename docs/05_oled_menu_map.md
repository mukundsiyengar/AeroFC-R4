# 5. OLED dashboard and menu

The same menu runs on the real OLED and on the **virtual OLED** in the simulator (software FC), so you can learn it today.

## Buttons

| Where | UP / DOWN | SELECT | BACK |
|---|---|---|---|
| Dashboard | previous / next page | open MENU (when armed on the ground: take off) | – |
| Dashboard, **hold** | – | **arm / disarm** (lands first if flying) | **KILL motors** |
| Menus | move | open | back |
| Editing a value | change (hold = faster) | done | done |

## Dashboard pages (UP / DOWN)

1. **STATUS**
   - PC link: WAIT PC / HANDSHAKE / RUNNING / LOST
   - state and mode
   - altitude and climb rate
   - roll and pitch
   - heading
   - compass ok/off
   - last disarm or failsafe reason
2. **ATTITUDE**: artificial horizon, roll / pitch / yaw, setpoints, heading lock.
3. **ALTITUDE**: altitude, target, climb rate, barometer, a 13-second graph.
4. **MOTORS**: four bars with % (FR, RL, FL, RR = the simulator's motor order).
5. **RC INPUT**: both sticks drawn, the raw values received, mode, link and packet rate.
6. **SENSORS**:
   - accelerometer, gyro (°/s) and raw counts
   - magnetometer (µT)
   - barometer
   - gyro bias the EKF has learned
   - GPS position from home
7. **SYSTEM**:
   - sensor rate and control rate
   - **step time in µs**
   - CRC errors
   - packets received and sent
   - uptime
   - whether settings came from flash or defaults

## MENU

| Item | Contents |
|---|---|
| **Flight** | Arm/Disarm (hold SELECT), Takeoff, Land, KILL (hold), Takeoff height |
| **Calibrate** | **Gyro** (keep still 2 s), **Level** (keep level 2 s), **Compass** (start, turn 360° in yaw, then SELECT), **Baro zero**, Reset estimator. Gyro, level and baro need the drone disarmed; compass can run in flight. **Save afterwards.** |
| **Storage** | Save settings, Load saved, Factory defaults (hold SELECT). Saving is refused while armed. |
| Rate PID | Roll P/I/D, Pitch P/I/D, Yaw P/I, D cut Hz, I limit |
| Angle/Yaw | Angle P, Max tilt, Max rate, Head(ing) P, Yaw rate |
| Altitude | Alt P, Climb P, Climb I, Max up, Max down, Hover, TO alt, TO speed, Land spd, Land low, Max alt |
| Features | Alt hold, Head hold, Compass, Crash det, Auto TO (take off on START), Telem D1 |
| Failsafe | FS action (LAND / HOLD / NONE), Link t/o (s), Crash deg, Idle disr(arm) |
| Input | Inv roll, Inv pitch, Inv yaw, Expo, Deadband |
| Motors | Motor max (1.00 for the 1000 rad/s world, 0.80 for the 800 one), Idle |
| Estimator | Gyro nse, Acc nse, Mag decl, Baro nse, GPS aid |
| Loop rate | Loop: SYNC (one step per sensor packet, recommended), 100 Hz, 200 Hz; plus live IMU / control rate and step µs |
| Sensor prof | "Board sensitivity": Profile on/off, Gyro FS (±250…2000 °/s), Acc FS (±2…16 g). Emulates a real 16-bit IMU: values are clipped at full scale and rounded to one LSB. |
| About | Version, parameter version, size of the saved settings |

## Storage details

- Settings and calibration are stored in the R4's data flash, in **two 1 KB slots with a CRC**. A power cut while saving can't destroy the last good copy.
- When the parameter layout changes in a new firmware version, old settings are ignored and defaults are used (the SYSTEM page says "dflt").
- **Factory reset:** hold **BACK** while plugging in the board. Defaults are loaded but not saved until you choose Save.
