# 5. OLED dashboard, menu and help

The same menu runs on the real 0.96" OLED and on the **virtual OLED** in the simulator (software FC), so you can learn it there.

## Help on the OLED itself

- **Bottom line:** in every menu and list it scrolls a short explanation of the selected item (every setting, menu entry, calibration and storage action has one).
- **Hold BACK** on a menu line: a full-screen help page about it, word-wrapped. UP/DOWN scroll, BACK closes.
- **MENU → Help:** the topics: the buttons, each dashboard page, display off, bridge too slow, speed modes, position hold, return home, height & limit, failsafe, calibration, saving.

## Buttons

| Where | UP / DOWN | SELECT | BACK |
|---|---|---|---|
| Dashboard | previous / next page | open MENU (when armed on the ground: take off) | – |
| Dashboard, **hold** | – | **arm / disarm** (lands first if flying) | **KILL motors** |
| Menus | move | open | back |
| Menus, **hold** | – | – | **help page** for the selected line (while armed: **KILL**, on every screen) |
| Editing a value | change (hold = faster) | done | done |

## Dashboard pages (UP / DOWN)

1. **STATUS**: PC link (WAIT PC / HANDSHAKE / RUNNING / LOST), flight state and mode, speed mode, height, climb rate, roll/pitch, heading, compass, position hold, last disarm or failsafe reason. While it flies by itself, the state reads `AUTO` with what it is doing, such as `RTH CRUISE`.
2. **ATTITUDE**: artificial horizon, roll / pitch / heading, the angles the controller asks for (rSp, pSp), heading lock.
3. **ALTITUDE**: height, target, climb rate, barometer, a 13-second graph. **CEIL** shows the height limit when it is on, and **AT CEIL** when the drone is there.
4. **POSITION**: position-hold state (HOLD, BRAKE, MOVE, OFF, NOT READY + why), distance from the hold point, speed, position from home, learned wind push, stick style.
5. **MOTORS**: the drone **seen from above, front at the top**:
   ```
   FL cw      FR ccw         (example: hovering, all about 66 %)
     66%  ^FRONT 66%
   RL ccw     RR cw
     66%       66%
   ```
   - Each motor shows its spin direction and its command in %, with a bar.
   - **PWR** in the middle is the motor power above or below hover, for example `PWR+5%` while it starts to climb.
   - **hov** is the hover power the flight controller has learned, about 66 %. It is the same at any height, because the weight stays the same (docs/04).
   - The numbers are the commands the simulator receives (tested: equal to what the bridge gets, in the bridge's order FR, RL, FL, RR).
   - When the drone rolls right, the left motors (FL, RL) run faster. When it turns, the cw and ccw pairs differ.
6. **RC INPUT**: both sticks drawn, the raw values received, mode, packet rate, LINK LOST.
7. **SENSORS**: accelerometer, gyro, compass, barometer, the gyro bias the EKF learned, GPS position from home.
8. **SYSTEM**:
   - sensor and control rates, **step time in µs**, CRC errors, packets, uptime
   - **Reset:** the cause of the last restart: power-on, reset btn, upload/USB, software, **WDT HANG!**, low volt, other
   - **I2C err**: OLED transfer errors (the driver repairs them by itself)
   - **WDT on/off**: the watchdog starts at the first arm, then stays on
   - whether settings came from flash or defaults
9. **PLOTS** (press UP once from the first page): live graphs of the last 12 s, on the board itself.
   - **RP:** roll stick, with pitch stick dotted.
   - **YT:** yaw stick, with throttle stick dotted.
   - **AT:** roll, with pitch dotted; top and bottom are 40°.
   - **AL:** height, scaled to fit, with the height now in metres.

## MENU

| Item | Contents |
|---|---|
| **Flight** | Arm/Disarm (hold SELECT), Take off, Land, **Return home**, **Land on marker**, **Speed** (ECO/NORMAL/SPORT), **Pos hold** (on/off), **Stick** (SPEED/ANGLE), **Alt limit** (on/off), **Max alt** (2–1000 m), TO alt, KILL motors (hold) |
| **Calibrate** | **Gyro** (keep still 2 s), **Level** (keep level 2 s), **Compass** (start, turn 360° in yaw, then SELECT), **Baro zero**, Reset estimator. Gyro, level and baro need the drone disarmed; compass can run in flight. **Save afterwards.** |
| **Storage** | Save settings, Load saved, Factory defaults (hold SELECT). Saving is refused while armed. |
| **Help** | the help topics |
| Flight modes | Speed, Pos hold, Stick, Alt hold, Head hold |
| Rate PID | Roll P/I/D, Pitch P/I/D, Yaw P/I, D cut Hz, I limit |
| Angle/Yaw | Angle P, Max rate, Head P |
| Position PID | **Pos P** (distance → speed), **Vel P** (speed error → tilt), **Vel I** (learns the wind) |
| Altitude | Alt P, Climb P, Climb I, Hover, TO alt, TO speed, Land spd, Land low, Land high, Alt limit, Max alt |
| Features | Compass, Crash det, Auto TO (take off on START), Telem D1 (off by default), **Disp off** (Never, 15 s to 30 min; default 2 min) |
| Failsafe | **RC lost** (RTH / LAND / HOLD / NONE; default RTH), Link t/o (s), Crash deg, Idle disr(arm) |
| Fly by itself | **RTH alt** (default 5 m), **RTH hover** (default 2 m), **Nav speed** (default 3 m/s), **Hold det** (someone holding it stops the motors, default on) |
| Input | Inv roll, Inv pitch, Inv yaw, Expo, Deadband |
| Motors | Motor max (1.00 for the 1000 rad/s world, 0.80 for the 800 one), Idle |
| Estimator | Gyro nse, Acc nse, **Mag decl** (90.8 for Gazebo's compass), Baro nse, GPS aid |
| Loop rate | Loop: SYNC (one step per sensor packet, recommended), 100 Hz, 200 Hz; plus live IMU / control rate and step µs |
| Sensor prof | "Board sensitivity": Profile on/off, Gyro FS, Acc FS. Emulates a real 16-bit IMU. |
| About | Version (AeroFC-R4 4.0), parameter version, size of the saved settings |

Max tilt, yaw rate and climb/descent speeds are no longer separate settings: they come from the **speed mode** (docs/04).

## Display off

The OLED turns itself off a set time after the last button press, on the ground only (**MENU → Features → Disp off**).

- It never turns off while armed or calibrating, and the timer restarts on landing.
- Any button wakes it, and that press does nothing else. A long press while armed is still KILL.
- A crash, failsafe, lost link or too-slow bridge wakes it. It may then turn off again.
- The panel and its charge pump are really switched off (under 1 mA instead of about 20 mA).

## Storage details

- Settings and calibration are stored in the R4's data flash, in **two 1 KB slots with a CRC**. A power cut while saving can't destroy the last good copy.
- **Factory reset:** hold **BACK** while plugging in the board. Defaults are loaded but not saved until you choose Save.
