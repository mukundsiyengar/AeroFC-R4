# 6. How it works (a beginner's tour of the flight code)

```
 simulator (Gazebo)          Arduino UNO R4  (firmware/AeroFC_R4/src/core)
 ───────────────────         ───────────────────────────────────────────────────────────────────────────
 IMU  BARO  MAG  GPS  ──USB──► hil_protocol ─► sensors ─► ahrs_ekf (roll, pitch, yaw)  ─┐
 every 20 ms                   (packets, CRC)  (FLU->FRD,   alt_ekf  (height, climb)    ├─► flight_core ─► controllers ─► mixer ─┐
                                                calibration, GPS aid (velocity, accel) ─┘   (state machine:   (angle->rate PID,   (4 motors) │
 pilot input ─────────USB──────────────────────────────────────────────────────────────►    arm, take-off,     height->climb->   │
                                                                                            land, failsafe)    thrust)           │
 4 motor commands ◄──USB── ACTUATOR_COMMAND ◄──────────────────────────────────────────────────────────────────────────────────┘
 + STATUS telemetry (plots) + HEARTBEAT 5 Hz
```

Everything in `src/core` is plain C++ with no Arduino functions. That's why the **same files** run on the R4, inside the browser simulator (compiled to WebAssembly) and in the automated tests.

## 1. Frames and signs (`fc_math.h`)

- **Gazebo** sends body data in **FLU**: x forward, y left, z up.
- Flight controllers usually use **FRD** (x forward, y right, z down) and **NED** for the earth (north, east, down).
- `sensors.cpp` converts once: **(x, −y, −z)**.
- After that:
  - roll + = right side down
  - pitch + = nose up
  - yaw + = turn right

  These are exactly the signs `HIL_PROTOCOL.md` uses for the pilot input.

## 2. Sensors, calibration and "board sensitivity" (`sensors.cpp`)

A real IMU outputs 16-bit numbers ("counts"). The sensitivity sets how many counts one unit is: at ±2000 °/s full scale, 1 count = 0.061 °/s.

With **Sensor prof = ON**, every value goes through this:
1. Converted to counts.
2. Clipped at full scale.
3. Rounded.
4. Converted back.

So the controller sees exactly what a real MPU-6500 would give. The raw counts are on the SENSORS page.

Then calibration is applied:
- gyro bias
- accelerometer "level" offset
- compass hard/soft iron (offset and scale)
- barometer zero

## 3. Attitude: the Extended Kalman Filter (`ahrs_ekf.cpp`)

The gyro measures rotation speed. Integrating it gives the angles, smooth and fast, but every small error adds up (drift). The accelerometer knows where "down" is, but it is noisy and fooled by acceleration. The compass knows north, but it is easily disturbed. A **Kalman filter** combines them, trusting each one according to how noisy it is.

- **State:** the attitude quaternion and the **gyro bias**. Its uncertainty is 6 numbers (3 angle errors + 3 bias errors) in a 6×6 covariance matrix P.
- **Predict** (every sensor packet): rotate the attitude by (gyro − bias)·dt. P grows by the gyro noise.
- **Accelerometer update:** "the measured force should point where the filter thinks *up* is". The difference corrects roll and pitch, and through P it also corrects the gyro bias.
  - On the ground it is trusted a lot.
  - In flight it is trusted little, because manoeuvres fake a tilt.
- **GPS aiding:**
  - Accelerating or braking makes the accelerometer read a fake tilt (≈ acceleration/g).
  - The firmware turns GPS positions into acceleration and passes its own IMU acceleration through *the same* filter chain, so both have the same delay.
  - The difference is the true slow attitude error.
  - A tiny 3-state Kalman filter also learns the angle between true north (GPS) and the compass, so the compass declination doesn't need to be known.
  - Result over a 40 s aggressive test: **0.95° RMS** attitude error with GPS aid, against **3.4° RMS** without.
- **Compass update:**
  1. The magnetic field is "levelled" with the current roll and pitch.
  2. The heading is computed.
  3. The correction is applied **only around the vertical axis**, so a disturbed compass can never tilt the horizon.

  Readings that disagree by more than 30°, or whose strength changes by more than ±30 %, are rejected. If the compass keeps disagreeing, it is switched off automatically and yaw comes from the gyro.
- Every measurement is fused as a scalar (one row at a time), so no matrix inversion is needed. One EKF step takes well under a millisecond on the R4's FPU.

## 4. Altitude (`alt_ekf.cpp`)

A 3-state Kalman filter: **[height, climb rate, accelerometer bias]**.
- Predicted with the vertical acceleration (the IMU rotated to the earth frame, minus gravity).
- Corrected by the barometer.

This gives a climb rate without the lag and noise of differentiating the barometer. Spikes more than 2 m and 5σ away are rejected.

## 5. Control (`controllers.cpp`, `pid.h`)

- **Angle loop:** angle error × Angle P = desired rotation rate (limited to 150 °/s). The Euler rates are converted exactly into body rates.
- **Rate loop (PID):** rate error → torque.
  - D acts on the measurement (no kick) and is low-pass filtered.
  - The integrator is frozen on the ground and doesn't wind up when the motors saturate.
  - The gains are in "normalized torque" units (fraction of one motor's thrust).
- **Heading hold:** after you release the yaw stick, it waits until the turn has stopped, then holds that heading.
- **Altitude:**
  1. Height error × Alt P = desired climb rate.
  2. Climb error × Climb P = desired acceleration.
  3. Thrust = learned hover thrust × (1 + acceleration/g) / cos(tilt).
  - The integrator **is** the learned hover thrust (shown as "Hover cmd").
  - Dividing by cos(tilt) keeps the height when tilting.
- **Take-off:** the height target rises at 0.8 m/s and brakes before 1.5 m (overshoot about 0.1 m).

## 6. Mixer (`controllers.cpp`)

The Gazebo X3 motor order: 0 front-right (CCW), 1 rear-left (CCW), 2 front-left (CW), 3 rear-right (CW).

| Motor | Thrust |
|---|---|
| FR | T − roll + pitch + yaw |
| RL | T + roll − pitch + yaw |
| FL | T + roll + pitch − yaw |
| RR | T − roll − pitch − yaw |

- **Thrust linearization:** propeller thrust grows with speed², so the command = motorMax·√thrust. The loops then behave the same at every throttle.
- **Saturation handling:** if a motor would go above 100 % or below idle:
  1. yaw is reduced first
  2. then the collective is shifted (up to ±0.15)
  3. then roll/pitch are scaled

  This way the drone always stays level.

> The organisers' `fake_arduino_v1.py` / `arduino_reference.ino` have the **roll and yaw signs reversed** for stabilisation (their gyro damping would push the wrong way). Our mixer was derived from the rotor positions in `model.sdf`, and a test checks every axis on every build.

## 7. State machine (`flight_core.cpp`)

```
DISARMED --arm (checks OK)--> IDLE --take-off / climb stick / START--> TAKEOFF --at height--> FLYING (ALT HOLD | MANUAL)
   ^                           |                                                                 |
   |                    15 s no take-off                                                         | land command
   |                           v                                                                 v
   +------ touch-down (auto-disarm) <--------------------------------------------------- LANDING / FAILSAFE
   +------ KILL / STOP / crash / sensors lost  (motors off immediately)
```

**Arming checks:**
- sensors fresh
- EKF ready (25 valid samples)
- tilt < 15°
- not calibrating
- at least 0.5 s since the last disarm

## 8. The protocol (`hil_protocol.cpp`, `fc_app.cpp`)

The official v1 protocol, byte-identical to `hil_protocol.py`; the test "Protocol" checks it against packets made by the official code.

- A corrupted byte loses at most one packet. The parser resyncs like the official bridge.
- Packets are written **whole or not at all**, only when the USB buffer has room, so the R4 can never hang on a full buffer.
- Priority: HELLO_ACK > motors > heartbeat > HELLO > STATUS.
- If the sensors stop for more than 1 s, the firmware sends **HELLO** once a second. The official bridge answers HELLO_ACK + START, so the session restarts after a board reset or a bridge failsafe.

**Our extension inside CONTROL_INPUT** (the bridge forwards the 4 numbers unchanged, so this works with the official bridge):

| Throttle field | Meaning |
|---|---|
| 0 | altitude hold (stock meaning) |
| 0 < t ≤ 1 | manual throttle (stock meaning; 0.66 ≈ hover) |
| 2.0 … 3.0 | altitude hold with climb stick: (t − 2.5) × 2 = −1 … +1 |
| 10 / 11 / 12 / 13 / 16 | ARM / DISARM / TAKEOFF / LAND / KILL (sent 3 times, acted on once) |

The pilot-link failsafe only becomes active after a continuous stream of at least 10 Hz in the extended format has been seen. The stock tool only sends on key presses, and its silence must not trigger a landing.

**STATUS telemetry** (type 0x25, 48 bytes, 20 Hz): the layout is in `src/core/telemetry.h`. The official bridge ignores it. Our simulator, `bridge_plus.py` and the D1 telemetry port decode it for the live plots.

## 9. Timing

- One control step per IMU packet: about 50 Hz, the rate the simulator sends.
- `dt` comes from the packets' own timestamps (clamped 2–100 ms). Gaps over 60 ms make the EKF less confident.
- The OLED is redrawn 5 times a second. Only changed pages are sent, in 1.5 ms chunks, and only when no USB data is waiting, so the display never delays a control step.
- A **watchdog** resets the board if the program ever hangs for 2 s. The OLED then shows "WATCHDOG RESET!".
