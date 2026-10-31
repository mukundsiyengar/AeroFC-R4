# 4. Flying: keyboard, gamepad, position hold, speed modes, RC test

Start the simulator with `START_SIMULATOR.bat`, then press **Fly the software FC** (or **Connect Arduino R4**). Click once on the page so it receives the keys.

## Keyboard (hold the keys; the "sticks" spring back when you let go)

| Key | Action |
|---|---|
| **W / S** | forward / back |
| **A / D** | left / right |
| **Q / E** or ← / → | turn left / right |
| **R / F** or ↑ / ↓ | climb / descend |
| hold **Shift** | full stick (otherwise the keys give 60 %) |
| **1 / 2 / 3** | speed mode **ECO / NORMAL / SPORT** (**G** = next mode) |
| **P** | position hold on / off |
| **K** | stick style SPEED / ANGLE |
| **Enter** | arm / disarm |
| **T** | take-off to the take-off height (1.5 m) |
| **L** | land (it disarms itself when it touches down) |
| **M** | manual throttle on/off (R/F then change the throttle) |
| **Space** | **KILL**: motors off immediately |

## Gamepad (Xbox layout; PlayStation names in brackets), "mode 2"

| Control | Action |
|---|---|
| Left stick up/down | climb / descend |
| Left stick left/right | turn |
| Right stick | forward / back / left / right |
| **A** (✕) | take-off |
| **B** (○) | land |
| **X** (□) | next speed mode |
| **RB** (R1) | position hold on / off |
| **Start** (Options) | arm / disarm |
| **Y** (△) | manual throttle on/off |
| hold **Back** (Share) 0.5 s | KILL |

Press any gamepad button once after connecting it; browsers only show gamepads after a button press.

**Good to know about the gamepad:**
- **Keyboard and gamepad work together.** A pressed key overrides the gamepad on that axis.
- **The sticks have a 25 % deadzone** (the value Microsoft recommends for Xbox sticks), because thumbsticks often rest off-centre after use. A stick reading anything below 0.25 sends 0, and the drone holds its position.
- **Card 3 (Pilot) shows the raw stick values** (`pad raw`). A worn stick that rests beyond the deadzone gets a red warning there: "Stick not centred".
- **If the page is hidden or minimised**, the sticks go to neutral and the drone holds its position.

## Position hold (ON at power-up)

The drone uses GPS + compass + IMU to know where it is and how fast it moves.

- **Let go of the sticks:**
  1. It **brakes** (state BRAKING).
  2. It stops (HOLDING) and **stays on that spot**.
  3. Wind pushes it, it leans into the wind and comes back. The POSITION page shows how hard it is pushing ("wind").
- **Stick styles** (Flight → Stick, or K):
  - **SPEED** (default, like a DJI drone): the stick sets the speed over the ground. In NORMAL, full stick is 5 m/s, even in wind.
  - **ANGLE**: the stick sets the tilt (like v1). Full stick is 25° in NORMAL.
- **Switch off** (Flight → Pos hold, P, or RB). The drone then holds height and heading only, and keeps drifting like a DJI drone in ATTI mode: the X3 has almost no air drag, so steer the other way to stop it.
- **Not ready?** The POSITION page and the bottom line tell why:
  - NO GPS
  - GPS SETTLING
  - NO COMPASS
  - FAILED (the safety check below)

  Until it is ready the drone flies like ANGLE mode with position hold off.
- **It works during** take-off, hover, landing and the failsafe landing, so landings are on the spot.
- **Safety check:** if the drone is far from its spot and keeps getting further for 3 s, position hold switches itself off (FAILED). After 5 s it tries again by itself: it brakes and holds where it stops. After 3 failures in a row it stays off until the next arm.
- **Compass error:** Gazebo's compass points 90.8° away from GPS north ("Mag decl" is set to 90.8 for that). If the compass is wrong anyway, the flight controller learns the error from GPS during the first move (tested from −76° to 180°).

## Speed modes (Flight → Speed, keys 1/2/3, gamepad X)

| | ECO | NORMAL (default) | SPORT |
|---|---|---|---|
| Top speed (SPEED sticks) | 2 m/s | 5 m/s | 10 m/s |
| Max tilt (ANGLE sticks) | 15° | 25° | 35° |
| Braking tilt (position hold) | 12° | 20° | 30° |
| Climb / descend | 1.0 / 0.7 m/s | 2.0 / 1.0 m/s | 4.0 / 2.5 m/s |
| Turn rate | 45°/s | 90°/s | 150°/s |

Switching in flight is smooth (tested: no jump).

## Landing with the stick

Let go of the right stick and hold the left stick (or **F**) down. The drone comes straight down by itself:
- **It slows down near the ground, whatever the stick says:** at most 0.6 m/s per metre of height, and 0.25 m/s for the last part.
- **The speed builds up gradually,** so the motors ease off instead of plunging.
- **When it is on the ground with the stick still down,** the motors stop after 0.8 s (reason: landed). Press **T** to fly again.
- **Let go of the stick before it touches,** and it stays in the air at that height.
- **On a gamepad,** pulling the left stick down also pushes it a little sideways. A sideways part under half of the up/down part is ignored, so the drone doesn't turn on its way down. A real diagonal still does both.

## Why the motors stay near 66 %

About **66 % holds this drone up at any height**, because its weight stays the same. The motors only change while the speed up or down is changing, then settle near 66 % again: about 72 % while it starts to climb, 61 % while it starts to descend, and they differ from each other when it tilts or turns.

To see your own input, use the card **Throttle and motors** on the page (and **PWR** on the OLED's MOTORS page):
- **Throttle stick:** what the flight controller really received, with a bar that grows from the centre.
- **Climb speed:** this is what the stick controls.
- **Power above hover:** the motor power compared with hover, for example +5 %.

Press **M** for manual throttle if you want the stick to set the motor % directly (then the drone doesn't hold its height).

## Height and the height limit

- Up to **1000 m** above the take-off point.
- **Height limit** (OLED: **MENU → Flight → Alt limit** ON, **Max alt** 2–1000 m), **OFF** by default.
  - With it ON the drone slows down before the limit and stops there even with full climb stick. The ALTITUDE page shows **AT CEIL**.
  - Lowering the limit in flight brings the drone down to it.
- Landing is faster above 20 m ("Land high", 2 m/s), then "Land spd" lower down.

## RC link test (failsafe)

In the simulator page, card **4. RC link test**:

1. Fly and hover (position hold on).
2. Press **Cut RC link**. The page stops sending pilot packets (a dead transmitter). The card counts down: "failsafe in 1.2 s".
3. After **Link t/o** (2 s) the flight controller starts the failsafe:
   - **LAND** (default): holds the position and lands slowly on the spot, then disarms. Reason: **pilot link**.
   - **HOLD**: hovers on the spot at the same height until the link comes back.
   - **NONE**: does nothing (not recommended).
4. **Restore RC link** gives you control back (HOLD case).

Choose the action in the card (software FC) or on the board: **MENU → Failsafe → RC lost**. The same button works in Gazebo mode (it stops sending to the bridge).

## What the flight controller does by itself

- **On START** (from the simulator or bridge), it arms and **takes off to 1.5 m**, then holds height, heading and position. Switch it off with MENU → Features → **Auto TO**.
- **Altitude hold:** stick centred holds the height.
- **Heading hold:** when you release the turn stick, it locks the heading where the turn stopped.
- **Landing:** descends, detects touch-down and disarms.
- **Failsafes:**

  | Situation | What happens |
  |---|---|
  | Pilot input stops for 2 s (only with a continuous pilot stream, like our page's) | RC lost action: LAND / HOLD / NONE |
  | Sensor data stops for 1.5 s | disarm |
  | Estimator fault | restart the estimator + landing |
  | **Crash** (tilted more than 70° for 0.2 s, or an impact while tilted) | motors off (switchable) |

## Why the organisers' smoke test "moved on its own"

The smoke test (`./start_hil.sh`) does **not** run your flight controller. It runs the organisers' **fake Arduino** (`fake_arduino_v1.py`), a simple demo:
- It takes off to about 1 m by itself.
- Its small roll/yaw corrections push the wrong way, so it slowly drifts and turns even when you touch nothing.

That is their demo code, not AeroFC. To steer it, use their keyboard tool **in a second window** (the launcher `gazebo\5_DEMO_organisers_fake_arduino.bat` opens it for you). Click into that window, then:

| Key | Action |
|---|---|
| w / s | pitch in steps of 0.1 (their "forward / back") |
| a / d | roll left / right (steps of 0.1) |
| q / e | turn left / right |
| u / j | throttle up / down (leaves altitude hold) |
| SPACE | neutral (altitude hold) |
| x | exit |

## Using the organisers' keyboard tool with AeroFC (`control_input.py --interactive`)

The firmware understands it exactly:
- throttle 0 = altitude hold
- `u` / `j` = manual throttle around 0.66
- `w a s d q e` = steps of 0.1 that **stay** until you press space (their help text says 0.2; the code sends 0.1)

Their tool labels `w` "forward" but sends *+pitch*. The official protocol (`HIL_PROTOCOL.md`) says *+pitch = backward*, and the firmware follows the protocol, so with their tool `w` flies backwards. If the organisers want `w` = forward, flip **MENU → Input → Inv pitch** (no upload needed).

## Sign convention (from HIL_PROTOCOL.md)

| Input | + means |
|---|---|
| roll | right side down (moves right) |
| pitch | nose up (moves back) |
| yaw | clockwise (turns right) |

The automated test "Directions" checks this on every build.
