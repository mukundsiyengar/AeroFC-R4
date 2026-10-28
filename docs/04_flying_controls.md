# 4. Flying: keyboard, gamepad and modes

Start the simulator with `START_SIMULATOR.bat`, then press **Fly the software FC** (or **Connect Arduino R4**). Click once on the page so it receives the keys.

## Keyboard (hold the keys, the "sticks" spring back when you let go)

| Key | Action |
|---|---|
| **W / S** | forward / back (nose down / up) |
| **A / D** | left / right (roll) |
| **Q / E** or ← / → | turn left / right (yaw) |
| **R / F** or ↑ / ↓ | climb / descend |
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
| **Start** (Options) | arm / disarm |
| **Y** (△) | manual throttle on/off |
| hold **Back** (Share) 0.5 s | KILL |

Press any gamepad button once after connecting it; browsers only show gamepads after a button press. The **Pilot** box shows "gamepad: …" when it is detected.

## What the flight controller does by itself

- **On START** (from the simulator), it arms and **takes off to 1.5 m**, then holds that height and heading. Switch it off with MENU → Features → **Auto TO**.
- **Altitude hold** (default): stick centred holds the height; climb/descend moves at up to 1 m/s up and 0.7 m/s down.
- **Heading hold:** when you release the turn stick, it locks the heading where the turn stopped.
- **Angle mode:** a full stick tilts 25°; centred = level.
- **Landing:** descends at 0.6 m/s, then 0.25 m/s below 1 m. It detects touch-down and disarms.
- **Failsafes:**

  | Situation | What happens |
  |---|---|
  | Pilot input stops for 2 s (only with our 25 Hz pilot stream) | automatic landing |
  | Sensor data stops for 1.5 s | disarm |
  | Estimator fault | restart the estimator + landing |
  | **Crash** (tilted more than 70° for 0.2 s, or an impact while tilted) | motors off (switchable) |

- **Ceiling:** 10 m (MENU → Altitude → Max alt).

## Why it drifts, and how to stop

The simulated X3 has almost no air drag. Once moving, it keeps sliding when you centre the stick, like a DJI drone in **ATTI mode**. To stop, give a short stick in the *opposite* direction.

## Using the organisers' keyboard tool (`control_input.py --interactive`)

The firmware understands it exactly:
- throttle 0 = altitude hold
- `u` / `j` = manual throttle around 0.66
- `w a s d q e` = ±0.1 steps that **stay** until you press space

Their tool labels `w` "forward" but sends *+pitch*. The official protocol (`HIL_PROTOCOL.md`) says *+pitch = backward*, and the firmware follows the protocol, so with their tool `w` flies backwards. If the organisers want `w` = forward, flip **MENU → Input → Inv pitch** (one button press, no upload needed).

## Sign convention (from HIL_PROTOCOL.md)

| Input | + means |
|---|---|
| roll | right side down (moves right) |
| pitch | nose up (moves back) |
| yaw | clockwise (turns right) |

The automated test "Directions" checks this on every build.
