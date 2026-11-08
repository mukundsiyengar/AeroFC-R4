# 14. Flying by itself: return to home, go to a point, missions

This is new in version 4.

## Starting

Double-click `START_ALL.bat`. If the R4 Minima is plugged in, it flies on the board. If not, the same flight code flies as a program on the PC. `STOP_ALL.bat` stops everything and gives the board back to Windows.

The board needs this firmware first: `UPLOAD_FIRMWARE.bat`.

## Return to home

**It starts when:**
- the RC link is lost: the pilot's packets stop for 2 s (`Link t/o`), too few arrive (under 5 a second for 2 s), or they aren't valid numbers
- you press **H**, the **Return home** button on the page, **Menu > Flight > Return home** on the OLED, or send `--throttle 22` with the organisers' tool

**What it does:**

| Stage | What happens | Setting |
|---|---|---|
| CLIMB | Brakes, then climbs if it is lower than the RTH height | `RTH alt`, default 5 m |
| TURN | Turns its nose towards home | |
| CRUISE | Flies straight home | `Nav speed`, default 3 m/s |
| DESCEND | Comes down over home | `RTH hover`, default 2 m |
| HOVER | Stays there | |

If it is already higher than the RTH height, it stays at its height on the way home. Home is the place it was armed.

**It ends when:**
- **You move a stick** more than about a third of the way. The link coming back isn't enough, so a short reconnect can't grab control.
- You command a landing.
- **Someone holds it** while it hovers at home: see below.

**If it can't navigate:** with no GPS or compass, or if it isn't getting closer for 5 s, it lands where it is and says `no navigation`. If the RC link is lost and it can't navigate at all, it lands on the spot.

**If a sensor goes wrong on the way:**

| What goes wrong | What it does |
|---|---|
| GPS stops changing | Stops navigating within about 0.5 s and lands where it is. With no GPS it can't brake, so it coasts while it comes down |
| GPS jumps somewhere else | Ignores it for 1.5 s. If the GPS stays there, it treats that as a fixed error, so the drone stays where it really is and still finds home. The jump is counted in the page's Faults panel |
| Compass sticks | Notices once the gyro has turned 15 degrees with no change in the reading. It stops using the compass and flies on the gyro, and `CMP off` shows |
| Compass reads zero | The same: it flies on the gyro |
| Compass turns wrong slowly | Reads like a good compass, so it can't be told apart at first. If it stops getting closer for 5 s, it lands. On the ground it believes the compass again, once the compass has disagreed for 2 s |
| Pushing the wrong way | While it brakes or goes too fast, its speed can only fall. If its speed rises by over 1 m/s for 0.5 s, position hold stops pushing, levels the drone, and tries again 2 s later with what it has learned |

**Change what RC loss does:** `Menu > Failsafe > RC lost`: RTH, LAND, HOLD or NONE.

### The organisers' keyboard tool

Their tool only sends something when a key is pressed, so its silence can't mean a lost link. RC loss therefore works with our page or a gamepad, which send all the time. With their tool, return to home starts by command.

## Someone holding it

While hovering at home, a steady push or pull for 1 s stops the motors, and the display says `held`. It looks for 20 % more or less thrust than hovering takes, or 0.4 m away from its height. Turn it off with `Menu > Fly by itself > Hold det`.

**This is for the simulation only. Never catch a real drone with spinning propellers.**

## Go to a marked position

Three ways to mark the place:

| Way | How |
|---|---|
| The map on the page | Click the map, set the height, press **Go there** |
| In Gazebo itself | Click the red pole, press T, drag it, press Esc. Then press **Fly to Gazebo marker** on the page |
| The organisers' tool | `python3 arduino_hil/control_input.py --throttle 23 --roll 10 --pitch 5 --yaw 3 --duration 0.2` goes 10 m north, 5 m east, 3 m up |

Distances are metres from the take-off point. Height 0 means stay at this height. If it is on the ground, it takes off first. Moving a stick takes over at any moment.

**How exact it is:** in Gazebo it stops within a few centimetres, but only because Gazebo's GPS has no noise. A real drone is only as exact as its GPS: a few metres with a normal one, a few centimetres with RTK.

## Missions

1. Tick **Clicks add mission points** on the page, and set the height for each point before clicking.
2. Click the points in order: up to 16.
3. Press **Start mission**.

It flies them in order, then returns home by itself and hovers at 2 m. Losing the RC link during a mission sends it home at once. Moving a stick takes over.

## Keys

The same as in docs/04, plus **H** to return home. They work while the Gazebo window, its launcher window or the page is active. In the Gazebo window, KILL is **Backspace**, because Space pauses Gazebo.

## On the OLED

The STATUS page shows what it is doing, such as `RTH CRUISE`. `Menu > Fly by itself` has `RTH alt`, `RTH hover`, `Nav speed` and `Hold det`. `Menu > Help` has a topic called Return home.

## Testing

`RUN_ALL_TESTS.bat` runs every automatic test and writes a dated report in `docs/test_results`. You can use the PC meanwhile, unless you add `keys`.
