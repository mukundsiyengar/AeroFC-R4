# 15. Cameras

There are three different cameras, and each does a different job. All of it runs on the laptop, and the board only receives the results.

| Camera | Job | How to turn it on |
|---|---|---|
| Gazebo's simulated down camera | Finds the landing marker, for landing on it | `START_ALL camera` |
| The laptop's own camera | Gesture control | Tick **Gesture control** on the page |
| A USB webcam, such as a Logitech C270 | Object detection | Tick **Object detection** on the page |

Both tick boxes start off. Unticking turns that camera off completely. The first time, Edge asks if the page may use the camera: press **Allow**.

## Landing on a marker

`START_ALL camera` adds a camera looking straight down from the drone, and a landing pad 4 m north and 6 m east of the start. `tools/vision/marker_tracker.py` looks at each picture, finds the ArUco marker with OpenCV, and works out where it is compared with the drone. It uses only the picture, the height the marker's size shows, and the drone's tilt and heading, never Gazebo's true position.

1. Fly over the pad: click the map at about 4 m north, 6 m east, then **Go there**. At 3 m up, the pad must be within about 1 m of the drone to fit in the picture. Higher up is easier.
2. When **Pad camera** says `sees the marker`, press **Land on the landing pad**, or use `Menu > Flight > Land on marker` on the OLED.

It moves over the marker (`ALIGN`), comes down slowly while staying lined up (`DOWN`), and from 1 m up lands on the last place it saw the marker, because the camera can no longer see all of it. If it drifts more than 0.3 m off, it stops coming down until it is lined up again.

**If the marker is lost:** it stops coming down, waits 3 s, then lands normally where it is. If the camera can't see the marker when you ask, the command is refused.

## Gesture control

Hold the gesture towards the laptop's camera for 1 s:

| Gesture | What it does |
|---|---|
| Thumb up | Take off |
| Thumb down | Land |
| Open palm | Stop the trip, hold here |
| Victory (two fingers) | Return home |

A gesture only counts if the recogniser is at least 60 % sure and you hold it for 1 s. It does its job once, until you let go. **No gesture can stop the motors**, because a wrong guess must never drop the drone. The page shows what it sees and how far through the hold you are.

The camera needs the page to be visible, so keep it on screen while using gestures.

## Object detection

The page draws boxes and names on the webcam's picture: people, cars, bottles and so on, from a model trained on 80 common kinds of object. **It only shows them: it doesn't steer the drone.** It looks at about 5 pictures a second, to leave the PC free for the simulator.

If no webcam is plugged in, it uses the laptop's camera instead.

## Where the files come from

Gestures and object detection need files that are too big for this repository: run `python tools/get_camera_files.py` once (about 42 MB). `simulator/vendor/README.md` lists them, all Apache 2.0. Ubuntu needs OpenCV once for the camera world: `python3 -m pip install --user opencv-contrib-python-headless==5.0.0.93`. No password is needed. The zips in `..\new` were used only for reference and a few test pictures: they aren't needed to run anything.

## Not built yet

Tracking a chosen object, optical flow, visual odometry, and obstacle warnings. Gazebo's plain grey ground has nothing for optical flow or visual odometry to follow, so those also need a world with a textured ground.
