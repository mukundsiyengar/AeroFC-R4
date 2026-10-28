# 7. Tuning

The default gains were computed from the X3 model (mass, inertia, motor constant) and checked against:
- 30 ms of extra delay
- 40 Hz sensors
- ±30 % inertia
- noise, bias and wind

You should not *need* to tune for the official simulator. Tune only if the real Gazebo behaves differently from the replica.

**How:** fly the software FC or the board, watch the **Attitude** and **Altitude** plots (solid = estimate, dashed = truth), change values in the OLED menu (live), then **Storage → Save**.

| Symptom | Change |
|---|---|
| Fast shaking / oscillation in roll or pitch | lower **Roll P / Pitch P** by 20 % (rate loop) |
| Slow wobble after a stick release | lower **Angle P** (4.0 → 3.5) |
| Sluggish, drifts back after a gust | raise **Roll I / Pitch I** |
| Overshoot on stick steps | raise **Roll D / Pitch D** a little, or lower **Angle P** |
| Heading swings back after a turn | lower **Head P**, or raise **Yaw P** |
| Height bobs up and down | lower **Alt P** or **Climb P** |
| Slowly sinks or rises after take-off | it learns the hover thrust within ~2 s. If not, check **Hover** (0.66 for the X3) |
| Take-off too slow / fast | **TO speed** |
| 800 rad/s world (root `quadcopter.sdf`) | **Motors → Motor max = 0.80** |

## Default values

| Parameter | Default |
|---|---|
| Roll P / I / D | 0.055 / 0.10 / 0.0015 |
| Pitch P / I / D | 0.17 / 0.30 / 0.0045 (the X3's pitch inertia is twice its roll inertia) |
| Yaw P / I | 0.60 / 0.25 |
| D cut | 8 Hz |
| Angle P | 4.0 (max tilt 25°, max rate 150 °/s) |
| Heading P | 1.5 (max yaw rate 90 °/s) |
| Alt P | 1.2 (climb +1.0 / −0.7 m/s) |
| Climb P / I | 3.0 / 1.0 |
| Hover | 0.66 |

Measured with these defaults in the replica:
- roll step rise time 0.32 s, overshoot 3 %, settled in 0.46 s
- take-off to 1.5 m in 3.1 s, peak 1.61 m
- hover ±1 mm, 0.03°

Use one change at a time and keep notes. "Factory defaults" (hold SELECT in Storage) always brings you back.
