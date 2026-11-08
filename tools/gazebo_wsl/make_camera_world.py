#!/usr/bin/env python3
"""make_camera_world.py - builds the camera world in ~/hil_test_r4, from the copies of the organisers' files there:

  models/aerofc_x3_cam     their betaflight_x3_sensor drone, with a camera looking straight down instead of its other sensors
  models/landing_marker    a landing pad with an ArUco marker
  worlds/quadcopter_cam    their world with that drone and that pad

install_tools.sh runs it.  The organisers' own folder is never touched.
"""
import os
import pathlib
import re
import sys

HOME = pathlib.Path.home() / "hil_test_r4" / "gz_betaflight_bridge"
MODELS, WORLDS = HOME / "models", HOME / "worlds"
WIDTH, HEIGHT, RATE, HFOV = [int(v) for v in os.environ.get("AEROFC_CAMERA", "640,480,15,1047").split(",")]   # HFOV in mrad
MARKER_ID = 7
PAD_M, MARKER_M = 1.0, 0.8          # the pad, and the black square of the marker on it
PAD_EAST, PAD_NORTH = 6.0, 4.0

CAMERA = f"""      <sensor name="down_camera" type="camera">
        <!-- a camera looks along its +X: pitched 90 degrees it looks straight down, with the top of the picture towards the nose -->
        <pose>0 0 -0.01 0 1.57079632679 0</pose>
        <always_on>true</always_on>
        <update_rate>{RATE}</update_rate>
        <topic>/X3/down_camera/image</topic>
        <camera>
          <horizontal_fov>{HFOV / 1000.0}</horizontal_fov>
          <image><width>{WIDTH}</width><height>{HEIGHT}</height><format>R8G8B8</format></image>
          <clip><near>0.05</near><far>200</far></clip>
        </camera>
      </sensor>
"""


def drone():
    src = (MODELS / "betaflight_x3_sensor" / "model.sdf").read_text(encoding="utf-8")
    src = re.sub(r"\s*<sensor name=\"front_camera\".*?</sensor>", "", src, flags=re.S)
    src = re.sub(r"\s*<sensor name=\"front_rangefinder\".*?</sensor>", "", src, flags=re.S)
    src, n = re.subn(r"\s*<sensor name=\"down_rangefinder\".*?</sensor>\n", "\n" + CAMERA, src, flags=re.S)
    if n != 1:
        sys.exit("their camera drone has changed: down_rangefinder not found")
    src = src.replace('<model name="betaflight_x3_sensor">', '<model name="aerofc_x3_cam">')
    d = MODELS / "aerofc_x3_cam"
    d.mkdir(exist_ok=True)
    (d / "model.sdf").write_text(src, encoding="utf-8")
    (d / "model.config").write_text('<?xml version="1.0"?>\n<model><name>aerofc_x3_cam</name><version>1.0</version><sdf version="1.11">model.sdf</sdf></model>\n', encoding="utf-8")


def pad():
    import cv2
    import numpy as np
    side = 800
    marker = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), MARKER_ID, side)
    border = int(side * (PAD_M - MARKER_M) / MARKER_M / 2)
    image = np.pad(marker, border, constant_values=255)
    d = MODELS / "landing_marker"
    d.mkdir(exist_ok=True)
    cv2.imwrite(str(d / "marker.png"), image)
    (d / "model.config").write_text('<?xml version="1.0"?>\n<model><name>landing_marker</name><version>1.0</version><sdf version="1.11">model.sdf</sdf></model>\n', encoding="utf-8")
    (d / "model.sdf").write_text(f"""<?xml version="1.0"?>
<sdf version="1.11">
  <model name="landing_marker">
    <static>true</static>
    <link name="link">
      <pose>0 0 0.005 0 0 0</pose>
      <visual name="pad">
        <geometry><box><size>{PAD_M} {PAD_M} 0.01</size></box></geometry>
        <material>
          <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0 0 0 1</specular>
          <pbr><metal><albedo_map>model://landing_marker/marker.png</albedo_map><metalness>0</metalness><roughness>1</roughness></metal></pbr>
        </material>
      </visual>
    </link>
  </model>
</sdf>
""", encoding="utf-8")


def world():
    src = (WORLDS / "quadcopter.sdf").read_text(encoding="utf-8")
    new, n = re.subn(r"<include>\s*<uri>model://betaflight_x3</uri>.*?</include>",
                     f"""<include>
      <uri>model://aerofc_x3_cam</uri>
      <name>X3</name>
    </include>
    <include>
      <uri>model://landing_marker</uri>
      <name>landing_marker</name>
      <pose>{PAD_EAST} {PAD_NORTH} 0 0 0 0</pose>
    </include>""", src, flags=re.S)
    if n != 1:
        sys.exit("their world has changed: the drone's include was not found")
    (WORLDS / "quadcopter_cam.sdf").write_text(new, encoding="utf-8")


drone()
pad()
world()
print(f"camera world ready: {WIDTH} x {HEIGHT} at {RATE} a second, pad at {PAD_NORTH} m north, {PAD_EAST} m east")
