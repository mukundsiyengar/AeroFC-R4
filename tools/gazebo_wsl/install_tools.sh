#!/usr/bin/env bash
# install_tools.sh - copies the AeroFC-R4 Gazebo tools into ~/hil_test_r4/arduino_hil.
# Run it from this project's main folder (gazebo/1_UPDATE_tools_in_Ubuntu.bat does exactly that).
set -e
DST="$HOME/hil_test_r4/arduino_hil"
# It works on its own copy of the simulator: the organisers' folder is never changed.
if [[ ! -d "$DST" && -d "$HOME/hil_test/arduino_hil" ]]; then
  cp -a "$HOME/hil_test" "$HOME/hil_test_r4"
fi
if [[ ! -d "$DST" ]]; then
  echo "$DST not found: Gazebo is not set up in this Ubuntu yet."
  echo "Run once:   bash tools/gazebo_wsl/setup_gazebo_ubuntu2204.sh      (see docs/08)"
  exit 1
fi
cp tools/gazebo_wsl/{bridge_plus.py,relay.py,run_hil.sh,run_sitl.sh,sitl_check.py,stop_all.sh} tools/vision/marker_tracker.py tools/sitl/aerofc_sitl "$DST"/
cd "$DST"
sed -i 's/\r$//' bridge_plus.py relay.py run_hil.sh run_sitl.sh sitl_check.py stop_all.sh
chmod +x aerofc_sitl run_hil.sh run_sitl.sh sitl_check.py stop_all.sh
# That copy of the world gets the plugin that lets a test push the drone, as a hand would (never the organisers' file)
WORLD="$HOME/hil_test_r4/gz_betaflight_bridge/worlds/quadcopter.sdf"
grep -q apply-link-wrench "$WORLD" || sed -i 's|<light type="directional" name="sun">|<plugin filename="gz-sim-apply-link-wrench-system" name="gz::sim::systems::ApplyLinkWrench"></plugin>\n    &|' "$WORLD"
# ... and a red pole to mark a place with: drag it with Gazebo's move tool. It has no collision, so nothing can hit it.
grep -q target_marker "$WORLD" || sed -i 's|<light type="directional" name="sun">|<model name="target_marker"><static>true</static><pose>8 6 0 0 0 0</pose><link name="link"><pose>0 0 1 0 0 0</pose><visual name="pole"><geometry><cylinder><radius>0.06</radius><length>2</length></cylinder></geometry><material><ambient>0.9 0.1 0.1 1</ambient><diffuse>0.9 0.1 0.1 1</diffuse></material></visual></link></model>\n    &|' "$WORLD"
# That copy's start script can run another world: AEROFC_WORLD=... (the camera world below)
sed -i 's|^WORLD_FILE="${SCRIPT_DIR}/gz_betaflight_bridge/worlds/quadcopter.sdf"|WORLD_FILE="${AEROFC_WORLD:-${SCRIPT_DIR}/gz_betaflight_bridge/worlds/quadcopter.sdf}"|' "$HOME/hil_test_r4/start_gazebo.sh"
if python3 -c "import cv2" 2>/dev/null; then
  python3 "$OLDPWD/tools/gazebo_wsl/make_camera_world.py"
else
  echo "OpenCV isn't installed in Ubuntu: no camera world (python3 -m pip install --user opencv-contrib-python-headless==5.0.0.93)"
fi
mkdir -p "$HOME/hil_test_r4/logs"
echo "OK: the AeroFC-R4 tools are in $DST"
