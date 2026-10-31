#!/usr/bin/env bash
# install_tools.sh - copies the AeroFC v2 Gazebo tools into ~/hil_test/arduino_hil.
# Run it from the AeroFC_R4_v2 folder (gazebo/1_UPDATE_tools_in_Ubuntu.bat does exactly that).
set -e
DST="$HOME/hil_test/arduino_hil"
if [[ ! -d "$DST" ]]; then
  echo "$DST not found: Gazebo is not set up in this Ubuntu yet."
  echo "Run once:   bash tools/gazebo_wsl/setup_gazebo_ubuntu2204.sh      (see docs/08)"
  exit 1
fi
cp tools/gazebo_wsl/{bridge_plus.py,relay.py,run_hil.sh,run_sitl.sh,sitl_check.py,stop_all.sh} tools/sitl/aerofc_sitl "$DST"/
cd "$DST"
sed -i 's/\r$//' bridge_plus.py relay.py run_hil.sh run_sitl.sh sitl_check.py stop_all.sh
chmod +x aerofc_sitl run_hil.sh run_sitl.sh sitl_check.py stop_all.sh
mkdir -p "$HOME/hil_test/logs"
echo "OK: the AeroFC v2 tools are in $DST"
