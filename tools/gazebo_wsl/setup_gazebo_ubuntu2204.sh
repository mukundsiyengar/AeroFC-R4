#!/usr/bin/env bash
# setup_gazebo_ubuntu2204.sh - installs everything the official UAV HIL simulator needs on Ubuntu 22.04
# (inside WSL on Windows, or on a real Ubuntu PC) and copies the simulator to ~/hil_test_r4.
#
#   bash setup_gazebo_ubuntu2204.sh [path/to/uav-hil-simulator-main]
#
# It asks for your Ubuntu password (sudo) once.  Takes 10-20 minutes (about 1 GB download).
set -euo pipefail

HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SIM_SRC="${1:-$HERE/../../../uav-hil-simulator-main/uav-hil-simulator-main}"

. /etc/os-release
if [[ "${VERSION_ID}" != "22.04" && "${VERSION_ID}" != "24.04" ]]; then
  echo "This is Ubuntu ${VERSION_ID}. Gazebo Harmonic needs Ubuntu 22.04 (validated by the simulator authors) or 24.04."
  echo "In Windows PowerShell run:   wsl --install -d Ubuntu-22.04"
  exit 1
fi

echo "=== 1/4  basic tools"
sudo apt-get update
sudo apt-get install -y curl lsb-release gnupg socat python3-pip python3-serial dos2unix

echo "=== 2/4  Gazebo Harmonic + Python bindings (needed: without them the bridge publishes motors very slowly)"
sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install -y gz-harmonic python3-gz-transport13 python3-gz-msgs10

echo "=== 3/4  copy the simulator to ~/hil_test_r4"
if [[ ! -d "$SIM_SRC/arduino_hil" ]]; then
  echo "Simulator not found at $SIM_SRC - pass its folder as the first argument."; exit 1
fi
mkdir -p ~/hil_test_r4 ~/hil_test_r4/logs
cp -r "$SIM_SRC"/. ~/hil_test_r4/
cp "$HERE/bridge_plus.py" "$HERE/relay.py" "$HERE/run_hil.sh" "$HERE/run_sitl.sh" "$HERE/sitl_check.py"    "$HERE/stop_all.sh" "$HERE/../sitl/aerofc_sitl" ~/hil_test_r4/arduino_hil/
find ~/hil_test_r4 -name "*.sh" -o -name "*.py" | xargs dos2unix -q 2>/dev/null || true
chmod +x ~/hil_test_r4/*.sh ~/hil_test_r4/arduino_hil/*.sh 2>/dev/null || true
chmod +x ~/hil_test_r4/arduino_hil/run_hil.sh ~/hil_test_r4/arduino_hil/run_sitl.sh ~/hil_test_r4/arduino_hil/stop_all.sh   ~/hil_test_r4/arduino_hil/aerofc_sitl

echo "=== 4/4  serial port permission"
sudo usermod -a -G dialout "$USER" || true

echo
echo "Done.  Check:"
gz sim --versions || true
python3 -c "import gz.transport13, serial; print('python bindings OK, pyserial', serial.__version__)" || true
echo
echo "Close this window and open a new Ubuntu window (for the dialout group), then see docs/08_gazebo_wsl_setup.md"
