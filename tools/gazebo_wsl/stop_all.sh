#!/usr/bin/env bash
# stop_all.sh - stops everything the Gazebo launchers may have started (Gazebo, bridges, SITL, socat, relay)
pkill -f "bridge_plus.py" 2>/dev/null || true
pkill -f "relay.py" 2>/dev/null || true
pkill -f "aerofc_sitl" 2>/dev/null || true
pkill -f "control_input.py" 2>/dev/null || true
pkill -f "run_sitl.sh" 2>/dev/null || true
pkill -f "run_hil.sh" 2>/dev/null || true
"$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/stop_hil.sh"
