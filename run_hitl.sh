#!/bin/bash
# PX4 HITL with Gazebo Harmonic
#
# Usage:
#   Terminal 1:  ./run_hitl.sh gazebo
#   Terminal 2:  ./run_hitl.sh bridge
#   Terminal 3:  Open QGroundControl

set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PX4_DIR="$HOME/DEV/PX4-Autopilot"
VENV_DIR="$SCRIPT_DIR/hitl_bridge_venv"

# Gazebo needs to find PX4 models
export GZ_SIM_RESOURCE_PATH="${PX4_DIR}/Tools/simulation/gz/models:${PX4_DIR}/Tools/simulation/gz/worlds:${GZ_SIM_RESOURCE_PATH:-}"

# Server config env var (adds Contact, AirSpeed, etc. on top of world SDF plugins)
export GZ_SIM_SERVER_CONFIG_PATH="${PX4_DIR}/Tools/simulation/gz/server.config"

case "${1:-}" in
  gazebo|gz)
    echo "=== Starting Gazebo Harmonic with HITL world ==="
    echo "    Model path: $GZ_SIM_RESOURCE_PATH"
    echo "    Starting server in background, then GUI ..."
    gz sim -s -r "$SCRIPT_DIR/hitl_world.sdf" &
    sleep 3
    gz sim -g
    ;;
  server)
    echo "=== Starting Gazebo server only (headless) ==="
    gz sim -s -r "$SCRIPT_DIR/hitl_world.sdf"
    ;;
  gui)
    echo "=== Starting Gazebo GUI (connect to running server) ==="
    gz sim -g
    ;;
  bridge)
    echo "=== Starting HITL Bridge ==="
    source "$VENV_DIR/bin/activate"
    python3 "$SCRIPT_DIR/hitl_bridge.py" "${@:2}"
    ;;
  *)
    echo "Usage: $0 {gazebo|bridge} [bridge args...]"
    echo ""
    echo "  gazebo  - Launch Gazebo Harmonic with the HITL world"
    echo "  bridge  - Launch the MAVLink HITL bridge (connects Gazebo <-> Pixhawk)"
    echo ""
    echo "Bridge options:"
    echo "  --port /dev/tty.usbmodem01  Serial port (default)"
    echo "  --baud 921600               Baud rate (default)"
    echo "  --world hitl                Gazebo world name (default)"
    echo "  --model x500                Model name (default)"
    ;;
esac
