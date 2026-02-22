#!/bin/bash
# PX4 HITL with Gazebo Harmonic
#
# Usage:
#   ./run_hitl.sh start [world]  - Launch everything (Gazebo + bridge + QGC)
#   ./run_hitl.sh gazebo [world] - Launch Gazebo only
#   ./run_hitl.sh bridge         - Launch bridge only
#   ./run_hitl.sh stop           - Kill all HITL processes
#
# Worlds: default, baylands

set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PX4_DIR="$HOME/DEV/PX4-Autopilot"
VENV_DIR="$SCRIPT_DIR/hitl_bridge_venv"

# Gazebo needs to find local models first (x500_hitl), then PX4 models
export GZ_SIM_RESOURCE_PATH="${SCRIPT_DIR}/models:${PX4_DIR}/Tools/simulation/gz/models:${PX4_DIR}/Tools/simulation/gz/worlds:${GZ_SIM_RESOURCE_PATH:-}"

# Server config env var (adds Contact, AirSpeed, etc. on top of world SDF plugins)
export GZ_SIM_SERVER_CONFIG_PATH="${PX4_DIR}/Tools/simulation/gz/server.config"

# Select world file
select_world() {
    case "${1:-default}" in
        baylands|bay) echo "$SCRIPT_DIR/hitl_baylands.sdf" ;;
        *)            echo "$SCRIPT_DIR/hitl_world.sdf" ;;
    esac
}

# Kill any stale Gazebo / bridge processes
kill_stale() {
    local killed=false
    if pgrep -f "gz sim" > /dev/null 2>&1; then
        pkill -9 -f "gz sim" 2>/dev/null || true
        killed=true
    fi
    if pgrep -f "hitl_bridge.py" > /dev/null 2>&1; then
        pkill -9 -f "hitl_bridge.py" 2>/dev/null || true
        killed=true
    fi
    if pgrep -f QGroundControl > /dev/null 2>&1; then
        pkill -9 -f QGroundControl 2>/dev/null || true
        killed=true
    fi
    if $killed; then
        echo "  Killed stale processes"
        sleep 2
    fi
}

case "${1:-}" in
  start|all)
    WORLD_FILE=$(select_world "${2:-}")
    WORLD_NAME=$(basename "$WORLD_FILE" .sdf)
    kill_stale
    echo "=== Starting HITL ==="
    echo "    World: $WORLD_NAME"
    echo ""

    # 1. Gazebo server (background)
    echo "  [1/3] Gazebo server ..."
    gz sim -s -r "$WORLD_FILE" 2>&1 | grep -v "gz_frame_id\|MotorFailurePlugin\|Utils.cc" &
    GZ_SERVER_PID=$!
    sleep 3

    # 2. Gazebo GUI (background)
    echo "  [2/3] Gazebo GUI ..."
    gz sim -g 2>&1 | grep -v "gz_frame_id\|MotorFailurePlugin\|Utils.cc" &
    GZ_GUI_PID=$!
    sleep 1

    # 3. Bridge grabs serial first, then QGC opens (UDP only)
    echo "  [3/3] Bridge + QGroundControl ..."
    echo ""
    source "$VENV_DIR/bin/activate"

    # Open QGC after a delay so bridge grabs serial port first
    (sleep 5 && open -a QGroundControl 2>/dev/null) &

    cleanup() {
        echo ""
        echo "Stopping all HITL processes ..."
        kill $GZ_SERVER_PID $GZ_GUI_PID 2>/dev/null || true
        pkill -f "gz sim" 2>/dev/null || true
        echo "Done."
    }
    trap cleanup EXIT

    python3 "$SCRIPT_DIR/hitl_bridge.py" "${@:3}"
    ;;
  gazebo|gz)
    WORLD_FILE=$(select_world "${2:-}")
    WORLD_NAME=$(basename "$WORLD_FILE" .sdf)
    kill_stale
    echo "=== Starting Gazebo Harmonic ==="
    echo "    World: $WORLD_NAME ($WORLD_FILE)"
    echo "    Starting server in background, then GUI ..."
    gz sim -s -r "$WORLD_FILE" 2>&1 | grep -v "gz_frame_id\|MotorFailurePlugin\|Utils.cc" &
    sleep 3
    gz sim -g 2>&1 | grep -v "gz_frame_id\|MotorFailurePlugin\|Utils.cc"
    ;;
  server)
    WORLD_FILE=$(select_world "${2:-}")
    kill_stale
    echo "=== Starting Gazebo server only (headless) ==="
    gz sim -s -r "$WORLD_FILE" 2>&1 | grep -v "gz_frame_id\|MotorFailurePlugin\|Utils.cc"
    ;;
  gui)
    echo "=== Starting Gazebo GUI (connect to running server) ==="
    gz sim -g
    ;;
  bridge)
    echo "=== Starting HITL Bridge ==="
    # Kill QGC first so bridge can grab serial port
    if pgrep -f QGroundControl > /dev/null 2>&1; then
        echo "  Killing QGroundControl (it grabs serial port) ..."
        pkill -9 -f QGroundControl 2>/dev/null || true
        sleep 2
    fi
    source "$VENV_DIR/bin/activate"
    python3 "$SCRIPT_DIR/hitl_bridge.py" "${@:2}"
    ;;
  stop)
    echo "=== Stopping all HITL processes ==="
    kill_stale
    echo "Done."
    ;;
  *)
    echo "Usage: $0 {start|gazebo|bridge|stop} [options]"
    echo ""
    echo "  start [world]   - Launch everything (Gazebo + bridge + QGC)"
    echo "  gazebo [world]  - Launch Gazebo only"
    echo "  bridge          - Launch the MAVLink HITL bridge"
    echo "  stop            - Kill all HITL processes"
    echo ""
    echo "Worlds:"
    echo "  default    - Flat ground plane (fast to load)"
    echo "  baylands   - Baylands Park with coast water"
    echo ""
    echo "Bridge options:"
    echo "  --port /dev/tty.usbmodem01  Serial port (default)"
    echo "  --baud 921600               Baud rate (default)"
    echo "  --world hitl                Gazebo world name (default)"
    echo "  --model x500                Model name (default)"
    ;;
esac
