#!/bin/bash
# HITL Diagnostic Script
# Checks all prerequisites before starting the HITL bridge

set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
VENV_DIR="$SCRIPT_DIR/hitl_bridge_venv"
PX4_DIR="$HOME/DEV/PX4-Autopilot"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

PASS=0
FAIL=0
WARN=0

pass() { echo -e "  ${GREEN}✓${NC} $1"; ((PASS++)); }
fail() { echo -e "  ${RED}✗${NC} $1"; ((FAIL++)); }
warn() { echo -e "  ${YELLOW}!${NC} $1"; ((WARN++)); }

echo "=============================="
echo " PX4 HITL Diagnostic"
echo "=============================="
echo ""

# --- 1. Pixhawk USB ---
echo "1. Pixhawk USB"
SERIAL_PORT=""
for dev in /dev/tty.usbmodem*; do
    if [ -e "$dev" ]; then
        SERIAL_PORT="$dev"
        break
    fi
done
if [ -n "$SERIAL_PORT" ]; then
    pass "Pixhawk found: $SERIAL_PORT"
else
    fail "No Pixhawk USB device (/dev/tty.usbmodem*)"
    echo "       -> Is the Pixhawk plugged in and powered?"
fi

# Check if something else is holding the port
if [ -n "$SERIAL_PORT" ]; then
    PORT_USER=$(lsof "$SERIAL_PORT" 2>/dev/null | tail -1 | awk '{print $1}')
    if [ -n "$PORT_USER" ]; then
        warn "Serial port in use by: $PORT_USER"
        echo "       -> Close $PORT_USER or it will conflict with the bridge"
    else
        pass "Serial port not held by another process"
    fi
fi

echo ""

# --- 2. Pixhawk MAVLink ---
echo "2. Pixhawk MAVLink"
if [ -n "$SERIAL_PORT" ] && [ -z "$PORT_USER" ]; then
    source "$VENV_DIR/bin/activate"
    MAVRESULT=$(python3 << PYEOF
import sys, json
sys.path.insert(0, "/usr/local/lib/python3.14/site-packages")
from pymavlink import mavutil

result = {"heartbeat": False, "hil_mode": False, "sys_hitl": None, "mav_type": None, "error": None}
try:
    mav = mavutil.mavlink_connection("$SERIAL_PORT", baud=921600)
    hb = mav.wait_heartbeat(timeout=5)
    if hb:
        result["heartbeat"] = True
        result["mav_type"] = hb.type
        result["hil_mode"] = bool(hb.base_mode & 0x20)
        mav.mav.param_request_read_send(mav.target_system, mav.target_component, b'SYS_HITL', -1)
        msg = mav.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
        if msg:
            result["sys_hitl"] = msg.param_value
    mav.close()
except Exception as e:
    result["error"] = str(e)

print(json.dumps(result))
PYEOF
    )
    if echo "$MAVRESULT" | python3 -c "import sys,json; r=json.load(sys.stdin); sys.exit(0 if r.get('heartbeat') else 1)" 2>/dev/null; then
        pass "Pixhawk heartbeat received"

        MAV_TYPE=$(echo "$MAVRESULT" | python3 -c "import sys,json; print(json.load(sys.stdin).get('mav_type','?'))")
        if [ "$MAV_TYPE" = "2" ]; then
            pass "Vehicle type: quadrotor (MAV_TYPE=2)"
        else
            warn "Vehicle type: MAV_TYPE=$MAV_TYPE (expected 2 for quadrotor)"
        fi

        HIL_MODE=$(echo "$MAVRESULT" | python3 -c "import sys,json; print(json.load(sys.stdin).get('hil_mode',False))")
        if [ "$HIL_MODE" = "True" ]; then
            pass "HIL flag active in base_mode"
        else
            fail "HIL flag NOT set in base_mode"
            echo "       -> PX4 is not in HITL mode"
        fi

        SYS_HITL=$(echo "$MAVRESULT" | python3 -c "import sys,json; v=json.load(sys.stdin).get('sys_hitl'); print(v if v is not None else 'unknown')")
        if [ "$SYS_HITL" = "1.0" ]; then
            pass "SYS_HITL = 1 (HITL enabled)"
        elif [ "$SYS_HITL" = "0.0" ]; then
            fail "SYS_HITL = 0 (HITL disabled)"
            echo "       -> In QGroundControl: Parameters > SYS_HITL = Enabled, then reboot"
        else
            warn "SYS_HITL = $SYS_HITL (could not confirm)"
            echo "       -> Check in QGroundControl: Parameters > SYS_HITL"
        fi
    else
        ERROR=$(echo "$MAVRESULT" | python3 -c "import sys,json; print(json.load(sys.stdin).get('error',''))" 2>/dev/null)
        if [ -n "$ERROR" ]; then
            fail "MAVLink error: $ERROR"
        else
            fail "No heartbeat from Pixhawk (5s timeout)"
            echo "       -> Is the Pixhawk booted? Try unplugging and replugging USB"
        fi
    fi
else
    if [ -z "$SERIAL_PORT" ]; then
        fail "Skipped (no USB device)"
    else
        warn "Skipped (port in use by $PORT_USER)"
    fi
fi

echo ""

# --- 3. Gazebo ---
echo "3. Gazebo Sim"
if command -v gz &>/dev/null; then
    GZ_VER=$(gz sim --version 2>/dev/null | head -1)
    pass "Gazebo installed: $GZ_VER"
else
    fail "gz command not found"
    echo "       -> Install Gazebo Harmonic: brew install gz-harmonic"
fi

# Check if gz-sim is running
GZ_PIDS=$(pgrep -f "gz sim" 2>/dev/null || true)
if [ -n "$GZ_PIDS" ]; then
    pass "Gazebo process running (PIDs: $(echo $GZ_PIDS | tr '\n' ' '))"

    # Check world name from topics
    TOPICS=$(gz topic -l 2>/dev/null || true)
    if echo "$TOPICS" | grep -q "/world/hitl/"; then
        pass "World 'hitl' is loaded"

        # Check for sensor topics
        if echo "$TOPICS" | grep -q "imu_sensor/imu"; then
            pass "IMU topic publishing"
        else
            fail "No IMU topic found"
            echo "       -> x500 model may not be loaded in the world"
        fi

        if echo "$TOPICS" | grep -q "navsat_sensor/navsat"; then
            pass "GPS (NavSat) topic publishing"
        else
            warn "No GPS topic found"
        fi

        if echo "$TOPICS" | grep -q "air_pressure_sensor/air_pressure"; then
            pass "Barometer topic publishing"
        else
            warn "No barometer topic found"
        fi

        if echo "$TOPICS" | grep -q "magnetometer_sensor/magnetometer"; then
            pass "Magnetometer topic publishing"
        else
            warn "No magnetometer topic found"
        fi
    elif echo "$TOPICS" | grep -q "/world/"; then
        WORLD_NAME=$(echo "$TOPICS" | grep -o '/world/[^/]*/' | head -1 | cut -d/ -f3)
        fail "Wrong world loaded: '$WORLD_NAME' (expected 'hitl')"
        echo "       -> Kill Gazebo: pkill -f 'gz sim'"
        echo "       -> Restart: ./run_hitl.sh gazebo"
    else
        warn "Gazebo running but no topics detected"
    fi
else
    warn "Gazebo not running"
    echo "       -> Start with: ./run_hitl.sh gazebo"
fi

echo ""

# --- 4. GZ Resource Path ---
echo "4. Model Resources"
X500_MODEL="$PX4_DIR/Tools/simulation/gz/models/x500/model.sdf"
if [ -f "$X500_MODEL" ]; then
    pass "x500 model found: $X500_MODEL"
else
    fail "x500 model not found at $X500_MODEL"
fi

X500_BASE="$PX4_DIR/Tools/simulation/gz/models/x500_base/model.sdf"
if [ -f "$X500_BASE" ]; then
    pass "x500_base model found"
else
    fail "x500_base model not found"
fi

if [ -f "$SCRIPT_DIR/hitl_world.sdf" ]; then
    pass "hitl_world.sdf found"
else
    fail "hitl_world.sdf missing from $SCRIPT_DIR"
fi

echo ""

# --- 5. Python Environment ---
echo "5. Python Environment"
if [ -f "$VENV_DIR/bin/python3" ]; then
    pass "venv exists: $VENV_DIR"
else
    fail "venv not found at $VENV_DIR"
    echo "       -> python3 -m venv $VENV_DIR && source $VENV_DIR/bin/activate && pip install pymavlink pyserial"
fi

source "$VENV_DIR/bin/activate" 2>/dev/null
if python3 -c "import pymavlink" 2>/dev/null; then
    pass "pymavlink importable"
else
    fail "pymavlink not installed in venv"
    echo "       -> source $VENV_DIR/bin/activate && pip install pymavlink"
fi

GZ_IMPORT=$(python3 -c "
import sys
sys.path.insert(0, '/usr/local/lib/python3.14/site-packages')
from gz.transport13 import Node
from gz.msgs10 import imu_pb2, magnetometer_pb2, fluid_pressure_pb2, navsat_pb2, actuators_pb2
print('ok')
" 2>&1)
if [ "$GZ_IMPORT" = "ok" ]; then
    pass "gz-transport + gz-msgs Python bindings OK"
else
    fail "gz Python bindings failed: $GZ_IMPORT"
    echo "       -> Ensure gz-harmonic Python bindings are at /usr/local/lib/python3.14/site-packages"
fi

echo ""

# --- Summary ---
echo "=============================="
echo -e " Results: ${GREEN}$PASS passed${NC}, ${RED}$FAIL failed${NC}, ${YELLOW}$WARN warnings${NC}"
echo "=============================="

if [ $FAIL -eq 0 ]; then
    echo ""
    echo -e "${GREEN}All checks passed! Ready to fly.${NC}"
    echo ""
    echo "  Terminal 1:  ./run_hitl.sh gazebo"
    echo "  Terminal 2:  ./run_hitl.sh bridge"
    echo "  Terminal 3:  Open QGroundControl"
else
    echo ""
    echo -e "${RED}Fix the failures above before starting HITL.${NC}"
fi
