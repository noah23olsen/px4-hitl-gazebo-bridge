# PX4 HITL with Gazebo Harmonic — Setup Guide

Custom Hardware-in-the-Loop setup: real Pixhawk + Gazebo Harmonic (gz-sim 8) + QGroundControl, connected via a custom Python MAVLink bridge.

## Architecture

```
┌──────────┐  gz-transport   ┌──────────────┐  USB serial   ┌──────────┐
│  Gazebo   │ ──────────────> │              │ ────────────> │          │
│ Harmonic  │  IMU/Mag/Baro  │  hitl_bridge  │  HIL_SENSOR   │ Pixhawk  │
│  (gz-sim) │  GPS sensors   │    .py        │  HIL_GPS      │ (PX4)    │
│           │ <────────────── │              │ <──────────── │          │
│           │  motor_speed    │              │  HIL_ACTUATOR │          │
└──────────┘                 │              │               └──────────┘
                              │              │
                              │     UDP      │
                              │  forwarding  │
                              │              │
                              │   :14550     │
                              └──────┬───────┘
                                     │
                              ┌──────▼───────┐
                              │    QGC       │
                              │ (UDP 14550)  │
                              └──────────────┘
```

## Prerequisites

- macOS (Apple Silicon, using x86_64 Homebrew for Gazebo)
- Gazebo Harmonic: `brew install gz-harmonic` (via /usr/local Homebrew)
- PX4-Autopilot source: `~/DEV/PX4-Autopilot` (for x500 model files)
- Python 3.14 with gz-transport/gz-msgs bindings at `/usr/local/lib/python3.14/site-packages`
- QGroundControl
- Pixhawk flight controller with PX4 firmware

## Pixhawk Configuration (one-time, via QGroundControl)

1. Connect Pixhawk directly to QGC via USB
2. Set parameter `SYS_HITL = Enabled` (1)
3. Set parameter `COM_RC_IN_MODE = RC Transmitter only` (0) — if using RC controller
4. Reboot Pixhawk
5. QGC will warn "HITL enabled" on next connect — this is expected

## Python Environment Setup

```bash
cd ~/DEV/pixhawk
python3 -m venv hitl_bridge_venv
source hitl_bridge_venv/bin/activate
pip install pymavlink pyserial
```

## Files

- `hitl_world.sdf` — Gazebo world: ground plane, sun, GPS origin (Zurich), x500 with FPV camera
- `hitl_baylands.sdf` — Gazebo world: Baylands Park, California with coast water
- `models/x500_hitl/` — x500 quadcopter model with FPV camera for HITL
- `hitl_bridge.py` — Python bridge: Gazebo <-> Pixhawk MAVLink <-> QGC UDP
- `run_hitl.sh` — One-command launcher (kills stale processes, starts Gazebo + bridge + QGC)
- `px4_hitl.params` — PX4 parameter backup for HITL configuration
- `diagnose_hitl.sh` — Pre-flight diagnostic checker

## Running HITL

### One Command (recommended)
```bash
./run_hitl.sh start           # default flat world
./run_hitl.sh start baylands  # Baylands Park world
```
This kills stale processes, launches Gazebo, starts the bridge, and opens QGC.
Press Ctrl+C to stop everything.

### Separate Components
```bash
./run_hitl.sh gazebo [world]  # Gazebo only
./run_hitl.sh bridge          # Bridge only (kills QGC first to grab serial port)
./run_hitl.sh stop            # Kill all HITL processes
```

### Arming & Flying
- Wait ~15-20 seconds for PX4's EKF to converge
- Arm via QGC slider or RC sticks (throttle down + yaw right)
- Take off!

### FPV Camera
To view the onboard camera feed in Gazebo GUI:
1. Click the plugin menu (vertical dots, top right)
2. Search for **Image Display**
3. Select topic: `/world/hitl/model/x500/link/fpv_cam_link/sensor/fpv_cam/image`

## Using a Real RC Controller

1. Remove all propellers from the real drone
2. Plug in the drone battery (powers RC receiver via PDB)
3. Turn on RC transmitter
4. Verify sticks in QGC > Vehicle Setup > Radio
5. ESCs will beep (no PWM signal in HITL mode) — this is normal and harmless
6. To stop beeping: disconnect ESC signal wires from Pixhawk PWM rail

## Troubleshooting

### "Resource busy" on serial port
QGC grabbed the port. Close QGC or disconnect its serial link, then restart bridge.

### "No such file or directory" on serial port
Pixhawk disconnected. Unplug battery + USB, replug USB, wait a few seconds.

### Sensors missing / no data
- Is Gazebo running? Check: `gz topic -l | grep hitl`
- Wrong world? Kill all: `pkill -f "gz sim"` and restart
- Stale Gazebo from previous session? Kill and restart clean

### Attitude failure / EKF errors on startup
- Pixhawk failure detector latched. Unplug USB, replug, restart bridge
- Make sure Gazebo drone is upright before starting bridge
- Wait 15-20s for EKF to converge before arming

### Compass needs calibration
Known issue — magnetometer unit conversion between Gazebo Harmonic and PX4. Does not prevent flying in most cases.

### Bridge crashes with struct.error
GPS velocity overflow when drone tumbles. Fixed with value clamping in the bridge.

## Clean Restart Procedure

```bash
# 1. Stop everything
./run_hitl.sh stop
# 2. Reboot Pixhawk (unplug USB, replug)
# 3. Start everything
./run_hitl.sh start
# 4. Wait 15-20s for EKF, then fly
```
