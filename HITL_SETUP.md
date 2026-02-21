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

- `hitl_world.sdf` — Gazebo world: ground plane, sun, GPS origin (Zurich), x500 quadcopter with IMU/mag/baro/GPS sensors
- `hitl_bridge.py` — Python bridge: Gazebo <-> Pixhawk MAVLink <-> QGC UDP
- `run_hitl.sh` — Launcher script
- `diagnose_hitl.sh` — Pre-flight diagnostic checker

## Running HITL

### Terminal 1 — Gazebo
```bash
./run_hitl.sh gazebo
```
Wait for the GUI to appear with the x500 drone visible and upright.

### Terminal 2 — Bridge
```bash
./run_hitl.sh bridge
```
You should see:
```
  -> first IMU message received!
  -> receiving sensor data from Gazebo
  [status] HIL sent: 1234  PX4 msgs: 5678  QGC: waiting
```

### Terminal 3 — QGroundControl
Open QGC. It auto-connects via UDP 14550. Do NOT let QGC connect to the serial port directly — the bridge owns it.

If QGC grabs the serial port: disconnect it in QGC > Application Settings > Comm Links.

### Arming & Flying
- Wait ~15-20 seconds for PX4's EKF to converge
- Arm via QGC slider or RC sticks (throttle down + yaw right)
- Take off!

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
# 1. Stop bridge (Ctrl+C)
# 2. Kill Gazebo
pkill -f "gz sim"
# 3. Reboot Pixhawk (unplug USB, replug)
# 4. Start Gazebo
./run_hitl.sh gazebo
# 5. Wait for drone to appear upright
# 6. Start bridge
./run_hitl.sh bridge
# 7. Open QGC
# 8. Wait 15-20s for EKF, then fly
```
