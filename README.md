# PX4 HITL Bridge for Gazebo Harmonic

A custom Python bridge that connects a **real Pixhawk flight controller** to **Gazebo Harmonic (gz-sim 8)** for hardware-in-the-loop simulation. The bridge also forwards MAVLink to QGroundControl over UDP, so you can monitor and command the vehicle.

PX4's official Gazebo integration only supports SITL (simulated autopilot). This project enables true HITL — your real Pixhawk firmware thinks it's flying a real drone.

## Architecture

```
┌──────────┐  gz-transport   ┌──────────────┐  USB serial   ┌──────────┐
│  Gazebo   │ ──────────────> │              │ ────────────> │          │
│ Harmonic  │  IMU/Mag/Baro  │ hitl_bridge  │  HIL_SENSOR   │ Pixhawk  │
│  (gz-sim) │  GPS sensors   │    .py       │  HIL_GPS      │  (PX4)   │
│           │ <────────────── │              │ <──────────── │          │
│           │  motor speeds  │              │  HIL_ACTUATOR │          │
└──────────┘                 │              │               └──────────┘
                              │   UDP :14550 │
                              └──────┬───────┘
                                     │
                              ┌──────▼───────┐
                              │     QGC      │
                              └──────────────┘
```

## What the bridge does

- Subscribes to Gazebo sensor topics (IMU @ 250Hz, magnetometer @ 100Hz, barometer @ 50Hz, GPS @ 30Hz) via gz-transport
- Converts Gazebo FLU frame to PX4 FRD frame
- Sends `HIL_SENSOR` and `HIL_GPS` MAVLink messages to Pixhawk over USB serial
- Reads `HIL_ACTUATOR_CONTROLS` from Pixhawk and publishes motor speed commands back to Gazebo
- Forwards all MAVLink traffic to QGroundControl over UDP (acts as a MAVLink router)

## Requirements

- Gazebo Harmonic (gz-sim 8)
- PX4-Autopilot source (for x500 model files) — `~/DEV/PX4-Autopilot`
- Python 3 with gz-transport and gz-msgs bindings
- pymavlink, pyserial
- Pixhawk with PX4 firmware
- QGroundControl

## Pixhawk Setup (one-time)

Connect to QGroundControl and set:
- `SYS_HITL` = **Enabled**
- `COM_RC_IN_MODE` = **RC Transmitter only** (if using a real RC controller)
- Reboot the Pixhawk

## Quick Start

```bash
# Setup Python environment
python3 -m venv hitl_bridge_venv
source hitl_bridge_venv/bin/activate
pip install pymavlink pyserial

# Terminal 1 — Start Gazebo
./run_hitl.sh gazebo

# Terminal 2 — Start bridge (after drone appears in Gazebo)
./run_hitl.sh bridge

# Terminal 3 — Open QGroundControl (connects automatically via UDP)
# Wait ~15-20s for EKF to converge, then arm and fly
```

## Using a Real RC Controller

1. Remove all propellers from the drone
2. Plug in the drone battery (powers RC receiver)
3. Turn on transmitter
4. Verify stick inputs in QGC > Vehicle Setup > Radio
5. Fly with real sticks in the sim

## Files

| File | Description |
|------|-------------|
| `hitl_bridge.py` | The bridge — Gazebo <-> Pixhawk MAVLink <-> QGC UDP |
| `hitl_world.sdf` | Gazebo world with x500 quadcopter and sensors |
| `run_hitl.sh` | Launcher script for Gazebo and bridge |
| `diagnose_hitl.sh` | Pre-flight diagnostic checker |
| `HITL_SETUP.md` | Detailed setup guide and troubleshooting |

## Troubleshooting

Run the diagnostic script to check all prerequisites:

```bash
./diagnose_hitl.sh
```

See [HITL_SETUP.md](HITL_SETUP.md) for detailed troubleshooting.
