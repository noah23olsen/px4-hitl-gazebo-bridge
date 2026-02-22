#!/usr/bin/env python3
"""
PX4 HITL Bridge for Gazebo Harmonic (gz-sim)

Bridges sensor data from gz-sim to a PX4 flight controller via MAVLink serial,
and motor commands from PX4 back to gz-sim.

Data flow:
  Gazebo (IMU, Mag, Baro, GPS) -> HIL_SENSOR / HIL_GPS -> Pixhawk serial
  Pixhawk serial -> HIL_ACTUATOR_CONTROLS -> Gazebo motor speed commands
"""

import sys
import os
import time
import math
import glob
import socket
import threading
import signal
import argparse
from datetime import datetime

# Add system site-packages for gz bindings (installed outside venv)
sys.path.insert(0, "/usr/local/lib/python3.14/site-packages")


def ts():
    """Timestamp prefix for log messages."""
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]

from pymavlink import mavutil
from gz.transport13 import Node
from gz.msgs10 import (
    imu_pb2,
    fluid_pressure_pb2,
    magnetometer_pb2,
    navsat_pb2,
    actuators_pb2,
)

# --- defaults ---
SERIAL_PORT = "/dev/tty.usbmodem01"
BAUD_RATE = 921600
WORLD_NAME = "hitl"
MODEL_NAME = "x500"
QGC_UDP_PORT = 14550  # QGroundControl default MAVLink UDP port

# Motor constants (from x500/model.sdf)
MAX_ROTOR_VELOCITY = 1000.0  # rad/s
SLOWDOWN_SIM = 10.0
MOTOR_SPEED_SCALE = MAX_ROTOR_VELOCITY * SLOWDOWN_SIM  # 10 000

# HIL_SENSOR fields_updated bitmask values
SENSOR_ACCEL = 0b111
SENSOR_GYRO = 0b111_000
SENSOR_MAG = 0b111_000_000
SENSOR_BARO = 0b1_101_000_000_000


class HITLBridge:
    def __init__(self, serial_port, baud_rate, world_name, model_name):
        self.world = world_name
        self.model = model_name
        self.running = False

        # Latest sensor values (written by gz callbacks, read by main loop)
        self.imu_data = None
        self.mag_data = None
        self.baro_data = None
        self.gps_data = None
        self.sim_time_us = 0
        self.lock = threading.Lock()

        # Flags: set True by callbacks, cleared after sending
        self.new_imu = False
        self.new_gps = False

        # MAVLink serial connection (retry until port is available)
        self.mav = self._connect_serial(serial_port, baud_rate)

        # UDP socket for QGC forwarding
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.setblocking(False)
        self.udp_sock.bind(("127.0.0.1", QGC_UDP_PORT + 1))  # listen on 14551
        self.qgc_addr = ("127.0.0.1", QGC_UDP_PORT)           # send to 14550
        self.qgc_seen = False
        print(f"  QGC UDP: forwarding to {self.qgc_addr[0]}:{self.qgc_addr[1]}")

        # gz-transport
        self.node = Node()
        self._setup_subscribers()
        self._setup_publisher()

    @staticmethod
    def _connect_serial(port, baud):
        while True:
            # Auto-detect port if the specified one doesn't exist
            actual_port = port
            if not os.path.exists(port):
                candidates = glob.glob("/dev/tty.usbmodem*")
                if candidates:
                    actual_port = candidates[0]
                    print(f"  {port} not found, using {actual_port}")
                else:
                    print(f"  Waiting for Pixhawk USB ...", end="\r")
                    time.sleep(1)
                    continue
            try:
                print(f"Connecting to Pixhawk on {actual_port} @ {baud} ...")
                mav = mavutil.mavlink_connection(actual_port, baud=baud)
                print(f"  Serial connected")
                return mav
            except Exception as e:
                reason = str(e)
                if "Resource busy" in reason:
                    print(f"  Port busy, retrying in 2s ... (close QGC serial link)")
                elif "No such file" in reason:
                    print(f"  Port disappeared, retrying ...")
                else:
                    print(f"  Serial error: {reason}, retrying ...")
                time.sleep(2)

    # ------------------------------------------------------------------ #
    #  gz-transport setup                                                 #
    # ------------------------------------------------------------------ #

    def _sensor_topic(self, sensor_path):
        return (
            f"/world/{self.world}/model/{self.model}"
            f"/link/base_link/sensor/{sensor_path}"
        )

    def _setup_subscribers(self):
        topics = {
            "imu_sensor/imu": (imu_pb2.IMU, self._on_imu),
            "magnetometer_sensor/magnetometer": (
                magnetometer_pb2.Magnetometer,
                self._on_mag,
            ),
            "air_pressure_sensor/air_pressure": (
                fluid_pressure_pb2.FluidPressure,
                self._on_baro,
            ),
            "navsat_sensor/navsat": (navsat_pb2.NavSat, self._on_gps),
        }
        for path, (msg_cls, cb) in topics.items():
            topic = self._sensor_topic(path)
            self.node.subscribe(msg_cls, topic, cb)
            print(f"  sub: {topic}")

    def _setup_publisher(self):
        topic = f"/{self.model}/command/motor_speed"
        self.motor_pub = self.node.advertise(topic, actuators_pb2.Actuators())
        print(f"  pub: {topic}")

    # ------------------------------------------------------------------ #
    #  gz-transport sensor callbacks                                      #
    # ------------------------------------------------------------------ #

    @staticmethod
    def _stamp_us(header):
        if header.HasField("stamp"):
            return header.stamp.sec * 1_000_000 + header.stamp.nsec // 1000
        return int(time.time() * 1e6)

    def _on_imu(self, msg):
        with self.lock:
            if self.imu_data is None:
                print(f"  [{ts()}] -> first IMU message received!")
            self.sim_time_us = self._stamp_us(msg.header)
            self.imu_data = msg
            self.new_imu = True

    def _on_mag(self, msg):
        with self.lock:
            if self.mag_data is None:
                print(f"  [{ts()}] -> first MAG: x={msg.field_tesla.x:.6f} "
                      f"y={msg.field_tesla.y:.6f} z={msg.field_tesla.z:.6f}")
            self.mag_data = msg

    def _on_baro(self, msg):
        with self.lock:
            if self.baro_data is None:
                print(f"  [{ts()}] -> first BARO: {msg.pressure:.1f} Pa "
                      f"({msg.pressure * 0.01:.2f} hPa)")
            self.baro_data = msg

    def _on_gps(self, msg):
        with self.lock:
            if self.gps_data is None:
                print(f"  [{ts()}] -> first GPS: lat={msg.latitude_deg:.6f} "
                      f"lon={msg.longitude_deg:.6f} alt={msg.altitude:.1f}")
            self.gps_data = msg
            self.new_gps = True

    # ------------------------------------------------------------------ #
    #  MAVLink senders                                                    #
    # ------------------------------------------------------------------ #

    def _send_hil_sensor(self):
        with self.lock:
            if self.imu_data is None:
                return
            imu = self.imu_data
            t_us = self.sim_time_us
            fields = 0

            # Accel m/s^2  FLU -> FRD  (x, -y, -z)
            ax = imu.linear_acceleration.x
            ay = -imu.linear_acceleration.y
            az = -imu.linear_acceleration.z
            fields |= SENSOR_ACCEL

            # Gyro rad/s   FLU -> FRD
            gx = imu.angular_velocity.x
            gy = -imu.angular_velocity.y
            gz_v = -imu.angular_velocity.z
            fields |= SENSOR_GYRO

            # Mag  (x, -y, z) — gz-sim Harmonic mag z already points
            # down (FRD convention), so do NOT negate it.
            mx = my = mz = 0.0
            if self.mag_data is not None:
                mx = self.mag_data.field_tesla.x
                my = -self.mag_data.field_tesla.y
                mz = self.mag_data.field_tesla.z
                fields |= SENSOR_MAG

            # Baro  Pa -> hPa
            abs_pressure = pressure_alt = 0.0
            temperature = 20.0
            if self.baro_data is not None:
                abs_pressure = self.baro_data.pressure * 0.01
                pressure_alt = (
                    (1.0 - (abs_pressure / 1013.25) ** 0.190284) * 44330.0
                )
                fields |= SENSOR_BARO

        # One-time dump of sensor values for debugging
        if not hasattr(self, '_dumped'):
            self._dumped = True
            print(f"\n  === SENSOR DUMP (first HIL_SENSOR) ===")
            print(f"  time_us:  {t_us}")
            print(f"  accel:    ({ax:.4f}, {ay:.4f}, {az:.4f})  [expect ~(0, 0, -9.8)]")
            print(f"  gyro:     ({gx:.4f}, {gy:.4f}, {gz_v:.4f})  [expect ~(0, 0, 0)]")
            print(f"  mag:      ({mx:.6f}, {my:.6f}, {mz:.6f})")
            print(f"  baro:     {abs_pressure:.2f} hPa  alt={pressure_alt:.2f} m")
            print(f"  fields:   0x{fields:04x}")
            print(f"  ========================================\n")

        self.mav.mav.hil_sensor_send(
            t_us,
            ax, ay, az,
            gx, gy, gz_v,
            mx, my, mz,
            abs_pressure,
            0.0,            # diff_pressure
            pressure_alt,
            temperature,
            fields,
            0,              # sensor id
        )

    def _send_hil_gps(self):
        with self.lock:
            if self.gps_data is None:
                return
            gps = self.gps_data
            t_us = self.sim_time_us

            lat = int(gps.latitude_deg * 1e7)
            lon = int(gps.longitude_deg * 1e7)
            alt = int(gps.altitude * 1000)  # m -> mm

            # Velocity  ENU -> NED, clamped to MAVLink int16/uint16 range
            vn = max(-32767, min(32767, int(gps.velocity_north * 100)))
            ve = max(-32767, min(32767, int(gps.velocity_east * 100)))
            vd = max(-32767, min(32767, int(-gps.velocity_up * 100)))
            vel = min(65535, int(math.sqrt(vn * vn + ve * ve + vd * vd)))
            cog = int(math.degrees(math.atan2(ve, vn)) * 100) % 36000

        self.mav.mav.hil_gps_send(
            t_us,
            3,       # fix_type  3-D
            lat, lon, alt,
            20, 20,   # eph, epv  (cm)
            vel,
            vn, ve, vd,
            cog,
            20,      # satellites_visible
        )

    def _send_heartbeat(self):
        self.mav.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GENERIC,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0,
        )

    # ------------------------------------------------------------------ #
    #  Actuator controls  (Pixhawk -> Gazebo)                             #
    # ------------------------------------------------------------------ #

    def _handle_actuator_controls(self, msg):
        if not hasattr(self, '_motor_count'):
            self._motor_count = 0
        self._motor_count += 1
        act = actuators_pb2.Actuators()
        for i in range(4):
            speed = max(0.0, msg.controls[i]) * MOTOR_SPEED_SCALE
            act.velocity.append(speed)
        if self._motor_count == 1:
            print(f"  [{ts()}] -> first MOTOR cmd: "
                  f"[{msg.controls[0]:.3f}, {msg.controls[1]:.3f}, "
                  f"{msg.controls[2]:.3f}, {msg.controls[3]:.3f}] "
                  f"-> speeds [{act.velocity[0]:.0f}, {act.velocity[1]:.0f}, "
                  f"{act.velocity[2]:.0f}, {act.velocity[3]:.0f}]")
        self.motor_pub.publish(act)

    # ------------------------------------------------------------------ #
    #  Main loop                                                          #
    # ------------------------------------------------------------------ #

    def run(self):
        self.running = True
        last_hb = 0.0
        last_status = 0.0
        got_first_imu = False
        self.px4_msg_count = 0
        self.hil_sent_count = 0

        print("\nHITL bridge running – waiting for sensor data from Gazebo ...")
        print("  (start gz sim with:  gz sim hitl_world.sdf )\n")

        while self.running:
            now = time.time()

            # heartbeat 1 Hz
            if now - last_hb >= 1.0:
                self._send_heartbeat()
                last_hb = now

            # drain all available Pixhawk messages and forward to QGC
            while True:
                msg = self.mav.recv_match(blocking=False)
                if msg is None:
                    break
                mtype = msg.get_type()
                if mtype == "HIL_ACTUATOR_CONTROLS":
                    self._handle_actuator_controls(msg)
                elif mtype == "STATUSTEXT":
                    print(f"  [{ts()}] PX4: {msg.text}")
                if mtype != "BAD_DATA":
                    self.px4_msg_count += 1
                    try:
                        self.udp_sock.sendto(msg.get_msgbuf(), self.qgc_addr)
                    except OSError:
                        pass

            # drain all available QGC packets and forward to Pixhawk
            while True:
                try:
                    data, addr = self.udp_sock.recvfrom(4096)
                    if data:
                        if not self.qgc_seen:
                            print(f"  [{ts()}] -> QGroundControl connected")
                            self.qgc_seen = True
                        self.mav.write(data)
                except BlockingIOError:
                    break

            # send sensor data only when new data arrives from Gazebo
            with self.lock:
                send_imu = self.new_imu
                send_gps = self.new_gps
                self.new_imu = False
                self.new_gps = False

            if send_imu:
                if not got_first_imu:
                    print(f"  [{ts()}] -> receiving sensor data from Gazebo")
                    got_first_imu = True
                self._send_hil_sensor()
                self.hil_sent_count += 1

            if send_gps:
                self._send_hil_gps()

            # periodic status (every 5s)
            if got_first_imu and now - last_status >= 5.0:
                sensors = []
                with self.lock:
                    if self.baro_data is None:
                        sensors.append("NO BARO")
                    if self.gps_data is None:
                        sensors.append("NO GPS")
                    if self.mag_data is None:
                        sensors.append("NO MAG")
                sensor_warn = f"  MISSING: {', '.join(sensors)}" if sensors else ""
                motor_count = getattr(self, '_motor_count', 0)
                print(f"  [{ts()}] HIL sent: {self.hil_sent_count}  "
                      f"PX4: {self.px4_msg_count}  "
                      f"motors: {motor_count}  "
                      f"QGC: {'connected' if self.qgc_seen else 'waiting'}"
                      f"{sensor_warn}")
                last_status = now

            time.sleep(0.001)  # 1 kHz poll ceiling

    def stop(self):
        self.running = False
        print("Bridge stopped.")


def main():
    parser = argparse.ArgumentParser(
        description="PX4 HITL Bridge for Gazebo Harmonic"
    )
    parser.add_argument("--port", default=SERIAL_PORT)
    parser.add_argument("--baud", type=int, default=BAUD_RATE)
    parser.add_argument("--world", default=WORLD_NAME)
    parser.add_argument("--model", default=MODEL_NAME)
    args = parser.parse_args()

    bridge = HITLBridge(args.port, args.baud, args.world, args.model)

    def on_signal(sig, frame):
        print("\nShutting down ...")
        bridge.stop()

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    bridge.run()


if __name__ == "__main__":
    main()
