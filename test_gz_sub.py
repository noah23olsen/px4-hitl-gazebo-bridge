#!/usr/bin/env python3
"""Minimal test: can we subscribe to a gz-transport topic from Python?"""
import sys, time
sys.path.insert(0, "/usr/local/lib/python3.14/site-packages")

from gz.transport13 import Node
from gz.msgs10 import imu_pb2

count = 0

def on_imu(msg):
    global count
    count += 1
    if count <= 5 or count % 100 == 0:
        print(f"  [{count}] accel=({msg.linear_acceleration.x:.2f}, "
              f"{msg.linear_acceleration.y:.2f}, {msg.linear_acceleration.z:.2f})")

node = Node()
topic = "/world/hitl/model/x500/link/base_link/sensor/imu_sensor/imu"
print(f"Subscribing to: {topic}")
node.subscribe(imu_pb2.IMU, topic, on_imu)
print("Waiting for messages (Ctrl+C to quit) ...")

try:
    while True:
        time.sleep(1)
        if count == 0:
            print("  (still waiting ...)")
        else:
            print(f"  total: {count} messages")
except KeyboardInterrupt:
    print(f"\nDone. Received {count} messages.")
