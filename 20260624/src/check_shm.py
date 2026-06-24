#!/usr/bin/env python3
"""
Diagnostic: read dds_robot_cmd shared memory and print what Isaac Sim sees.
Run this WHILE drop_to_basket_isaac.py is running to verify data flows.
"""
import json
import time
import sys
from multiprocessing import shared_memory

SHM_NAME = "dds_robot_cmd"
ARM_NAMES = [
    "left_shoulder_pitch",  "left_shoulder_roll",  "left_shoulder_yaw",
    "left_elbow",           "left_wrist_roll",      "left_wrist_pitch",
    "left_wrist_yaw",
    "right_shoulder_pitch", "right_shoulder_roll",  "right_shoulder_yaw",
    "right_elbow",          "right_wrist_roll",     "right_wrist_pitch",
    "right_wrist_yaw",
]
ARM_INDICES = list(range(15, 29))  # 15..28

try:
    shm = shared_memory.SharedMemory(name=SHM_NAME)
    print(f"Opened shared memory: {SHM_NAME}  (size={shm.size})")
except FileNotFoundError:
    print(f"ERROR: {SHM_NAME} not found. Is Isaac Sim running?")
    sys.exit(1)

print("Reading every 0.5s — press Ctrl-C to stop\n")
try:
    while True:
        ts    = int.from_bytes(bytes(shm.buf[0:4]), "little")
        dlen  = int.from_bytes(bytes(shm.buf[4:8]), "little")
        if dlen == 0:
            print(f"[ts={ts}] data_len=0 — no command written yet")
        elif dlen > shm.size - 8:
            print(f"[ts={ts}] data_len={dlen} EXCEEDS shm size {shm.size}! Corrupt.")
        else:
            try:
                data = json.loads(bytes(shm.buf[8:8+dlen]).decode())
                pos  = data.get("motor_cmd", {}).get("positions", [])
                print(f"[ts={ts}] data_len={dlen}  motor_cmd.positions ({len(pos)} values):")
                for name, idx in zip(ARM_NAMES, ARM_INDICES):
                    val = pos[idx] if idx < len(pos) else "N/A"
                    print(f"    [{idx:2d}] {name:24s} = {val}")
            except Exception as e:
                print(f"[ts={ts}] JSON parse error: {e}")
        print()
        time.sleep(0.5)
except KeyboardInterrupt:
    print("Stopped.")
finally:
    shm.close()
