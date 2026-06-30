#!/usr/bin/env python3
"""
g1_record_pose.py — snapshot arm joint positions from the real G1.

Connects to the robot, shows live arm joint values, and saves a snapshot
to JSON each time you press Enter.  Use this to:
  - Verify the L-shape basket pose on the real robot
  - Record custom waypoints for the drop motion

Usage:
    conda activate unitree_sim_env
    python g1_record_pose.py --iface enp8s0

    # Change output folder:
    python g1_record_pose.py --iface enp8s0 --out-dir data/recorded_poses
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime
from pathlib import Path

SDK_PATH = Path("/home/techshare/ILkit/unitree_sdk2_python")
if SDK_PATH.exists() and str(SDK_PATH) not in sys.path:
    sys.path.insert(0, str(SDK_PATH))

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

ARM_IDX: dict[str, int] = {
    "left_shoulder_pitch":  15,
    "left_shoulder_roll":   16,
    "left_shoulder_yaw":    17,
    "left_elbow":           18,
    "left_wrist_roll":      19,
    "left_wrist_pitch":     20,
    "left_wrist_yaw":       21,
    "right_shoulder_pitch": 22,
    "right_shoulder_roll":  23,
    "right_shoulder_yaw":   24,
    "right_elbow":          25,
    "right_wrist_roll":     26,
    "right_wrist_pitch":    27,
    "right_wrist_yaw":      28,
}

WAIST_IDX: dict[str, int] = {
    "waist_yaw":   12,
    "waist_roll":  13,
    "waist_pitch": 14,
}


class LowStateReader:
    def __init__(self) -> None:
        self.low_state: LowState_ | None = None
        self.last_update_s = 0.0

    def callback(self, msg: LowState_) -> None:
        self.low_state = msg
        self.last_update_s = time.monotonic()


def read_joints(low_state: LowState_, indices: dict[str, int]) -> dict[str, float]:
    return {name: float(low_state.motor_state[idx].q) for name, idx in indices.items()}


def print_joints(low_state: LowState_) -> None:
    age_ms = (time.monotonic() - reader.last_update_s) * 1000.0
    mode   = getattr(low_state, "mode_machine", "?")
    print(f"\nmode_machine={mode}  state_age={age_ms:.0f} ms")
    print("  Left arm:")
    for name, idx in ARM_IDX.items():
        if name.startswith("left_"):
            q = low_state.motor_state[idx].q
            print(f"    {idx:02d}  {name:22s}  q = {q: .4f} rad")
    print("  Right arm:")
    for name, idx in ARM_IDX.items():
        if name.startswith("right_"):
            q = low_state.motor_state[idx].q
            print(f"    {idx:02d}  {name:22s}  q = {q: .4f} rad")


def save_snapshot(low_state: LowState_, out_dir: Path, label: str) -> Path:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename  = f"{timestamp}_{label}.json" if label else f"{timestamp}.json"
    out_path  = out_dir / filename
    out_dir.mkdir(parents=True, exist_ok=True)

    arm_q   = read_joints(low_state, ARM_IDX)
    waist_q = read_joints(low_state, WAIST_IDX)
    mode    = int(getattr(low_state, "mode_machine", 0))

    payload = {
        "timestamp":    timestamp,
        "label":        label,
        "mode_machine": mode,
        "arm":          arm_q,
        "waist":        waist_q,
        # split for easy copy-paste into real-robot scripts
        "left_arm":  {k: v for k, v in arm_q.items() if k.startswith("left_")},
        "right_arm": {k: v for k, v in arm_q.items() if k.startswith("right_")},
    }
    out_path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    return out_path


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Record G1 arm joint positions to JSON.")
    p.add_argument("--iface",   default="enp8s0")
    p.add_argument("--domain",  type=int, default=0)
    p.add_argument("--out-dir", type=Path,
                   default=Path(__file__).resolve().parent / "data" / "recorded_poses")
    p.add_argument("--rate", type=float, default=2.0, help="live display update frequency in Hz")
    return p.parse_args()


if __name__ == "__main__":
    args   = parse_args()
    reader = LowStateReader()

    ChannelFactoryInitialize(args.domain, args.iface)
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(reader.callback, 10)

    print(f"Connecting to robot on iface={args.iface}...")
    start_s = time.monotonic()
    while reader.low_state is None:
        if time.monotonic() - start_s > 5.0:
            raise TimeoutError("No LowState received — is the robot ON and cable connected?")
        time.sleep(0.05)

    print("Connected!  Showing live arm positions.\n")
    print("Commands:")
    print("  Enter        → save snapshot (you will be asked for a label)")
    print("  Ctrl-C       → quit\n")

    period_s    = 1.0 / args.rate
    next_print  = 0.0
    snapshot_n  = 0

    try:
        import select

        while True:
            now = time.monotonic()
            if now >= next_print:
                print_joints(reader.low_state)
                next_print = now + period_s

            # non-blocking check for Enter key
            if select.select([sys.stdin], [], [], 0.05)[0]:
                label = input("  Label for this snapshot (or just Enter to skip): ").strip()
                if not label:
                    label = f"snapshot_{snapshot_n:03d}"
                out_path = save_snapshot(reader.low_state, args.out_dir, label)
                snapshot_n += 1
                print(f"  Saved → {out_path}")

    except KeyboardInterrupt:
        print("\nDone.")
