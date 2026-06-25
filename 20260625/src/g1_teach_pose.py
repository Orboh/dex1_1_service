#!/usr/bin/env python3
"""
g1_teach_pose.py  —  Kinesthetic teaching for the real G1.

1. Puts both arms into SOFT / damping mode (kp=0, kd=3).
   You can physically push the arms to any position.
2. Shows live joint angles on screen.
3. Press Enter  → saves current angles to a JSON file.
4. Press Ctrl-C → releases arm_sdk, arms go stiff again.

Usage:
    conda activate unitree_sim_env
    python g1_teach_pose.py --iface enp8s0

    # Custom output folder:
    python g1_teach_pose.py --iface enp8s0 --out-dir data/taught_poses
"""
from __future__ import annotations

import argparse
import json
import select
import sys
import time
from datetime import datetime
from pathlib import Path

SDK_PATH = Path("/home/techshare/ILkit/unitree_sdk2_python")
if SDK_PATH.exists() and str(SDK_PATH) not in sys.path:
    sys.path.insert(0, str(SDK_PATH))

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

# ── Joint indices ──────────────────────────────────────────────────────────────
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

kNotUsedJoint = 29

# Damping mode — arm is soft, you can push it
KP_DAMPING = 0.0
KD_DAMPING = 3.0

# Waist stays locked at its current position
KP_WAIST = 300.0
KD_WAIST = 3.0

SEND_RATE_HZ = 50.0


# ── LowState reader ────────────────────────────────────────────────────────────

class LowStateReader:
    def __init__(self) -> None:
        self.low_state: LowState_ | None = None
        self.last_update_s = 0.0

    def callback(self, msg: LowState_) -> None:
        self.low_state = msg
        self.last_update_s = time.monotonic()


def wait_for_lowstate(reader: LowStateReader, timeout_s: float = 5.0) -> LowState_:
    start = time.monotonic()
    while reader.low_state is None:
        if time.monotonic() - start > timeout_s:
            raise TimeoutError("No LowState — is the robot ON and cable connected?")
        time.sleep(0.02)
    return reader.low_state


def read_joints(low_state: LowState_, indices: dict[str, int]) -> dict[str, float]:
    return {name: float(low_state.motor_state[idx].q) for name, idx in indices.items()}


# ── Command builder ────────────────────────────────────────────────────────────

def build_damping_cmd(
    mode_machine: int,
    waist_q: list[float],
) -> unitree_hg_msg_dds__LowCmd_:
    """Arms: kp=0, kd=3 (soft/backdrivable).  Waist: locked."""
    cmd = unitree_hg_msg_dds__LowCmd_()
    cmd.mode_pr = 0
    cmd.mode_machine = mode_machine
    cmd.motor_cmd[kNotUsedJoint].q = 1.0   # activate arm_sdk

    # Arms — soft damping only
    for idx in ARM_IDX.values():
        m = cmd.motor_cmd[idx]
        m.mode = 1
        m.q   = 0.0   # target doesn't matter when kp=0
        m.dq  = 0.0
        m.tau = 0.0
        m.kp  = KP_DAMPING
        m.kd  = KD_DAMPING

    # Waist — hold current position
    for i, widx in enumerate([12, 13, 14]):
        m = cmd.motor_cmd[widx]
        m.mode = 1
        m.q   = waist_q[i]
        m.dq  = 0.0
        m.tau = 0.0
        m.kp  = KP_WAIST
        m.kd  = KD_WAIST

    return cmd


def release_arm_sdk(pub: ChannelPublisher, crc: CRC, mode_machine: int) -> None:
    cmd = unitree_hg_msg_dds__LowCmd_()
    cmd.mode_pr = 0
    cmd.mode_machine = mode_machine
    cmd.motor_cmd[kNotUsedJoint].q = 0.0   # release arm_sdk
    for _ in range(30):
        cmd.crc = crc.Crc(cmd)
        pub.Write(cmd)
        time.sleep(0.02)
    print("arm_sdk released — arms are stiff again.")


# ── Display ────────────────────────────────────────────────────────────────────

def print_live(low_state: LowState_, snapshot_n: int) -> None:
    age_ms = (time.monotonic() - low_state_reader.last_update_s) * 1000.0
    print(
        f"\r[snapshots saved: {snapshot_n}]  state_age={age_ms:.0f} ms"
        "  | Enter=record  Ctrl-C=quit",
        end="",
        flush=True,
    )


def print_joints(low_state: LowState_) -> None:
    print("\nCurrent arm joint positions:")
    print("  Left arm:")
    for name, idx in ARM_IDX.items():
        if name.startswith("left_"):
            q = low_state.motor_state[idx].q
            print(f"    {idx:02d}  {name:22s}  {q: .4f} rad")
    print("  Right arm:")
    for name, idx in ARM_IDX.items():
        if name.startswith("right_"):
            q = low_state.motor_state[idx].q
            print(f"    {idx:02d}  {name:22s}  {q: .4f} rad")


# ── Snapshot saver ─────────────────────────────────────────────────────────────

def save_snapshot(
    low_state: LowState_,
    out_dir: Path,
    label: str,
    snapshot_n: int,
) -> Path:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename  = f"{snapshot_n:03d}_{timestamp}_{label}.json"
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / filename

    arm_q   = read_joints(low_state, ARM_IDX)
    waist_q = read_joints(low_state, WAIST_IDX)
    mode    = int(getattr(low_state, "mode_machine", 0))

    payload = {
        "snapshot":     snapshot_n,
        "timestamp":    timestamp,
        "label":        label,
        "mode_machine": mode,
        "left_arm":  {k: v for k, v in arm_q.items() if k.startswith("left_")},
        "right_arm": {k: v for k, v in arm_q.items() if k.startswith("right_")},
        "waist":     waist_q,
    }
    out_path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    return out_path


# ── Main ───────────────────────────────────────────────────────────────────────

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Kinesthetic teaching — move G1 arm by hand and record poses.")
    p.add_argument("--iface",   default="enp8s0")
    p.add_argument("--domain",  type=int, default=0)
    p.add_argument("--out-dir", type=Path,
                   default=Path(__file__).resolve().parent / "data" / "taught_poses")
    p.add_argument("--rate",    type=float, default=4.0, help="live display refresh Hz")
    return p.parse_args()


low_state_reader: LowStateReader   # module-level so print_live can access it


if __name__ == "__main__":
    args = parse_args()

    low_state_reader = LowStateReader()
    ChannelFactoryInitialize(args.domain, args.iface)

    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(low_state_reader.callback, 10)

    pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
    pub.Init()
    crc = CRC()

    print(f"Connecting on iface={args.iface}...")
    low_state    = wait_for_lowstate(low_state_reader)
    mode_machine = int(getattr(low_state, "mode_machine", 0))
    waist_q      = [float(low_state.motor_state[i].q) for i in [12, 13, 14]]

    print(f"Connected.  mode_machine={mode_machine}")
    print_joints(low_state)

    print("\n>>> Arms switching to SOFT mode (kp=0, kd=3) <<<")
    print("    You can now PUSH the arms by hand to any position.")
    print("    Press Enter to record.  Press Ctrl-C to quit.\n")

    damping_cmd = build_damping_cmd(mode_machine, waist_q)
    period_s    = 1.0 / SEND_RATE_HZ
    display_period = 1.0 / args.rate
    next_display   = 0.0
    snapshot_n     = 0

    try:
        while True:
            t0 = time.perf_counter()

            # Keep sending damping command so arm_sdk stays active
            damping_cmd.crc = crc.Crc(damping_cmd)
            pub.Write(damping_cmd)

            now = time.monotonic()
            if now >= next_display:
                print_live(low_state_reader.low_state, snapshot_n)
                next_display = now + display_period

            # Non-blocking check for Enter key
            if select.select([sys.stdin], [], [], 0.0)[0]:
                line = sys.stdin.readline()
                print()   # newline after the \r status line
                print_joints(low_state_reader.low_state)
                label = input("  Label (e.g. left_basket, drop_point): ").strip()
                if not label:
                    label = f"pose_{snapshot_n:03d}"
                out_path = save_snapshot(
                    low_state_reader.low_state, args.out_dir, label, snapshot_n
                )
                snapshot_n += 1
                print(f"  Saved ({snapshot_n}) → {out_path}\n")

            elapsed = time.perf_counter() - t0
            if elapsed < period_s:
                time.sleep(period_s - elapsed)

    except KeyboardInterrupt:
        print("\n\nReleasing arm_sdk...")
    finally:
        release_arm_sdk(pub, crc, mode_machine)
        print(f"Done. {snapshot_n} snapshot(s) saved to {args.out_dir}/")
