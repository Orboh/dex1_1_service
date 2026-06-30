#!/usr/bin/env python3
"""
drop_to_basket_real.py — G1 okra harvest: 3-phase arm motion on the REAL robot.

Reads poses from data/mujoco_right_arm_drop_pose.json (verified in Isaac Sim).
Sends to real G1 via rt/arm_sdk at 50 Hz.

Safety:
  - Reads actual joint positions from rt/lowstate before moving.
  - Smooth cosine interpolation (ArmInterpolator).
  - Per-joint speed limit enforced via move duration.
  - Interactive confirmation before each phase.
  - Ctrl-C releases arm_sdk at any time.

Usage:
    # Robot must be ON, standing, cable connected to enp8s0.
    conda activate unitree_sim_env
    python drop_to_basket_real.py --iface enp8s0
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

SDK_PATH = Path("/home/techshare/ILkit/unitree_sdk2_python")
if SDK_PATH.exists() and str(SDK_PATH) not in sys.path:
    sys.path.insert(0, str(SDK_PATH))

from arm_interpolator import ArmInterpolator
from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

DEFAULT_POSE_JSON = Path(__file__).resolve().parent / "data" / "mujoco_right_arm_drop_pose.json"

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
kNotUsedJoint = 29  # set to 1.0 to activate arm_sdk mode on real robot

# Waist joints — must be locked when arm_sdk is active (otherwise body rotates)
WAIST_IDX = [12, 13, 14]   # WaistYaw, WaistRoll, WaistPitch

RIGHT_KEYS = [k for k in ARM_IDX if k.startswith("right_")]
LEFT_KEYS  = [k for k in ARM_IDX if k.startswith("left_")]

# ── Safety / motion limits ─────────────────────────────────────────────────────
SEND_RATE_HZ       = 50.0
MAX_SPEED_RAD_S    = 0.4   # conservative for real robot
MIN_DURATION_S     = 3.0

# kp/kd matching official G1_29_ArmController values
KP_SHOULDER_ELBOW  = 80.0   # shoulder + elbow
KP_WRIST           = 40.0   # wrist joints
KD_ARM             = 3.0
KP_WAIST           = 300.0  # high stiffness to lock waist in place
KD_WAIST           = 3.0

WRIST_INDICES = {26, 27, 28, 19, 20, 21}  # left+right wrist joint indices

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
            raise TimeoutError("No LowState within 5 s — is the robot ON and cable connected?")
        time.sleep(0.02)
    return reader.low_state


def read_arm_q(low_state: LowState_) -> dict[str, float]:
    return {name: float(low_state.motor_state[idx].q) for name, idx in ARM_IDX.items()}


def read_waist_q(low_state: LowState_) -> list[float]:
    return [float(low_state.motor_state[i].q) for i in WAIST_IDX]


# ── Command helpers ────────────────────────────────────────────────────────────

def _build_cmd(pose: dict[str, float], mode_machine: int,
               waist_q: list[float]) -> unitree_hg_msg_dds__LowCmd_:
    cmd = unitree_hg_msg_dds__LowCmd_()
    cmd.mode_pr = 0
    cmd.mode_machine = mode_machine
    cmd.motor_cmd[kNotUsedJoint].q = 1.0  # enable arm_sdk mode

    # Lock waist joints at their current positions — prevents body from rotating
    for i, widx in enumerate(WAIST_IDX):
        m = cmd.motor_cmd[widx]
        m.mode = 1
        m.q    = waist_q[i]
        m.dq   = 0.0
        m.tau  = 0.0
        m.kp   = KP_WAIST
        m.kd   = KD_WAIST

    # Arm joints — correct kp/kd per joint type
    for name, idx in ARM_IDX.items():
        m = cmd.motor_cmd[idx]
        m.mode = 1
        m.q    = float(pose.get(name, 0.0))
        m.dq   = 0.0
        m.tau  = 0.0
        m.kp   = KP_WRIST if idx in WRIST_INDICES else KP_SHOULDER_ELBOW
        m.kd   = KD_ARM
    return cmd


def _send(pub: ChannelPublisher, crc: CRC, cmd: unitree_hg_msg_dds__LowCmd_) -> None:
    cmd.crc = crc.Crc(cmd)
    pub.Write(cmd)


def _hold(pub: ChannelPublisher, crc: CRC, pose: dict[str, float],
          mode_machine: int, duration_s: float, waist_q: list[float]) -> None:
    period = 1.0 / SEND_RATE_HZ
    cmd = _build_cmd(pose, mode_machine, waist_q)
    until = time.monotonic() + duration_s
    while time.monotonic() < until:
        t0 = time.perf_counter()
        _send(pub, crc, cmd)
        elapsed = time.perf_counter() - t0
        if elapsed < period:
            time.sleep(period - elapsed)


def release_arm_sdk(pub: ChannelPublisher, crc: CRC, mode_machine: int) -> None:
    cmd = unitree_hg_msg_dds__LowCmd_()
    cmd.mode_pr = 0
    cmd.mode_machine = mode_machine
    cmd.motor_cmd[kNotUsedJoint].q = 0.0  # release arm_sdk
    for _ in range(30):
        cmd.crc = crc.Crc(cmd)
        pub.Write(cmd)
        time.sleep(0.02)
    print("arm_sdk released.")


# ── Pose loading ───────────────────────────────────────────────────────────────

def load_poses(path: Path):
    data = json.loads(path.read_text())
    right_drop   = data["right_arm_drop_pose"]
    left_basket  = data["left_basket_pose"]
    right_center = data["right_arm_body_center"]
    left_center  = data["left_arm_body_center"]
    return right_drop, left_basket, right_center, left_center


# ── Phase runner ───────────────────────────────────────────────────────────────

def _run_phase(
    pub: ChannelPublisher,
    crc: CRC,
    start_pose: dict[str, float],
    goal_pose: dict[str, float],
    mode_machine: int,
    label: str,
    waist_q: list[float],
) -> None:
    right_start = {k: start_pose[k] for k in RIGHT_KEYS}
    right_goal  = {k: goal_pose[k]  for k in RIGHT_KEYS}
    left_start  = {k: start_pose[k] for k in LEFT_KEYS}
    left_goal   = {k: goal_pose[k]  for k in LEFT_KEYS}

    max_delta = max(
        abs(goal_pose[k] - start_pose[k])
        for k in ARM_IDX
    )
    duration_s = max(MIN_DURATION_S, max_delta / MAX_SPEED_RAD_S)

    right_interp = ArmInterpolator(right_start, right_goal, duration_s, round(SEND_RATE_HZ))
    left_interp  = ArmInterpolator(left_start,  left_goal,  duration_s, round(SEND_RATE_HZ))

    period = 1.0 / SEND_RATE_HZ
    print(f"  {label}: {duration_s:.1f}s ({right_interp.total_steps} steps)")
    for step in range(right_interp.total_steps + 1):
        t0   = time.perf_counter()
        pose = {}
        pose.update(right_interp.get_target(step))
        pose.update(left_interp.get_target(step))
        cmd = _build_cmd(pose, mode_machine, waist_q)
        _send(pub, crc, cmd)
        elapsed = time.perf_counter() - t0
        if elapsed < period:
            time.sleep(period - elapsed)


# ── Main ───────────────────────────────────────────────────────────────────────

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Move G1 arms for okra drop on real robot.")
    p.add_argument("--iface",   default="enp8s0", help="DDS network interface (robot cable)")
    p.add_argument("--domain",  type=int, default=0)
    p.add_argument("--pose-json", type=Path, default=DEFAULT_POSE_JSON)
    p.add_argument("--yes", action="store_true", help="skip confirmation prompts")
    return p.parse_args()


def confirm(msg: str, yes: bool) -> None:
    if yes:
        print(f"[auto-yes] {msg}")
        return
    answer = input(f"{msg}  Type YES to continue: ").strip()
    if answer != "YES":
        raise SystemExit("Canceled by user.")


def main() -> None:
    args = parse_args()

    print(f"Loading poses from {args.pose_json}")
    right_drop, left_basket, right_center, left_center = load_poses(args.pose_json)

    print(f"\nConnecting to robot on iface={args.iface} domain={args.domain}...")
    ChannelFactoryInitialize(args.domain, args.iface)

    reader = LowStateReader()
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(reader.callback, 10)

    pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
    pub.Init()
    crc = CRC()

    print("Waiting for LowState from robot...")
    low_state    = wait_for_lowstate(reader)
    mode_machine = int(getattr(low_state, "mode_machine", 0))
    current_q    = read_arm_q(low_state)
    waist_q      = read_waist_q(low_state)   # lock waist at startup position

    print(f"\nmode_machine={mode_machine}")
    print(f"Waist positions locked: {[f'{v:.3f}' for v in waist_q]}  (joints 12,13,14)")
    print("Current arm joint positions:")
    for name, val in current_q.items():
        print(f"  {name:24s} = {val: .4f} rad")

    print("\nTarget poses:")
    print("  Right arm (drop):")
    for k, v in right_drop.items():
        print(f"    {k:24s} = {v: .4f}")
    print("  Left arm (basket):")
    for k, v in left_basket.items():
        print(f"    {k:24s} = {v: .4f}")

    # Body-center waypoints
    center_pose = {**right_center, **left_center}
    final_pose  = {**right_drop, **left_basket}

    confirm(
        "\n[PHASE 1] Both arms will move to body-center waypoint.",
        args.yes,
    )

    try:
        # ── Phase 1: current → body center ────────────────────────────────────
        print("\n→ Phase 1: both arms to body center")
        _run_phase(pub, crc, current_q, center_pose, mode_machine, "phase 1", waist_q)
        _hold(pub, crc, center_pose, mode_machine, 1.0, waist_q)

        confirm(
            "\n[PHASE 2] Right arm will move above basket; left arm will hold basket.",
            args.yes,
        )

        # ── Phase 2: body center → final positions ─────────────────────────────
        print("\n→ Phase 2: arms to final positions")
        _run_phase(pub, crc, center_pose, final_pose, mode_machine, "phase 2", waist_q)

        # ── Phase 3: hold (okra drops) ─────────────────────────────────────────
        confirm(
            "\n[PHASE 3] Holding final pose — MANUALLY OPEN the gripper to drop okra.",
            args.yes,
        )
        print("→ Phase 3: holding final pose for 5 s")
        _hold(pub, crc, final_pose, mode_machine, 5.0, waist_q)
        print("  Done — okra should be in basket.")

        # ── Return to body center ──────────────────────────────────────────────
        confirm("\n[RETURN] Arms will return to body center.", args.yes)
        print("→ Returning to body center")
        cur_q = read_arm_q(reader.low_state)
        _run_phase(pub, crc, cur_q, center_pose, mode_machine, "return", waist_q)
        _hold(pub, crc, center_pose, mode_machine, 1.0, waist_q)

        print("\nAll phases complete.")

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        release_arm_sdk(pub, crc, mode_machine=mode_machine)


if __name__ == "__main__":
    main()
