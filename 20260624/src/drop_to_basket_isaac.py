#!/usr/bin/env python3
"""
drop_to_basket_isaac.py — Full okra harvest sequence in Isaac Sim.

Motion sequence:
  Phase 0 : right arm → pick pose  (above okra)
  Grip    : right gripper closes   (grab okra)
  Phase 1 : both arms → body center (lift okra safely)
  Phase 2 : right arm → above basket, left arm → hold basket
  Phase 3 : right gripper opens    (okra drops into basket)
  Return  : both arms → body center

Usage:
    # Terminal 1 — Isaac Sim:
    cd ~/xr_teleoperate/unitree_sim_isaaclab
    python sim_main.py --task Isaac-G1-Custom-Room-Joint --action_source dds \
        --enable_dex1_dds --robot_type g129 --enable_cameras

    # Terminal 2:
    python drop_to_basket_isaac.py

    # Skip pick-up (okra already in hand):
    python drop_to_basket_isaac.py --skip-pick
"""
from __future__ import annotations

import argparse
import json
import time
import sys
from pathlib import Path
from multiprocessing import shared_memory

from arm_interpolator import ArmInterpolator

# ── Paths ──────────────────────────────────────────────────────────────────────
POSE_JSON = Path(__file__).resolve().parent / "data" / "mujoco_right_arm_drop_pose.json"

# ── Shared memory names ────────────────────────────────────────────────────────
ARM_SHM_NAME     = "dds_robot_cmd"
GRIPPER_SHM_NAME = "isaac_gripper_cmd"
NUM_MOTORS       = 35

# ── Gripper joint values ───────────────────────────────────────────────────────
GRIPPER_CLOSED = 0.024   # holding okra
GRIPPER_OPEN   = -0.02   # drop okra

# ── Motion settings ────────────────────────────────────────────────────────────
SEND_RATE_HZ       = 50
SIM_STEPS_PER_S    = 200
MIN_DURATION_S     = 1.5
MAX_SPEED_RAD_S    = 0.5
GRIP_HOLD_S        = 0.8   # seconds to hold gripper closed after gripping
DROP_HOLD_S        = 1.5   # seconds to hold gripper open (okra falls)
PHASE_HOLD_S       = 0.5   # brief pause between phases

# ── Motor indices ──────────────────────────────────────────────────────────────
MOTOR_IDX: dict[str, int] = {
    "left_shoulder_pitch":  15, "left_shoulder_roll":  16,
    "left_shoulder_yaw":    17, "left_elbow":          18,
    "left_wrist_roll":      19, "left_wrist_pitch":    20,
    "left_wrist_yaw":       21,
    "right_shoulder_pitch": 22, "right_shoulder_roll": 23,
    "right_shoulder_yaw":   24, "right_elbow":         25,
    "right_wrist_roll":     26, "right_wrist_pitch":   27,
    "right_wrist_yaw":      28,
}
RIGHT_KEYS = [k for k in MOTOR_IDX if k.startswith("right_")]
LEFT_KEYS  = [k for k in MOTOR_IDX if k.startswith("left_")]


# ── Shared memory writers ──────────────────────────────────────────────────────

class ArmWriter:
    def __init__(self) -> None:
        try:
            self.shm = shared_memory.SharedMemory(name=ARM_SHM_NAME)
            print(f"  arm shm    : {ARM_SHM_NAME} ✓")
        except FileNotFoundError:
            raise RuntimeError(f"'{ARM_SHM_NAME}' not found — is Isaac Sim running?")

    def write(self, pose: dict[str, float]) -> None:
        positions = [0.0] * NUM_MOTORS
        kp        = [0.0] * NUM_MOTORS
        kd        = [0.0] * NUM_MOTORS
        for name, idx in MOTOR_IDX.items():
            positions[idx] = float(pose.get(name, 0.0))
            kp[idx] = 50.0
            kd[idx] = 2.0
        cmd = {
            "mode_pr": 0, "mode_machine": 0,
            "motor_cmd": {
                "positions":  positions,
                "velocities": [0.0] * NUM_MOTORS,
                "torques":    [0.0] * NUM_MOTORS,
                "kp": kp, "kd": kd,
            },
        }
        blob = json.dumps(cmd).encode()
        ts   = int(time.time()) & 0xFFFFFFFF
        self.shm.buf[0:4] = ts.to_bytes(4, "little")
        self.shm.buf[4:8] = len(blob).to_bytes(4, "little")
        self.shm.buf[8:8 + len(blob)] = blob

    def close(self) -> None:
        self.shm.close()


class GripperWriter:
    def __init__(self) -> None:
        try:
            self.shm = shared_memory.SharedMemory(name=GRIPPER_SHM_NAME)
            print(f"  gripper shm: {GRIPPER_SHM_NAME} ✓")
        except FileNotFoundError:
            print(f"  WARNING: '{GRIPPER_SHM_NAME}' not found — gripper commands disabled")
            self.shm = None

    def write(self, right_pos: float, left_pos: float = GRIPPER_CLOSED) -> None:
        if self.shm is None:
            return
        cmd = {
            "right_gripper_cmd": {
                "positions":  [right_pos],
                "velocities": [0.0],
                "torques":    [0.0],
                "kp":         [50.0],
                "kd":         [2.0],
            },
            "left_gripper_cmd": {
                "positions":  [left_pos],
                "velocities": [0.0],
                "torques":    [0.0],
                "kp":         [50.0],
                "kd":         [2.0],
            },
        }
        blob = json.dumps(cmd).encode()
        ts   = int(time.time()) & 0xFFFFFFFF
        self.shm.buf[0:4] = ts.to_bytes(4, "little")
        self.shm.buf[4:8] = len(blob).to_bytes(4, "little")
        self.shm.buf[8:8 + len(blob)] = blob

    def close(self) -> None:
        if self.shm:
            self.shm.close()


# ── Helpers ────────────────────────────────────────────────────────────────────

def load_poses(path: Path) -> dict:
    data = json.loads(path.read_text())
    return {
        "pick":       data["right_arm_pick_pose"],
        "right_drop": data["right_arm_drop_pose"],
        "left_basket":data["left_basket_pose"],
        "right_center":data["right_arm_body_center"],
        "left_center": data["left_arm_body_center"],
    }


def _duration(start: dict, goal: dict) -> float:
    if not start or not goal:
        return MIN_DURATION_S
    common = set(start) & set(goal)
    if not common:
        return MIN_DURATION_S
    return max(MIN_DURATION_S, max(abs(goal[k] - start[k]) for k in common) / MAX_SPEED_RAD_S)


def _move(arm: ArmWriter, grip: GripperWriter,
          right_start: dict, right_goal: dict,
          left_start: dict, left_goal: dict,
          gripper_pos: float, label: str) -> None:
    combined_start = {**right_start, **left_start}
    combined_goal  = {**right_goal,  **left_goal}
    dur = _duration(combined_start, combined_goal)
    ri  = ArmInterpolator(right_start, right_goal, dur, SIM_STEPS_PER_S)
    li  = ArmInterpolator(left_start,  left_goal,  dur, SIM_STEPS_PER_S)
    period = 1.0 / SEND_RATE_HZ
    print(f"  {label}: {dur:.1f}s")
    for step in range(ri.total_steps + 1):
        t0   = time.perf_counter()
        pose = {}
        pose.update(ri.get_target(step))
        pose.update(li.get_target(step))
        arm.write(pose)
        grip.write(gripper_pos)
        elapsed = time.perf_counter() - t0
        if elapsed < period:
            time.sleep(period - elapsed)


def _hold(arm: ArmWriter, grip: GripperWriter,
          pose: dict, gripper_pos: float, duration_s: float) -> None:
    period  = 1.0 / SEND_RATE_HZ
    until   = time.monotonic() + duration_s
    while time.monotonic() < until:
        t0 = time.perf_counter()
        arm.write(pose)
        grip.write(gripper_pos)
        elapsed = time.perf_counter() - t0
        if elapsed < period:
            time.sleep(period - elapsed)


# ── Main ───────────────────────────────────────────────────────────────────────

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--skip-pick", action="store_true",
                   help="skip pick-up phase (okra already in gripper)")
    return p.parse_args()


def run() -> None:
    args = parse_args()

    print(f"Loading poses from {POSE_JSON}")
    poses = load_poses(POSE_JSON)

    print("\nConnecting to Isaac Sim shared memory...")
    arm  = ArmWriter()
    grip = GripperWriter()
    print()

    zeros_right = {k: 0.0 for k in RIGHT_KEYS}
    zeros_left  = {k: 0.0 for k in LEFT_KEYS}
    center      = {**poses["right_center"], **poses["left_center"]}
    final       = {**poses["right_drop"],   **poses["left_basket"]}
    pick_right  = poses["pick"]

    print("Waiting 2s before motion starts...")
    time.sleep(2.0)

    try:
        if not args.skip_pick:
            # ── Phase 0: both arms → pick pose (right reaches for okra) ──────
            print("\n→ Phase 0: right arm reaches for okra")
            _move(arm, grip,
                  zeros_right,  pick_right,
                  zeros_left,   poses["left_center"],
                  GRIPPER_OPEN, "reach to okra")
            _hold(arm, grip, {**pick_right, **poses["left_center"]},
                  GRIPPER_OPEN, PHASE_HOLD_S)

            # ── Grip: close gripper around okra ───────────────────────────────
            print("\n→ Grip: closing gripper around okra")
            _hold(arm, grip, {**pick_right, **poses["left_center"]},
                  GRIPPER_CLOSED, GRIP_HOLD_S)
            print("  gripper CLOSED ✓")
        else:
            print("\n[--skip-pick] Skipping pick-up — starting with gripper closed")

        # ── Phase 1: both arms → body center (lift okra) ──────────────────────
        print("\n→ Phase 1: lifting okra to body center")
        start_right = pick_right if not args.skip_pick else zeros_right
        start_left  = poses["left_center"] if not args.skip_pick else zeros_left
        _move(arm, grip,
              start_right,   poses["right_center"],
              start_left,    poses["left_center"],
              GRIPPER_CLOSED, "lift to center")
        _hold(arm, grip, center, GRIPPER_CLOSED, PHASE_HOLD_S)

        # ── Phase 2: right arm → above basket, left arm → basket ──────────────
        print("\n→ Phase 2: positioning above basket")
        _move(arm, grip,
              poses["right_center"], poses["right_drop"],
              poses["left_center"],  poses["left_basket"],
              GRIPPER_CLOSED, "move to drop pose")
        _hold(arm, grip, final, GRIPPER_CLOSED, PHASE_HOLD_S)

        # ── Phase 3: open gripper → okra drops ────────────────────────────────
        print("\n→ Phase 3: opening gripper — okra drops into basket")
        _hold(arm, grip, final, GRIPPER_OPEN, DROP_HOLD_S)
        print("  gripper OPEN ✓  — okra dropped!")

        # ── Return: both arms → body center ───────────────────────────────────
        print("\n→ Return: arms returning to body center")
        _move(arm, grip,
              poses["right_drop"],   poses["right_center"],
              poses["left_basket"],  poses["left_center"],
              GRIPPER_OPEN, "return to center")

        print("\nAll phases complete.")

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        arm.close()
        grip.close()


if __name__ == "__main__":
    run()
