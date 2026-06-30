#!/usr/bin/env python3
"""
demo_okra_harvest.py — Slow, visible okra harvest demo in Isaac Sim.

Motion you will SEE in Isaac Sim:
  1. Right arm reaches DOWN  (picking okra from table level)
  2. Gripper CLOSES          (grabs okra)
  3. Right arm lifts UP      (carrying okra)
  4. Both arms move to ready (left arm raises basket)
  5. Right arm swings ACROSS (above left hand basket)
  6. Gripper OPENS           (okra drops into basket)
  7. Both arms return to rest

Usage:
    # Terminal 1 — Isaac Sim running
    # Terminal 2:
    conda activate unitree_sim_env
    python demo_okra_harvest.py
"""
from __future__ import annotations

import json
import time
import sys
from pathlib import Path
from multiprocessing import shared_memory

from arm_interpolator import ArmInterpolator

POSE_JSON = Path(__file__).resolve().parent / "data" / "mujoco_right_arm_drop_pose.json"

ARM_SHM_NAME     = "dds_robot_cmd"
GRIPPER_SHM_NAME = "isaac_gripper_cmd"
NUM_MOTORS       = 35

GRIPPER_CLOSED = 0.024
GRIPPER_OPEN   = -0.02

# Slow speed so every phase is clearly visible
SEND_RATE_HZ    = 50
MAX_SPEED_RAD_S = 0.25   # very slow — easy to follow
MIN_DURATION_S  = 2.5
PAUSE_S         = 1.2    # pause between phases so viewer can see each step

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


# ── Shared memory ──────────────────────────────────────────────────────────────

def _connect_shm(name: str, required: bool = True):
    try:
        shm = shared_memory.SharedMemory(name=name)
        print(f"  ✓ {name}")
        return shm
    except FileNotFoundError:
        if required:
            print(f"  ✗ {name} NOT FOUND — is Isaac Sim running?")
            sys.exit(1)
        print(f"  ✗ {name} not found (gripper disabled)")
        return None


def _write_arm(shm, pose: dict[str, float]) -> None:
    positions = [0.0] * NUM_MOTORS
    kp = [0.0] * NUM_MOTORS
    kd = [0.0] * NUM_MOTORS
    for name, idx in MOTOR_IDX.items():
        positions[idx] = float(pose.get(name, 0.0))
        kp[idx] = 50.0
        kd[idx] = 2.0
    blob = json.dumps({
        "mode_pr": 0, "mode_machine": 0,
        "motor_cmd": {
            "positions": positions, "velocities": [0.0]*NUM_MOTORS,
            "torques": [0.0]*NUM_MOTORS, "kp": kp, "kd": kd,
        },
    }).encode()
    ts = int(time.time()) & 0xFFFFFFFF
    shm.buf[0:4] = ts.to_bytes(4, "little")
    shm.buf[4:8] = len(blob).to_bytes(4, "little")
    shm.buf[8:8+len(blob)] = blob


def _write_gripper(shm, right_pos: float) -> None:
    if shm is None:
        return
    blob = json.dumps({
        "right_gripper_cmd": {
            "positions": [right_pos], "velocities": [0.0],
            "torques": [0.0], "kp": [50.0], "kd": [2.0],
        },
        "left_gripper_cmd": {
            "positions": [GRIPPER_CLOSED], "velocities": [0.0],
            "torques": [0.0], "kp": [50.0], "kd": [2.0],
        },
    }).encode()
    ts = int(time.time()) & 0xFFFFFFFF
    shm.buf[0:4] = ts.to_bytes(4, "little")
    shm.buf[4:8] = len(blob).to_bytes(4, "little")
    shm.buf[8:8+len(blob)] = blob


# ── Motion helpers ─────────────────────────────────────────────────────────────

def _dur(start: dict, goal: dict) -> float:
    common = set(start) & set(goal)
    if not common:
        return MIN_DURATION_S
    return max(MIN_DURATION_S,
               max(abs(goal[k] - start[k]) for k in common) / MAX_SPEED_RAD_S)


def move(arm_shm, grip_shm,
         right_start: dict, right_goal: dict,
         left_start: dict, left_goal: dict,
         gripper: float, label: str) -> None:
    dur = _dur({**right_start, **left_start}, {**right_goal, **left_goal})
    ri  = ArmInterpolator(right_start, right_goal, dur, 200)
    li  = ArmInterpolator(left_start,  left_goal,  dur, 200)
    period = 1.0 / SEND_RATE_HZ
    print(f"    moving... {dur:.1f}s")
    for step in range(ri.total_steps + 1):
        t0 = time.perf_counter()
        pose = {}
        pose.update(ri.get_target(step))
        pose.update(li.get_target(step))
        _write_arm(arm_shm, pose)
        _write_gripper(grip_shm, gripper)
        elapsed = time.perf_counter() - t0
        if elapsed < period:
            time.sleep(period - elapsed)


def hold(arm_shm, grip_shm, pose: dict, gripper: float, duration_s: float) -> None:
    period = 1.0 / SEND_RATE_HZ
    until  = time.monotonic() + duration_s
    while time.monotonic() < until:
        t0 = time.perf_counter()
        _write_arm(arm_shm, pose)
        _write_gripper(grip_shm, gripper)
        elapsed = time.perf_counter() - t0
        if elapsed < period:
            time.sleep(period - elapsed)


# ── Main ───────────────────────────────────────────────────────────────────────

def main() -> None:
    data = json.loads(POSE_JSON.read_text())

    # All poses
    pick    = data["right_arm_pick_pose"]      # right arm reaches DOWN
    r_ctr   = data["right_arm_body_center"]    # right arm at chest level
    l_ctr   = data["left_arm_body_center"]     # left arm at chest level
    r_drop  = data["right_arm_drop_pose"]      # right arm ABOVE basket
    l_bskt  = data["left_basket_pose"]         # left arm HOLDS basket

    zeros_r = {k: 0.0 for k in RIGHT_KEYS}
    zeros_l = {k: 0.0 for k in LEFT_KEYS}

    center = {**r_ctr, **l_ctr}
    final  = {**r_drop, **l_bskt}

    print("Connecting to Isaac Sim shared memory...")
    arm_shm  = _connect_shm(ARM_SHM_NAME,     required=True)
    grip_shm = _connect_shm(GRIPPER_SHM_NAME, required=False)
    print()

    print("Starting in 3 seconds — watch Isaac Sim viewport...")
    time.sleep(3.0)

    try:
        # ── STEP 1: right arm reaches DOWN to okra ─────────────────────────────
        print("\n━━━ STEP 1: Right arm reaches DOWN to pick up okra")
        move(arm_shm, grip_shm,
             zeros_r, pick,
             zeros_l, l_ctr,
             GRIPPER_OPEN, "reach down")
        hold(arm_shm, grip_shm,
             {**pick, **l_ctr}, GRIPPER_OPEN, PAUSE_S)

        # ── STEP 2: gripper closes (grab okra) ────────────────────────────────
        print("\n━━━ STEP 2: Gripper CLOSES — grabbing okra")
        hold(arm_shm, grip_shm,
             {**pick, **l_ctr}, GRIPPER_CLOSED, 1.5)
        print("    gripper closed ✓")
        hold(arm_shm, grip_shm,
             {**pick, **l_ctr}, GRIPPER_CLOSED, PAUSE_S)

        # ── STEP 3: lift arm UP to body center ────────────────────────────────
        print("\n━━━ STEP 3: Right arm lifts UP — carrying okra")
        move(arm_shm, grip_shm,
             pick,  r_ctr,
             l_ctr, l_ctr,
             GRIPPER_CLOSED, "lift up")
        hold(arm_shm, grip_shm, center, GRIPPER_CLOSED, PAUSE_S)

        # ── STEP 4: left arm raises basket, right arm swings ABOVE basket ─────
        print("\n━━━ STEP 4: Both arms move — right ABOVE basket, left HOLDS basket")
        move(arm_shm, grip_shm,
             r_ctr, r_drop,
             l_ctr, l_bskt,
             GRIPPER_CLOSED, "position above basket")
        hold(arm_shm, grip_shm, final, GRIPPER_CLOSED, PAUSE_S)

        # ── STEP 5: gripper opens — okra drops ────────────────────────────────
        print("\n━━━ STEP 5: Gripper OPENS — okra drops into basket!")
        hold(arm_shm, grip_shm, final, GRIPPER_OPEN, 2.0)
        print("    gripper open ✓  okra dropped!")
        hold(arm_shm, grip_shm, final, GRIPPER_OPEN, PAUSE_S)

        # ── STEP 6: return to rest ─────────────────────────────────────────────
        print("\n━━━ STEP 6: Both arms return to rest")
        move(arm_shm, grip_shm,
             r_drop, zeros_r,
             l_bskt, zeros_l,
             GRIPPER_OPEN, "return to rest")

        print("\n✓ Demo complete — full okra harvest motion shown")

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        arm_shm.close()
        if grip_shm:
            grip_shm.close()


if __name__ == "__main__":
    main()
