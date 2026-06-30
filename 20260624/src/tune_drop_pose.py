#!/usr/bin/env python3
"""
tune_drop_pose.py — Interactively adjust the right arm drop pose in Isaac Sim.

Holds the left arm (basket) in place and lets you tweak the right arm joints
live so the right hand aligns above the basket.  Save when satisfied.

Usage:
    # Isaac Sim must be running first
    cd /home/techshare/user/yokote/20260624/src
    conda activate unitree_sim_env
    python tune_drop_pose.py
"""
from __future__ import annotations

import json
import time
import sys
from pathlib import Path
from multiprocessing import shared_memory

POSE_JSON        = Path(__file__).resolve().parent / "data" / "mujoco_right_arm_drop_pose.json"
SHM_NAME         = "dds_robot_cmd"
GRIPPER_SHM_NAME = "isaac_gripper_cmd"
NUM_MOTORS       = 35
GRIPPER_CLOSED   = 0.024
GRIPPER_OPEN     = -0.02

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
ALL_KEYS   = RIGHT_KEYS + LEFT_KEYS

# Short aliases so user can type less
ALIASES: dict[str, str] = {
    # right arm
    "rsp":  "right_shoulder_pitch",
    "rsr":  "right_shoulder_roll",
    "rsy":  "right_shoulder_yaw",
    "re":   "right_elbow",
    "rwr":  "right_wrist_roll",
    "rwp":  "right_wrist_pitch",
    "rwy":  "right_wrist_yaw",
    # left arm (basket)
    "lsp":  "left_shoulder_pitch",
    "lsr":  "left_shoulder_roll",
    "lsy":  "left_shoulder_yaw",
    "le":   "left_elbow",
    "lwr":  "left_wrist_roll",
    "lwp":  "left_wrist_pitch",
    "lwy":  "left_wrist_yaw",
}

STEP_DEFAULT = 0.05  # radians

def _write_gripper(shm, right_pos: float, left_pos: float = GRIPPER_CLOSED) -> None:
    if shm is None:
        return
    cmd = {
        "right_gripper_cmd": {"positions": [right_pos], "velocities": [0.0],
                              "torques": [0.0], "kp": [50.0], "kd": [2.0]},
        "left_gripper_cmd":  {"positions": [left_pos],  "velocities": [0.0],
                              "torques": [0.0], "kp": [50.0], "kd": [2.0]},
    }
    blob = json.dumps(cmd).encode()
    ts   = int(time.time()) & 0xFFFFFFFF
    shm.buf[0:4] = ts.to_bytes(4, "little")
    shm.buf[4:8] = len(blob).to_bytes(4, "little")
    shm.buf[8:8 + len(blob)] = blob


def _write_shm(shm, pose: dict[str, float]) -> None:
    positions  = [0.0] * NUM_MOTORS
    kp         = [0.0] * NUM_MOTORS
    kd         = [0.0] * NUM_MOTORS
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
    shm.buf[0:4] = ts.to_bytes(4, "little")
    shm.buf[4:8] = len(blob).to_bytes(4, "little")
    shm.buf[8:8 + len(blob)] = blob


def _print_pose(pose: dict[str, float]) -> None:
    print("\nRight arm (drop):")
    for k in RIGHT_KEYS:
        alias = next((a for a, full in ALIASES.items() if full == k), "")
        print(f"  {k:24s} ({alias:3s}) = {pose[k]: .4f} rad")
    print("Left arm (basket):")
    for k in LEFT_KEYS:
        alias = next((a for a, full in ALIASES.items() if full == k), "")
        print(f"  {k:24s} ({alias:3s}) = {pose[k]: .4f} rad")


def _print_help() -> None:
    print("""
Commands:
  <joint> [+/-] [step]   — adjust a joint.  step defaults to 0.05 rad
  Examples:
    lsr -         → left_shoulder_roll  -0.05 rad  (basket moves RIGHT)
    lsr +         → left_shoulder_roll  +0.05 rad  (basket moves LEFT)
    rsr +         → right_shoulder_roll +0.05 rad
    rsp - 0.1     → right_shoulder_pitch -0.10 rad
    le + 0.2      → right_elbow +0.2 rad

  Left arm aliases (basket):   lsp  lsr  lsy  le  lwr  lwp  lwy
  Right arm aliases (drop):    rsp  rsr  rsy  re  rwr  rwp  rwy

  Modes:
    drop          — show DROP pose (right arm above basket)
    pick          — show PICK pose (right arm at okra location)

  Gripper:
    open          — open right gripper in Isaac Sim
    close         — close right gripper in Isaac Sim

  p               — print current pose
  save            — save ALL poses to JSON and exit
  reset           — reload original pose from JSON
  q / quit        — exit WITHOUT saving
  help            — show this help
""")


def main() -> None:
    # Load poses
    data = json.loads(POSE_JSON.read_text())
    left_basket = {k: float(data["left_basket_pose"][k])    for k in LEFT_KEYS}
    right_drop  = {k: float(data["right_arm_drop_pose"][k]) for k in RIGHT_KEYS}
    right_pick  = {k: float(data["right_arm_pick_pose"][k]) for k in RIGHT_KEYS}

    # Active mode: "drop" or "pick"
    mode = "drop"
    pose: dict[str, float] = {**left_basket, **right_drop}

    # Connect to arm shared memory
    try:
        shm = shared_memory.SharedMemory(name=SHM_NAME)
        print(f"Connected arm shm   : {SHM_NAME}")
    except FileNotFoundError:
        print(f"ERROR: '{SHM_NAME}' not found. Start Isaac Sim first.")
        sys.exit(1)

    # Connect to gripper shared memory (optional)
    try:
        grip_shm = shared_memory.SharedMemory(name=GRIPPER_SHM_NAME)
        print(f"Connected gripper shm: {GRIPPER_SHM_NAME}")
    except FileNotFoundError:
        print(f"WARNING: '{GRIPPER_SHM_NAME}' not found — gripper commands disabled")
        grip_shm = None

    # Send initial pose
    _write_shm(shm, pose)
    _write_gripper(grip_shm, GRIPPER_CLOSED)
    print("\nMode: DROP  (type 'pick' to switch to pick pose tuning)")
    _print_pose(pose)
    _print_help()

    try:
        while True:
            try:
                raw = input(">>> ").strip()
            except (EOFError, KeyboardInterrupt):
                print("\nExiting without saving.")
                break

            if not raw:
                continue
            tokens = raw.split()
            cmd0 = tokens[0].lower()

            if cmd0 in ("q", "quit"):
                print("Exiting without saving.")
                break

            if cmd0 == "help":
                _print_help()
                continue

            if cmd0 == "p":
                _print_pose(pose)
                continue

            if cmd0 == "open":
                _write_gripper(grip_shm, GRIPPER_OPEN)
                print("  right gripper OPEN")
                continue

            if cmd0 == "close":
                _write_gripper(grip_shm, GRIPPER_CLOSED)
                print("  right gripper CLOSED")
                continue

            if cmd0 == "drop":
                mode = "drop"
                pose.update(right_drop)
                _write_shm(shm, pose)
                print("Mode: DROP — editing right arm drop pose")
                _print_pose(pose)
                continue

            if cmd0 == "pick":
                mode = "pick"
                pose.update(right_pick)
                _write_shm(shm, pose)
                print("Mode: PICK — editing right arm pick pose")
                _print_pose(pose)
                continue

            if cmd0 == "reset":
                data2 = json.loads(POSE_JSON.read_text())
                right_drop = {k: float(data2["right_arm_drop_pose"][k]) for k in RIGHT_KEYS}
                right_pick = {k: float(data2["right_arm_pick_pose"][k]) for k in RIGHT_KEYS}
                pose.update({k: float(data2["left_basket_pose"][k]) for k in LEFT_KEYS})
                pose.update(right_drop if mode == "drop" else right_pick)
                _write_shm(shm, pose)
                print("Reset to original JSON pose.")
                _print_pose(pose)
                continue

            if cmd0 == "save":
                original = json.loads(POSE_JSON.read_text())
                for k in LEFT_KEYS:
                    original["left_basket_pose"][k] = round(pose[k], 6)
                for k in RIGHT_KEYS:
                    original["right_arm_drop_pose"][k] = round(right_drop[k], 6)
                    original["right_arm_pick_pose"][k] = round(right_pick[k], 6)
                POSE_JSON.write_text(json.dumps(original, indent=2))
                print(f"Saved to {POSE_JSON}")
                _print_pose(pose)
                break

            # Joint adjustment: <joint_name_or_alias> [+/-] [step]
            joint_raw = ALIASES.get(cmd0, cmd0)
            if joint_raw not in ALL_KEYS:
                print(f"Unknown joint '{cmd0}'. Type 'help' for commands.")
                continue

            # Parse sign and optional step
            sign = +1.0
            step = STEP_DEFAULT
            if len(tokens) >= 2:
                sign_tok = tokens[1]
                if sign_tok == "-":
                    sign = -1.0
                elif sign_tok != "+":
                    try:
                        step = float(sign_tok)
                    except ValueError:
                        print(f"Bad value '{sign_tok}'")
                        continue
            if len(tokens) >= 3:
                try:
                    step = float(tokens[2])
                except ValueError:
                    print(f"Bad step '{tokens[2]}'")
                    continue

            pose[joint_raw] = round(pose[joint_raw] + sign * step, 6)
            # keep the active right-arm dict in sync
            if joint_raw in RIGHT_KEYS:
                if mode == "drop":
                    right_drop[joint_raw] = pose[joint_raw]
                else:
                    right_pick[joint_raw] = pose[joint_raw]
            _write_shm(shm, pose)
            print(f"  {joint_raw} = {pose[joint_raw]: .4f} rad")

    finally:
        shm.close()
        if grip_shm:
            grip_shm.close()


if __name__ == "__main__":
    main()
