#!/usr/bin/env python3
"""
drop_to_basket_real.py — G1 okra harvest: 9-phase arm motion on the REAL robot.

Phase 0 : Right gripper OPENS → user places okra → RIGHT gripper CLOSES
Phase 1 : Left arm → basket (L-shape, recorded position)
Phase 2 : Right arm → approach (safe intermediate)
Phase 3 : Right arm → drop position above basket
Phase 4 : Right gripper OPENS → okra drops → 3 s wait
Phase 5 : Right gripper CLOSES
Phase 6 : Right arm retreats to approach position
Phase 7 : Right arm parks (tucks to right)
Phase 8 : Left arm returns SLOWLY via safe waypoint (avoid leg)
Phase 9 : Right arm returns to natural

Prerequisites:
  - dex1_1_gripper_server must be running on robot PC 192.168.123.164:
      ssh unitree@192.168.123.164
      sudo /path/to/dex1_1_gripper_server --network eth0

Safety:
  - Reads actual joint positions from rt/lowstate before moving.
  - Smooth cosine interpolation (ArmInterpolator).
  - Per-joint speed limit; left arm return uses half speed.
  - Interactive confirmation before each phase.
  - Ctrl-C releases arm_sdk at any time.

Usage:
    conda activate unitree_sim_env
    python drop_to_basket_real.py --iface enp8s0
"""
from __future__ import annotations

import argparse
import json
import sys
import threading
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
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorCmd_
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
kNotUsedJoint = 29

WAIST_IDX = [12, 13, 14]

RIGHT_KEYS = [k for k in ARM_IDX if k.startswith("right_")]
LEFT_KEYS  = [k for k in ARM_IDX if k.startswith("left_")]

# ── Safety / motion limits ─────────────────────────────────────────────────────
SEND_RATE_HZ            = 50.0
MAX_SPEED_RAD_S         = 0.4   # conservative for real robot
LEFT_RETURN_SPEED_RAD_S = 0.2   # extra slow for left arm return (avoid leg)
SAFE_STOP_SPEED_RAD_S   = 0.3   # speed used during emergency safe stop
MIN_DURATION_S          = 3.0

KP_SHOULDER_ELBOW = 80.0
KP_WRIST          = 40.0
KD_ARM            = 3.0
KP_WAIST          = 300.0
KD_WAIST          = 3.0

WRIST_INDICES = {26, 27, 28, 19, 20, 21}

# ── Gripper constants ──────────────────────────────────────────────────────────
GRIPPER_RATE_HZ = 200.0
GRIPPER_Q_OPEN  = 5.2    # rad (safe open, measured)
GRIPPER_Q_GRIP  = 4.8    # rad (gentle grip — holds okra without squeezing)
GRIPPER_Q_CLOSE = 4.4    # rad (fully closed — used after drop, okra already released)
GRIPPER_KP      = 5.0
GRIPPER_KD      = 0.05

# ── Poses ──────────────────────────────────────────────────────────────────────

# Left arm basket position — recorded from real robot 20260625 (DO NOT CHANGE)
LEFT_ARM_L_SHAPE: dict[str, float] = {k: 0.0 for k in ARM_IDX}
LEFT_ARM_L_SHAPE.update({
    "left_shoulder_pitch": -0.1170,
    "left_shoulder_roll":  -0.0167,
    "left_shoulder_yaw":   -0.3997,
    "left_elbow":           1.1330,
    "left_wrist_roll":      0.0834,
    "left_wrist_pitch":    -1.0673,
    "left_wrist_yaw":      -0.2355,
})

# Right arm drop position — recorded from real robot (DO NOT CHANGE)
RIGHT_ARM_DROP_POSE: dict[str, float] = {k: 0.0 for k in ARM_IDX}
RIGHT_ARM_DROP_POSE.update({
    "right_shoulder_pitch": -0.0556,
    "right_shoulder_roll":  -0.1400,
    "right_shoulder_yaw":   -0.0624,
    "right_elbow":          -0.4139,
    "right_wrist_roll":     -0.6738,
    "right_wrist_pitch":    -0.8083,
    "right_wrist_yaw":       0.9169,
})

# Right arm approach — safe intermediate, avoids left arm (from MuJoCo)
RIGHT_ARM_APPROACH: dict[str, float] = {k: 0.0 for k in ARM_IDX}
RIGHT_ARM_APPROACH.update({
    "right_shoulder_pitch": -0.70,
    "right_shoulder_roll":  -0.60,
    "right_shoulder_yaw":    0.00,
    "right_elbow":           0.60,
})

# Right arm park — tucked to the right after drop
RIGHT_ARM_PARK: dict[str, float] = {k: 0.0 for k in ARM_IDX}
RIGHT_ARM_PARK.update({
    "right_shoulder_pitch": -0.30,
    "right_shoulder_roll":  -0.70,
    "right_elbow":           1.20,
})

# Left arm safe return waypoint — de-rotate shoulder_yaw, start straightening elbow
LEFT_RETURN_WAYPOINT: dict[str, float] = {k: 0.0 for k in ARM_IDX}
LEFT_RETURN_WAYPOINT.update({
    "left_shoulder_pitch":  0.00,
    "left_shoulder_roll":  -0.20,
    "left_shoulder_yaw":    0.00,  # de-rotated from -0.3997
    "left_elbow":           0.30,  # partially straightened from 1.133
})

# Left arm safe rest position — shoulder_roll +0.30 keeps arm away from leg
# Used as start AND final position so Ctrl+C stop is always safe
LEFT_ARM_REST_POSE: dict[str, float] = {k: 0.0 for k in ARM_IDX}
LEFT_ARM_REST_POSE["left_shoulder_roll"] = 0.60

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


# ── Arm command helpers ────────────────────────────────────────────────────────

def _build_cmd(pose: dict[str, float], mode_machine: int,
               waist_q: list[float]) -> unitree_hg_msg_dds__LowCmd_:
    cmd = unitree_hg_msg_dds__LowCmd_()
    cmd.mode_pr = 0
    cmd.mode_machine = mode_machine
    cmd.motor_cmd[kNotUsedJoint].q = 1.0

    for i, widx in enumerate(WAIST_IDX):
        m = cmd.motor_cmd[widx]
        m.mode = 1
        m.q    = waist_q[i]
        m.dq   = 0.0
        m.tau  = 0.0
        m.kp   = KP_WAIST
        m.kd   = KD_WAIST

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
    cmd.motor_cmd[kNotUsedJoint].q = 0.0
    for _ in range(30):
        cmd.crc = crc.Crc(cmd)
        pub.Write(cmd)
        time.sleep(0.02)
    print("arm_sdk released.")


def _run_one_arm(
    pub: ChannelPublisher,
    crc: CRC,
    current_pose: dict[str, float],
    goal_pose: dict[str, float],
    hold_pose: dict[str, float],
    keys: list[str],
    mode_machine: int,
    label: str,
    waist_q: list[float],
    max_speed: float = MAX_SPEED_RAD_S,
) -> None:
    """Move one arm while the other holds its current position."""
    moving_start = {k: current_pose[k] for k in keys}
    moving_goal  = {k: goal_pose[k]    for k in keys}
    still_pose   = {k: hold_pose[k]    for k in ARM_IDX if k not in keys}

    max_delta  = max(abs(moving_goal[k] - moving_start[k]) for k in keys)
    duration_s = max(MIN_DURATION_S, max_delta / max_speed)
    interp     = ArmInterpolator(moving_start, moving_goal, duration_s, round(SEND_RATE_HZ))

    period = 1.0 / SEND_RATE_HZ
    print(f"  {label}: {duration_s:.1f}s ({interp.total_steps} steps)")
    for step in range(interp.total_steps + 1):
        t0   = time.perf_counter()
        pose = {**still_pose, **interp.get_target(step)}
        cmd  = _build_cmd(pose, mode_machine, waist_q)
        _send(pub, crc, cmd)
        elapsed = time.perf_counter() - t0
        if elapsed < period:
            time.sleep(period - elapsed)


# ── Gripper helpers ────────────────────────────────────────────────────────────

def _make_gripper_cmd(q: float) -> MotorCmds_:
    cmd = MotorCmds_()
    cmd.cmds = [MotorCmd_(mode=1, q=q, dq=0.0, tau=0.0,
                          kp=GRIPPER_KP, kd=GRIPPER_KD, reserve=[0, 0, 0])]
    return cmd


def _gripper_set(gripper_pub: ChannelPublisher, q: float, duration_s: float) -> None:
    """Send gripper position command at 200 Hz for duration_s seconds."""
    cmd    = _make_gripper_cmd(q)
    period = 1.0 / GRIPPER_RATE_HZ
    until  = time.monotonic() + duration_s
    while time.monotonic() < until:
        t0 = time.perf_counter()
        gripper_pub.Write(cmd)
        elapsed = time.perf_counter() - t0
        if elapsed < period:
            time.sleep(period - elapsed)


def _gripper_hold_while_confirm(
    gripper_pub: ChannelPublisher, q: float, msg: str, yes: bool
) -> None:
    """Keep gripper at q (200 Hz background thread) while waiting for user confirmation."""
    stop_evt = threading.Event()
    cmd = _make_gripper_cmd(q)

    def _send_loop() -> None:
        period = 1.0 / GRIPPER_RATE_HZ
        while not stop_evt.is_set():
            t0 = time.perf_counter()
            gripper_pub.Write(cmd)
            elapsed = time.perf_counter() - t0
            if elapsed < period:
                time.sleep(period - elapsed)

    t = threading.Thread(target=_send_loop, daemon=True)
    t.start()
    try:
        confirm(msg, yes)
    finally:
        stop_evt.set()
        t.join(timeout=2.0)


# ── Pose loading ───────────────────────────────────────────────────────────────

def load_poses(path: Path):
    data = json.loads(path.read_text())
    RIGHT_ARM_DROP_POSE   = data["right_arm_drop_pose"]
    left_basket  = data["left_basket_pose"]
    right_center = data["right_arm_body_center"]
    left_center  = data["left_arm_body_center"]
    return RIGHT_ARM_DROP_POSE, left_basket, right_center, left_center


# ── Safe stop ─────────────────────────────────────────────────────────────────

def _safe_stop(
    pub: ChannelPublisher,
    crc: CRC,
    reader: LowStateReader,
    mode_machine: int,
    waist_q: list[float],
) -> None:
    """Read current arm position and smoothly move both arms to safe rest before release."""
    print("\n[SAFE STOP] Moving arms to safe rest position — do not touch the robot...")

    if reader.low_state is None:
        print("[SAFE STOP] No robot state — skipping safe stop.")
        return

    current_q = read_arm_q(reader.low_state)

    # Safe rest: left shoulder_roll +0.30 keeps arm away from leg, right arm natural
    target = {k: 0.0 for k in ARM_IDX}
    target["left_shoulder_roll"] = LEFT_ARM_REST_POSE["left_shoulder_roll"]

    max_delta  = max(abs(target[k] - current_q[k]) for k in ARM_IDX)
    duration_s = max(MIN_DURATION_S, max_delta / SAFE_STOP_SPEED_RAD_S)

    left_interp  = ArmInterpolator(
        {k: current_q[k] for k in LEFT_KEYS},
        {k: target[k]    for k in LEFT_KEYS},
        duration_s, round(SEND_RATE_HZ),
    )
    right_interp = ArmInterpolator(
        {k: current_q[k] for k in RIGHT_KEYS},
        {k: target[k]    for k in RIGHT_KEYS},
        duration_s, round(SEND_RATE_HZ),
    )

    period = 1.0 / SEND_RATE_HZ
    print(f"[SAFE STOP] Duration: {duration_s:.1f}s")
    for step in range(left_interp.total_steps + 1):
        t0   = time.perf_counter()
        pose = {}
        pose.update(left_interp.get_target(step))
        pose.update(right_interp.get_target(step))
        cmd  = _build_cmd(pose, mode_machine, waist_q)
        _send(pub, crc, cmd)
        elapsed = time.perf_counter() - t0
        if elapsed < period:
            time.sleep(period - elapsed)

    rest_pose = {**left_interp.get_target(left_interp.total_steps),
                 **right_interp.get_target(right_interp.total_steps)}
    _hold(pub, crc, rest_pose, mode_machine, 1.0, waist_q)
    print("[SAFE STOP] Arms at safe rest position.")


# ── Main ───────────────────────────────────────────────────────────────────────

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Move G1 arms for okra drop on real robot.")
    p.add_argument("--iface",     default="enp8s0", help="DDS network interface (robot cable)")
    p.add_argument("--domain",    type=int, default=0)
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

    print(f"\nConnecting to robot on iface={args.iface} domain={args.domain}...")
    ChannelFactoryInitialize(args.domain, args.iface)

    reader = LowStateReader()
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(reader.callback, 10)

    pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
    pub.Init()
    crc = CRC()

    print("\nInitializing right gripper publisher (rt/dex1/right/cmd)...")
    print("  NOTE: dex1_1_gripper_server must be running on 192.168.123.164")
    gripper_pub = ChannelPublisher("rt/dex1/right/cmd", MotorCmds_)
    gripper_pub.Init()

    print("\nWaiting for LowState from robot...")
    low_state    = wait_for_lowstate(reader)
    mode_machine = int(getattr(low_state, "mode_machine", 0))
    current_q    = read_arm_q(low_state)
    waist_q      = read_waist_q(low_state)

    print(f"mode_machine={mode_machine}")
    print(f"Waist locked: {[f'{v:.3f}' for v in waist_q]}")
    print("Current arm positions:")
    for name, val in current_q.items():
        print(f"  {name:24s} = {val: .4f} rad")

    normal_pose = {k: 0.0 for k in ARM_IDX}

    try:
        # ── Phase 0: gripper open → user places okra → gripper close ──────────
        print("\n" + "="*60)
        print("PHASE 0: Okra loading")
        print("="*60)
        print("→ Opening right gripper...")
        _gripper_set(gripper_pub, GRIPPER_Q_OPEN, 2.0)
        print("  Gripper OPEN (q=5.2 rad)")

        _gripper_hold_while_confirm(
            gripper_pub, GRIPPER_Q_OPEN,
            "\n[PHASE 0] Gripper is open. Place okra into the right hand.",
            args.yes,
        )

        print("\n→ Closing gripper gently to hold okra (not fully closed)...")
        _gripper_set(gripper_pub, GRIPPER_Q_GRIP, 2.0)
        print("  Gripper GRIP (q=4.8 rad) — okra held without squeezing.")

        confirm(
            "\n[START] Okra loaded. Ready to begin arm motion. Type YES to start.",
            args.yes,
        )

        # ── Phase 1: left arm → basket (L-shape) ──────────────────────────────
        print("\n" + "="*60)
        print("PHASE 1: Left arm → basket (L-shape)")
        print("="*60)
        _run_one_arm(pub, crc, current_q, LEFT_ARM_L_SHAPE, current_q,
                     LEFT_KEYS, mode_machine, "left arm → basket", waist_q)
        left_at_basket = {**current_q, **{k: LEFT_ARM_L_SHAPE[k] for k in LEFT_KEYS}}
        _hold(pub, crc, left_at_basket, mode_machine, 1.0, waist_q)

        confirm("\n[PHASE 2] Right arm moves to approach position.", args.yes)

        # ── Phase 2: right arm → approach ─────────────────────────────────────
        print("\n" + "="*60)
        print("PHASE 2: Right arm → approach (safe intermediate)")
        print("="*60)
        _run_one_arm(pub, crc, left_at_basket, RIGHT_ARM_APPROACH, left_at_basket,
                     RIGHT_KEYS, mode_machine, "right arm → approach", waist_q)
        at_approach = {**left_at_basket, **{k: RIGHT_ARM_APPROACH[k] for k in RIGHT_KEYS}}
        _hold(pub, crc, at_approach, mode_machine, 0.5, waist_q)

        confirm("\n[PHASE 3] Right arm moves to drop position above basket.", args.yes)

        # ── Phase 3: right arm → drop position ────────────────────────────────
        print("\n" + "="*60)
        print("PHASE 3: Right arm → drop position")
        print("="*60)
        _run_one_arm(pub, crc, at_approach, RIGHT_ARM_DROP_POSE, at_approach,
                     RIGHT_KEYS, mode_machine, "right arm → drop", waist_q)
        at_drop = {**at_approach, **{k: RIGHT_ARM_DROP_POSE[k] for k in RIGHT_KEYS}}
        _hold(pub, crc, at_drop, mode_machine, 1.0, waist_q)

        confirm("\n[PHASE 4] OPEN right gripper — okra drops into basket.", args.yes)

        # ── Phase 4: gripper opens → okra drops ───────────────────────────────
        print("\n" + "="*60)
        print("PHASE 4: Right gripper opens — okra drops")
        print("="*60)
        print("→ Opening gripper (okra drops into basket)...")
        _gripper_set(gripper_pub, GRIPPER_Q_OPEN, 3.0)
        print("  Gripper OPEN. Okra should be in basket.")
        _hold(pub, crc, at_drop, mode_machine, 0.5, waist_q)

        # ── Phase 5: gripper closes ────────────────────────────────────────────
        print("\n" + "="*60)
        print("PHASE 5: Right gripper closes")
        print("="*60)
        print("→ Closing gripper...")
        _gripper_set(gripper_pub, GRIPPER_Q_CLOSE, 1.5)
        print("  Gripper CLOSED.")
        _hold(pub, crc, at_drop, mode_machine, 0.5, waist_q)

        confirm("\n[PHASE 6] Right arm retreats to approach position.", args.yes)

        # ── Phase 6: right arm retreats to approach ────────────────────────────
        print("\n" + "="*60)
        print("PHASE 6: Right arm → approach (retreat)")
        print("="*60)
        _run_one_arm(pub, crc, at_drop, RIGHT_ARM_APPROACH, at_drop,
                     RIGHT_KEYS, mode_machine, "right arm → approach", waist_q)
        at_retreat = {**at_drop, **{k: RIGHT_ARM_APPROACH[k] for k in RIGHT_KEYS}}
        _hold(pub, crc, at_retreat, mode_machine, 0.5, waist_q)

        confirm("\n[PHASE 7] Right arm parks (tucks to right side).", args.yes)

        # ── Phase 7: right arm → park ──────────────────────────────────────────
        print("\n" + "="*60)
        print("PHASE 7: Right arm → park")
        print("="*60)
        _run_one_arm(pub, crc, at_retreat, RIGHT_ARM_PARK, at_retreat,
                     RIGHT_KEYS, mode_machine, "right arm → park", waist_q)
        at_park = {**at_retreat, **{k: RIGHT_ARM_PARK[k] for k in RIGHT_KEYS}}
        _hold(pub, crc, at_park, mode_machine, 0.5, waist_q)

        confirm("\n[PHASE 8] Left arm returns SLOWLY (safe 2-step path).", args.yes)

        # ── Phase 8: left arm returns via safe waypoint ────────────────────────
        print("\n" + "="*60)
        print("PHASE 8: Left arm returns slowly (via safe waypoint)")
        print("="*60)
        print("→ Phase 8a: de-rotate shoulder, start straightening elbow...")
        _run_one_arm(pub, crc, at_park, LEFT_RETURN_WAYPOINT, at_park,
                     LEFT_KEYS, mode_machine, "left arm → waypoint", waist_q,
                     max_speed=LEFT_RETURN_SPEED_RAD_S)
        at_waypoint = {**at_park, **{k: LEFT_RETURN_WAYPOINT[k] for k in LEFT_KEYS}}
        _hold(pub, crc, at_waypoint, mode_machine, 0.5, waist_q)

        print("→ Phase 8b: left arm to safe rest position (arm away from leg)...")
        _run_one_arm(pub, crc, at_waypoint, LEFT_ARM_REST_POSE, at_waypoint,
                     LEFT_KEYS, mode_machine, "left arm → rest", waist_q,
                     max_speed=LEFT_RETURN_SPEED_RAD_S)
        left_returned = {**at_waypoint, **{k: LEFT_ARM_REST_POSE[k] for k in LEFT_KEYS}}
        _hold(pub, crc, left_returned, mode_machine, 0.5, waist_q)

        confirm("\n[PHASE 9] Right arm returns to natural position.", args.yes)

        # ── Phase 9: right arm returns to natural ──────────────────────────────
        print("\n" + "="*60)
        print("PHASE 9: Right arm → natural")
        print("="*60)
        _run_one_arm(pub, crc, left_returned, normal_pose, left_returned,
                     RIGHT_KEYS, mode_machine, "right arm → natural", waist_q)

        print("\n" + "="*60)
        print("All phases complete! Okra harvested.")
        print("="*60)

    except KeyboardInterrupt:
        print("\nStopped by user.")
        try:
            _safe_stop(pub, crc, reader, mode_machine, waist_q)
        except (KeyboardInterrupt, Exception) as e:
            print(f"[SAFE STOP] Interrupted during safe stop: {e}")
    finally:
        release_arm_sdk(pub, crc, mode_machine=mode_machine)


if __name__ == "__main__":
    main()
