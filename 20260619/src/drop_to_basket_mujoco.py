from __future__ import annotations

import argparse
import json
import signal
import time
from dataclasses import dataclass
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

from arm_interpolator import ArmInterpolator


DEFAULT_SCENE_XML = "/home/techshare/drl_kit/mujoco_ws/ts_mujoco-main/unitree_robots/g1/scene_29dof_with_dex1_gripper.xml"
DEFAULT_POSE_JSON = Path(__file__).resolve().parent / "data" / "mujoco_right_arm_drop_pose.json"

SIMULATE_DT = 0.005
VIEWER_DT = 0.02
WAIT_BEFORE_MOVE_S = 3.0
MIN_MOVE_DURATION_S = 1.5
MAX_JOINT_SPEED_RAD_S  = 0.5
SAFE_STOP_SPEED_RAD_S  = 0.3   # speed used when Ctrl+C safe-stop fires
GRIPPER_OPEN_DURATION_S     = 0.8
GRIPPER_CLOSE_DURATION_S    = 0.8
GRIPPER_HOLD_OPEN_LOAD_S    = 1.5   # hold open: "place okra into hand"
GRIPPER_HOLD_CLOSED_S       = 1.0   # hold closed before arm motion starts
GRIPPER_HOLD_AT_BASKET_S    = 2.0   # hold gripper closed at basket before opening to drop
GRIPPER_HOLD_OPEN_DROP_S    = 1.5   # hold open: "okra falling into basket"
HOLD_AFTER_DONE_S = 1.0
FREE_BASE_QPOS = slice(0, 7)
FREE_BASE_QVEL = slice(0, 6)
SAFE_START_MAX_TRIES = 200
DROP_HEIGHT_ABOVE_LEFT_HAND_M = 0.30
RIGHT_DROP_Y_OFFSET_M = -0.05
LEFT_BASKET_TARGET_REL_PELVIS = np.array([0.32,  0.03, -0.07])
RIGHT_DROP_ELBOW_TARGET_REL_PELVIS = np.array([0.18, -0.18, 0.10])
RIGHT_DROP_ELBOW_TASK_WEIGHT = 0.7
RIGHT_DROP_MIN_ELBOW_X_REL_PELVIS = 0.10
LEFT_BASKET_ELBOW_TARGET_REL_PELVIS = np.array([0.20, 0.18, 0.12])
LEFT_BASKET_ELBOW_TASK_WEIGHT = 0.4
RIGHT_HARVEST_TARGET_X_RANGE = (0.36, 0.58)
RIGHT_HARVEST_TARGET_Y_RANGE = (-0.38, 0.10)
RIGHT_HARVEST_TARGET_Z_RANGE = (-0.10, 0.20)
RIGHT_HARVEST_MAX_ELBOW_RAD = 1.05
RIGHT_HARVEST_MAX_IK_ERROR_M = 0.035
IK_MAX_ITERS = 300
IK_POS_TOLERANCE_M = 0.01
IK_DAMPING = 0.08
IK_MAX_STEP_RAD = 0.05

RIGHT_ARM_JOINTS = (
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
)

RIGHT_HAND_JOINTS = (
    "right_gripper_finger1_joint",
    "right_gripper_finger2_joint",
)

LEFT_ARM_JOINTS = (
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
)
LEFT_SHOULDER_ONLY_JOINTS = (
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
)

RIGHT_HAND_TARGET_BODY = "right_gripper_finger1_tip"
LEFT_HAND_TARGET_BODY = "left_basket"
RIGHT_ELBOW_TARGET_BODY = "right_elbow_link"
LEFT_ELBOW_TARGET_BODY = "left_elbow_link"

RIGHT_DROP_POSE_INITIAL_GUESS = {
    "right_shoulder_pitch_joint": 0.0,
    "right_shoulder_roll_joint": 0.3,
    "right_shoulder_yaw_joint": 0.0,
    "right_elbow_joint": 1.0,
    "right_wrist_roll_joint": 0.0,
    "right_wrist_pitch_joint": 0.0,
    "right_wrist_yaw_joint": 0.0,
}

RIGHT_HARVEST_POSE_INITIAL_GUESS = {
    "right_shoulder_pitch_joint": -0.25,
    "right_shoulder_roll_joint": 0.25,
    "right_shoulder_yaw_joint": 0.0,
    "right_elbow_joint": 0.35,
    "right_wrist_roll_joint": 0.0,
    "right_wrist_pitch_joint": 0.0,
    "right_wrist_yaw_joint": 0.0,
}

LEFT_BASKET_POSE_INITIAL_GUESS = {
    "left_shoulder_pitch_joint": 0.1,
    "left_shoulder_roll_joint": -0.3,
    "left_shoulder_yaw_joint": -0.2,
    "left_elbow_joint": 0.6,
    "left_wrist_roll_joint": 0.0,
    "left_wrist_pitch_joint": 0.0,
    "left_wrist_yaw_joint": 0.0,
}

# Safe resting pose for left arm — shoulder_roll +0.30 keeps arm away from leg
LEFT_ARM_REST_POSE = {j: 0.0 for j in LEFT_ARM_JOINTS}
LEFT_ARM_REST_POSE["left_shoulder_roll_joint"] = 0.60

# Safe park pose inserted BEFORE the arm returns to rest.
# Arm stretched out to the left (shoulder_roll=1.20, wrist Y≈0.324m, well outside leg).
# wrist_roll=-1.20 rotates basket opening upward so okra stays inside.
# From this pose the final swing to LEFT_ARM_BASKET_REST_POSE stays outside the leg.
LEFT_ARM_SAFE_PARK_POSE = {
    "left_shoulder_pitch_joint":  0.0,
    "left_shoulder_roll_joint":   1.20,
    "left_shoulder_yaw_joint":    0.0,
    "left_elbow_joint":           0.0,
    "left_wrist_roll_joint":     -1.20,
    "left_wrist_pitch_joint":     0.0,
    "left_wrist_yaw_joint":       0.0,
}

# Final rest pose for left arm when basket is attached.
# shoulder_roll=0.90 → wrist Y≈0.297m → 181mm from outer hip (satisfies 175mm clearance).
# wrist_roll=-0.20 keeps basket opening facing world-up so okra stays inside.
LEFT_ARM_BASKET_REST_POSE = {
    "left_shoulder_pitch_joint":  0.0,
    "left_shoulder_roll_joint":   0.90,
    "left_shoulder_yaw_joint":    0.0,
    "left_elbow_joint":           0.0,
    "left_wrist_roll_joint":     -0.20,
    "left_wrist_pitch_joint":     0.0,
    "left_wrist_yaw_joint":       0.0,
}

# Final rest pose for right arm — 175mm clearance from right leg.
# shoulder_roll=-0.90 → wrist Y≈-0.297m → 181mm from outer right hip.
RIGHT_ARM_NATURAL_REST_POSE = {
    "right_shoulder_pitch_joint":  0.0,
    "right_shoulder_roll_joint":  -0.90,
    "right_shoulder_yaw_joint":    0.0,
    "right_elbow_joint":           0.0,
    "right_wrist_roll_joint":      0.0,
    "right_wrist_pitch_joint":     0.0,
    "right_wrist_yaw_joint":       0.0,
}

# ============================================================
# LOCKED DROP POSITIONS — DO NOT MODIFY THESE VALUES
# Source: recorded from real robot on 2026-06-25 16:34:52
# File:   src/data/taught_poses/000_20260625_163452_pose_000.json
# These values are NOT loaded from any JSON file at runtime.
# Recording a new robot position will NOT change these values.
# To intentionally update, change both the values AND the
# _LOCKED_* checksums below, then justify the change in git.
# ============================================================

LEFT_ARM_L_SHAPE_POSE = {
    "left_shoulder_pitch_joint":  0.0037,
    "left_shoulder_roll_joint":   0.2527,
    "left_shoulder_yaw_joint":   -0.0478,
    "left_elbow_joint":           1.1991,
    "left_wrist_roll_joint":     -0.1714,
    "left_wrist_pitch_joint":    -1.0626,
    "left_wrist_yaw_joint":      -1.0144,
}

RECORDED_RIGHT_DROP_POSE = {
    "right_shoulder_pitch_joint": -0.11606722325086594,
    "right_shoulder_roll_joint":  -0.35777705907821655,
    "right_shoulder_yaw_joint":    0.5581882281249830,
    "right_elbow_joint":          -0.23972046375274658,
    "right_wrist_roll_joint":     -0.0300226437634435,
    "right_wrist_pitch_joint":    -0.2594746064536180,
    "right_wrist_yaw_joint":       0.8287091851234436,
}

# Checksums — automatically verified at startup
_LOCKED_LEFT_SUM  = round(sum(LEFT_ARM_L_SHAPE_POSE.values()), 6)
_LOCKED_RIGHT_SUM = 0.383835   # updated: +11 cm left +13 cm forward +5 cm up (2026-06-29)

def _assert_drop_poses_locked() -> None:
    left_sum  = round(sum(LEFT_ARM_L_SHAPE_POSE.values()), 6)
    right_sum = round(sum(RECORDED_RIGHT_DROP_POSE.values()), 6)
    if left_sum != _LOCKED_LEFT_SUM:
        raise RuntimeError(
            f"LEFT_ARM_L_SHAPE_POSE has been modified! "
            f"Expected checksum {_LOCKED_LEFT_SUM}, got {left_sum}. "
            "These are LOCKED recorded robot positions — do not edit them."
        )
    if right_sum != _LOCKED_RIGHT_SUM:
        raise RuntimeError(
            f"RECORDED_RIGHT_DROP_POSE has been modified! "
            f"Expected checksum {_LOCKED_RIGHT_SUM}, got {right_sum}. "
            "These are LOCKED recorded robot positions — do not edit them."
        )

_assert_drop_poses_locked()

FALLBACK_FRONT_START_POSE = {
    "right_shoulder_pitch_joint": -0.25,
    "right_shoulder_roll_joint": 0.25,
    "right_shoulder_yaw_joint": 0.0,
    "right_elbow_joint": 0.35,
    "right_wrist_roll_joint": 0.0,
    "right_wrist_pitch_joint": 0.0,
    "right_wrist_yaw_joint": 0.0,
}

RIGHT_HAND_CLOSED_POSE = {
    "right_gripper_finger1_joint": 0.020,
    "right_gripper_finger2_joint": 0.020,
}

RIGHT_HAND_OPEN_POSE = {
    "right_gripper_finger1_joint": -0.02,
    "right_gripper_finger2_joint": -0.02,
}


@dataclass(frozen=True)
class JointActuator:
    joint_name: str
    actuator_name: str
    joint_id: int
    actuator_id: int
    qposadr: int
    qveladr: int
    joint_range: tuple[float, float]
    ctrlrange: tuple[float, float]


def actuator_name_for_joint(joint_name: str) -> str:
    return joint_name.removesuffix("_joint")


def get_joint_actuator(model: mujoco.MjModel, joint_name: str) -> JointActuator:
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if joint_id < 0:
        raise ValueError(f"joint not found: {joint_name}")

    actuator_name = actuator_name_for_joint(joint_name)
    actuator_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name
    )
    if actuator_id < 0:
        raise ValueError(f"actuator not found: {actuator_name}")

    joint_range = tuple(float(v) for v in model.jnt_range[joint_id])
    ctrlrange = tuple(float(v) for v in model.actuator_ctrlrange[actuator_id])
    return JointActuator(
        joint_name=joint_name,
        actuator_name=actuator_name,
        joint_id=joint_id,
        actuator_id=actuator_id,
        qposadr=int(model.jnt_qposadr[joint_id]),
        qveladr=int(model.jnt_dofadr[joint_id]),
        joint_range=joint_range,
        ctrlrange=ctrlrange,
    )


def get_actuated_joint_names(model: mujoco.MjModel) -> list[str]:
    joint_names = []
    for actuator_id in range(model.nu):
        joint_id = int(model.actuator_trnid[actuator_id][0])
        joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        if joint_name is not None:
            joint_names.append(joint_name)
    return joint_names


def read_targets(
    data: mujoco.MjData, refs: dict[str, JointActuator], joint_names: tuple[str, ...] | list[str]
) -> dict[str, float]:
    return {name: float(data.qpos[refs[name].qposadr]) for name in joint_names}


def clamp_targets(
    targets: dict[str, float], refs: dict[str, JointActuator]
) -> dict[str, float]:
    return {
        name: float(np.clip(value, refs[name].joint_range[0], refs[name].joint_range[1]))
        for name, value in targets.items()
    }


def sample_right_arm_start(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    rng: np.random.Generator,
    refs: dict[str, JointActuator],
) -> dict[str, float]:
    original_qpos = data.qpos.copy()
    original_qvel = data.qvel.copy()
    pelvis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
    if pelvis_id < 0:
        raise ValueError("body not found: pelvis")
    try:
        for _ in range(SAFE_START_MAX_TRIES):
            mujoco.mj_forward(model, data)
            pelvis_pos = data.xpos[pelvis_id]
            target_pos = pelvis_pos + np.array(
                [
                    rng.uniform(*RIGHT_HARVEST_TARGET_X_RANGE),
                    rng.uniform(*RIGHT_HARVEST_TARGET_Y_RANGE),
                    rng.uniform(*RIGHT_HARVEST_TARGET_Z_RANGE),
                ]
            )
            start, _, ik_error_m = solve_arm_to_body_position(
                model,
                data,
                refs,
                RIGHT_ARM_JOINTS,
                RIGHT_HAND_TARGET_BODY,
                target_pos,
                clamp_targets(RIGHT_HARVEST_POSE_INITIAL_GUESS, refs),
            )
            if (
                ik_error_m > RIGHT_HARVEST_MAX_IK_ERROR_M
                or start["right_elbow_joint"] > RIGHT_HARVEST_MAX_ELBOW_RAD
            ):
                continue
            set_joint_positions(data, refs, start)
            mujoco.mj_forward(model, data)
            if is_safe_front_start(model, data):
                return start
        return dict(FALLBACK_FRONT_START_POSE)
    finally:
        data.qpos[:] = original_qpos
        data.qvel[:] = original_qvel
        mujoco.mj_forward(model, data)


def is_safe_front_start(model: mujoco.MjModel, data: mujoco.MjData) -> bool:
    pelvis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
    wrist_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "right_wrist_yaw_link")
    hand_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "right_hand_index_0_link")
    elbow_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "right_elbow_link")
    pelvis_pos = data.xpos[pelvis_id]
    wrist_rel = data.xpos[wrist_id] - pelvis_pos
    hand_rel = data.xpos[hand_id] - pelvis_pos
    elbow_rel = data.xpos[elbow_id] - pelvis_pos
    return (
        wrist_rel[0] >= 0.0
        and hand_rel[0] >= 0.05
        and -0.34 <= hand_rel[1] <= 0.22
        and hand_rel[2] >= -0.30
        and elbow_rel[1] <= -0.13
    )


def estimate_move_duration_s(
    start_angles: dict[str, float],
    goal_angles: dict[str, float],
    max_joint_speed_rad_s: float = MAX_JOINT_SPEED_RAD_S,
    min_duration_s: float = MIN_MOVE_DURATION_S,
) -> float:
    if max_joint_speed_rad_s <= 0:
        raise ValueError("max_joint_speed_rad_s must be positive")
    max_delta = max(abs(goal_angles[name] - start_angles[name]) for name in start_angles)
    return max(min_duration_s, max_delta / max_joint_speed_rad_s)


def solve_arm_to_body_position(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    refs: dict[str, JointActuator],
    joint_names: tuple[str, ...],
    body_name: str,
    target_pos: np.ndarray,
    initial_guess: dict[str, float],
) -> tuple[dict[str, float], np.ndarray, float]:
    original_qpos = data.qpos.copy()
    original_qvel = data.qvel.copy()
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    if body_id < 0:
        raise ValueError(f"body not found: {body_name}")

    try:
        set_joint_positions(data, refs, initial_guess)
        mujoco.mj_forward(model, data)

        jacp = np.zeros((3, model.nv))
        jacr = np.zeros((3, model.nv))
        dof_ids = np.array([refs[name].qveladr for name in joint_names], dtype=int)
        qpos_ids = [refs[name].qposadr for name in joint_names]

        for _ in range(IK_MAX_ITERS):
            mujoco.mj_forward(model, data)
            err = target_pos - data.xpos[body_id]
            if np.linalg.norm(err) <= IK_POS_TOLERANCE_M:
                break

            mujoco.mj_jacBody(model, data, jacp, jacr, body_id)
            jac = jacp[:, dof_ids]
            lhs = jac @ jac.T + (IK_DAMPING**2) * np.eye(3)
            step = jac.T @ np.linalg.solve(lhs, err)
            step = np.clip(step, -IK_MAX_STEP_RAD, IK_MAX_STEP_RAD)

            for qpos_id, joint_name, delta in zip(qpos_ids, joint_names, step):
                lower, upper = refs[joint_name].joint_range
                data.qpos[qpos_id] = np.clip(data.qpos[qpos_id] + delta, lower, upper)

        mujoco.mj_forward(model, data)
        solved = read_targets(data, refs, joint_names)
        final_pos = data.xpos[body_id].copy()
        error_m = float(np.linalg.norm(target_pos - final_pos))
        return solved, final_pos, error_m
    finally:
        data.qpos[:] = original_qpos
        data.qvel[:] = original_qvel
        mujoco.mj_forward(model, data)


def solve_arm_to_body_position_with_secondary(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    refs: dict[str, JointActuator],
    joint_names: tuple[str, ...],
    primary_body_name: str,
    primary_target_pos: np.ndarray,
    secondary_body_name: str,
    secondary_target_pos: np.ndarray,
    secondary_weight: float,
    initial_guess: dict[str, float],
    min_secondary_x: float | None = RIGHT_DROP_MIN_ELBOW_X_REL_PELVIS,
) -> tuple[dict[str, float], np.ndarray, np.ndarray, float, float]:
    original_qpos = data.qpos.copy()
    original_qvel = data.qvel.copy()
    primary_body_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, primary_body_name
    )
    secondary_body_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, secondary_body_name
    )
    if primary_body_id < 0:
        raise ValueError(f"body not found: {primary_body_name}")
    if secondary_body_id < 0:
        raise ValueError(f"body not found: {secondary_body_name}")

    try:
        set_joint_positions(data, refs, initial_guess)
        mujoco.mj_forward(model, data)

        jacp = np.zeros((3, model.nv))
        jacr = np.zeros((3, model.nv))
        dof_ids = np.array([refs[name].qveladr for name in joint_names], dtype=int)
        qpos_ids = [refs[name].qposadr for name in joint_names]

        for _ in range(IK_MAX_ITERS):
            mujoco.mj_forward(model, data)
            primary_err = primary_target_pos - data.xpos[primary_body_id]
            secondary_err = secondary_target_pos - data.xpos[secondary_body_id]
            if (
                np.linalg.norm(primary_err) <= IK_POS_TOLERANCE_M
                and (min_secondary_x is None or data.xpos[secondary_body_id][0] >= min_secondary_x)
            ):
                break

            mujoco.mj_jacBody(model, data, jacp, jacr, primary_body_id)
            primary_jac = jacp[:, dof_ids]
            mujoco.mj_jacBody(model, data, jacp, jacr, secondary_body_id)
            secondary_jac = jacp[:, dof_ids]
            jac = np.vstack((primary_jac, secondary_jac * secondary_weight))
            err = np.concatenate((primary_err, secondary_err * secondary_weight))
            lhs = jac @ jac.T + (IK_DAMPING**2) * np.eye(jac.shape[0])
            step = jac.T @ np.linalg.solve(lhs, err)
            step = np.clip(step, -IK_MAX_STEP_RAD, IK_MAX_STEP_RAD)

            for qpos_id, joint_name, delta in zip(qpos_ids, joint_names, step):
                lower, upper = refs[joint_name].joint_range
                data.qpos[qpos_id] = np.clip(data.qpos[qpos_id] + delta, lower, upper)

        mujoco.mj_forward(model, data)
        solved = read_targets(data, refs, joint_names)
        primary_pos = data.xpos[primary_body_id].copy()
        secondary_pos = data.xpos[secondary_body_id].copy()
        primary_error_m = float(np.linalg.norm(primary_target_pos - primary_pos))
        secondary_error_m = float(np.linalg.norm(secondary_target_pos - secondary_pos))
        return solved, primary_pos, secondary_pos, primary_error_m, secondary_error_m
    finally:
        data.qpos[:] = original_qpos
        data.qvel[:] = original_qvel
        mujoco.mj_forward(model, data)


LEFT_BASKET_WRIST_UP = {
    "left_wrist_roll_joint":  0.0,
    "left_wrist_pitch_joint": 0.0,
    "left_wrist_yaw_joint":   0.0,
}

RIGHT_DROP_WRIST_DOWN = {
    "right_wrist_roll_joint":  -0.400,
    "right_wrist_pitch_joint": -0.400,
    "right_wrist_yaw_joint":    0.400,
}

RIGHT_ARM_PARK_POSE = {
    "right_shoulder_pitch_joint": -0.30,
    "right_shoulder_roll_joint":  -0.70,
    "right_shoulder_yaw_joint":    0.00,
    "right_elbow_joint":           1.20,
    "right_wrist_roll_joint":      0.00,
    "right_wrist_pitch_joint":     0.00,
    "right_wrist_yaw_joint":       0.00,
}

# Safe intermediate pose used on BOTH approach (before drop) and retreat (after drop).
# Arm is forward, elevated, and shifted to the RIGHT — far from the left arm/basket.
RIGHT_ARM_APPROACH_POSE = {
    "right_shoulder_pitch_joint": -0.70,
    "right_shoulder_roll_joint":  -0.60,
    "right_shoulder_yaw_joint":    0.00,
    "right_elbow_joint":           0.60,
    "right_wrist_roll_joint":      0.00,
    "right_wrist_pitch_joint":     0.00,
    "right_wrist_yaw_joint":       0.00,
}

RIGHT_ARM_CENTER_POSE = {
    "right_shoulder_pitch_joint": -0.20,
    "right_shoulder_roll_joint":  -0.10,
    "right_shoulder_yaw_joint":    0.00,
    "right_elbow_joint":           0.50,
    "right_wrist_roll_joint":      0.00,
    "right_wrist_pitch_joint":     0.00,
    "right_wrist_yaw_joint":       0.00,
}

LEFT_ARM_CENTER_POSE = {
    "left_shoulder_pitch_joint":  0.00,
    "left_shoulder_roll_joint":   0.00,
    "left_shoulder_yaw_joint":    0.00,
    "left_elbow_joint":           0.50,
    "left_wrist_roll_joint":      0.00,
    "left_wrist_pitch_joint":     0.00,
    "left_wrist_yaw_joint":       0.00,
}


def solve_left_basket_pose(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    refs: dict[str, JointActuator],
) -> tuple[dict[str, float], np.ndarray, np.ndarray, float]:
    """Left arm fixed L-shape pose — no IK needed. Hand position from FK."""
    hand_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, LEFT_HAND_TARGET_BODY)
    if hand_id < 0:
        raise ValueError(f"body not found: {LEFT_HAND_TARGET_BODY}")
    set_joint_positions(data, refs, LEFT_ARM_L_SHAPE_POSE)
    mujoco.mj_forward(model, data)
    hand_pos = data.xpos[hand_id].copy()
    return LEFT_ARM_L_SHAPE_POSE, hand_pos, hand_pos, 0.0


def solve_right_arm_drop_pose(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    refs: dict[str, JointActuator],
    left_basket_pose: dict[str, float],
) -> tuple[dict[str, float], np.ndarray, np.ndarray, np.ndarray, float, float]:
    original_qpos = data.qpos.copy()
    original_qvel = data.qvel.copy()
    right_hand_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, RIGHT_HAND_TARGET_BODY)
    right_elbow_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, RIGHT_ELBOW_TARGET_BODY)
    try:
        solved = clamp_targets(RECORDED_RIGHT_DROP_POSE, refs)
        set_joint_positions(data, refs, solved)
        mujoco.mj_forward(model, data)
        hand_pos = data.xpos[right_hand_id].copy()
        elbow_pos = data.xpos[right_elbow_id].copy()
        return solved, hand_pos, hand_pos, elbow_pos, 0.0, 0.0
    finally:
        data.qpos[:] = original_qpos
        data.qvel[:] = original_qvel
        mujoco.mj_forward(model, data)


def set_joint_positions(
    data: mujoco.MjData, refs: dict[str, JointActuator], targets: dict[str, float]
) -> None:
    for name, value in targets.items():
        data.qpos[refs[name].qposadr] = value
        data.qvel[refs[name].qveladr] = 0.0


def apply_pd_control(
    data: mujoco.MjData,
    refs: dict[str, JointActuator],
    targets: dict[str, float],
) -> None:
    for name, target in targets.items():
        ref = refs[name]
        q = float(data.qpos[ref.qposadr])
        dq = float(data.qvel[ref.qveladr])
        kp, kd = gains_for(ref)
        torque = kp * (target - q) - kd * dq
        data.ctrl[ref.actuator_id] = np.clip(torque, ref.ctrlrange[0], ref.ctrlrange[1])


def gains_for(ref: JointActuator) -> tuple[float, float]:
    if ref.joint_name.startswith("right_gripper_"):
        return 4.0, 0.15
    if "wrist_pitch" in ref.joint_name or "wrist_yaw" in ref.joint_name:
        return 25.0, 1.0
    if "shoulder" in ref.joint_name or "elbow" in ref.joint_name or "wrist_roll" in ref.joint_name:
        return 55.0, 2.0
    if ref.joint_name.startswith("waist_"):
        return 80.0, 3.0
    return 120.0, 5.0


def build_targets(
    step: int,
    hold_targets: dict[str, float],
    hand_open_for_load: ArmInterpolator,
    gripper_hold_open_load_steps: int,
    hand_close_for_hold: ArmInterpolator,
    gripper_hold_closed_steps: int,
    right_to_approach: ArmInterpolator,
    right_to_drop: ArmInterpolator,
    left_to_basket: ArmInterpolator,
    hand_open_for_drop: ArmInterpolator,
    gripper_hold_at_basket_steps: int,
    gripper_hold_open_drop_steps: int,
    hand_close_after_drop: ArmInterpolator,
    drop_to_approach: ArmInterpolator,
    right_to_park: ArmInterpolator,
    left_to_safe_park: ArmInterpolator,
    left_to_normal: ArmInterpolator,
    right_to_normal: ArmInterpolator,
    wait_steps: int,
) -> tuple[dict[str, float], bool, str]:
    targets = dict(hold_targets)

    # Phase 0a: gripper opens (place okra into hand)
    if not hand_open_for_load.is_done(step):
        targets.update(hand_open_for_load.get_target(step))
        return targets, False, "phase0a_gripper_open_load"

    targets.update(hand_open_for_load.get_target(hand_open_for_load.total_steps))

    # Phase 0b: hold open (user places okra)
    s0b = step - hand_open_for_load.total_steps
    if s0b < gripper_hold_open_load_steps:
        return targets, False, "phase0b_hold_open_load"

    # Phase 0c: gripper closes (holding okra)
    s0c = s0b - gripper_hold_open_load_steps
    if not hand_close_for_hold.is_done(s0c):
        targets.update(hand_close_for_hold.get_target(s0c))
        return targets, False, "phase0c_gripper_close_hold"

    targets.update(hand_close_for_hold.get_target(hand_close_for_hold.total_steps))

    # Phase 0d: hold closed before arm motion (wait_steps + gripper_hold_closed_steps)
    s0d = s0c - hand_close_for_hold.total_steps
    pre_move_steps = gripper_hold_closed_steps + wait_steps
    if s0d < pre_move_steps:
        return targets, False, "waiting"

    move_step = s0d - pre_move_steps

    # Phase 1: left arm → basket (right arm stays at start)
    if not left_to_basket.is_done(move_step):
        targets.update(left_to_basket.get_target(move_step))
        return targets, False, "phase1_left_to_basket"

    targets.update(left_to_basket.get_target(left_to_basket.total_steps))

    # Phase 2: right arm → approach (safe intermediate — forward, right, elevated)
    phase2_step = move_step - left_to_basket.total_steps
    if not right_to_approach.is_done(phase2_step):
        targets.update(right_to_approach.get_target(phase2_step))
        return targets, False, "phase2_right_to_approach"

    targets.update(right_to_approach.get_target(right_to_approach.total_steps))

    # Phase 3: right arm → drop (from approach position)
    phase3_step = phase2_step - right_to_approach.total_steps
    if not right_to_drop.is_done(phase3_step):
        targets.update(right_to_drop.get_target(phase3_step))
        return targets, False, "phase3_right_to_drop"

    targets.update(right_to_drop.get_target(right_to_drop.total_steps))

    # Phase 3b: hold closed at basket before opening
    s3b = phase3_step - right_to_drop.total_steps
    if s3b < gripper_hold_at_basket_steps:
        return targets, False, "phase3b_hold_at_basket"

    # Phase 4: gripper opens — okra drops into basket
    phase4_step = s3b - gripper_hold_at_basket_steps
    if not hand_open_for_drop.is_done(phase4_step):
        targets.update(hand_open_for_drop.get_target(phase4_step))
        return targets, False, "phase4_gripper_open_drop"

    targets.update(hand_open_for_drop.get_target(hand_open_for_drop.total_steps))

    # Phase 4b: hold open while okra falls
    s4b = phase4_step - hand_open_for_drop.total_steps
    if s4b < gripper_hold_open_drop_steps:
        return targets, False, "phase4b_hold_open_drop"

    # Phase 5: gripper closes after drop
    s5 = s4b - gripper_hold_open_drop_steps
    if not hand_close_after_drop.is_done(s5):
        targets.update(hand_close_after_drop.get_target(s5))
        return targets, False, "phase5_gripper_close_drop"

    targets.update(hand_close_after_drop.get_target(hand_close_after_drop.total_steps))

    # Phase 6: right arm retreats to approach pose (clears left arm before parking)
    phase6_step = s5 - hand_close_after_drop.total_steps
    if not drop_to_approach.is_done(phase6_step):
        targets.update(drop_to_approach.get_target(phase6_step))
        return targets, False, "phase6_right_retreat"

    targets.update(drop_to_approach.get_target(drop_to_approach.total_steps))

    # Phase 7: right arm parks (tucks to right side)
    phase7_step = phase6_step - drop_to_approach.total_steps
    if not right_to_park.is_done(phase7_step):
        targets.update(right_to_park.get_target(phase7_step))
        return targets, False, "phase7_right_park"

    targets.update(right_to_park.get_target(right_to_park.total_steps))

    # Phase 8a: left arm → safe park (arm out to side, basket facing up, clear of leg)
    phase8a_step = phase7_step - right_to_park.total_steps
    if not left_to_safe_park.is_done(phase8a_step):
        targets.update(left_to_safe_park.get_target(phase8a_step))
        return targets, False, "phase8a_left_safe_park"

    targets.update(left_to_safe_park.get_target(left_to_safe_park.total_steps))

    # Phase 8b: left arm → rest (already outside leg zone, safe swing)
    phase8b_step = phase8a_step - left_to_safe_park.total_steps
    if not left_to_normal.is_done(phase8b_step):
        targets.update(left_to_normal.get_target(phase8b_step))
        return targets, False, "phase8b_left_return"

    targets.update(left_to_normal.get_target(left_to_normal.total_steps))

    # Phase 9: right arm returns to normal from park
    phase9_step = phase8b_step - left_to_normal.total_steps
    if not right_to_normal.is_done(phase9_step):
        targets.update(right_to_normal.get_target(phase9_step))
        return targets, False, "phase9_right_return"

    targets.update(right_to_normal.get_target(right_to_normal.total_steps))
    return targets, True, "done"


def check_wrist_orientations(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    for side, body, label, target_up in [
        ("right", "right_wrist_yaw_link", "palm DOWN (drop)", -1.0),
        ("left",  "left_wrist_yaw_link",  "basket UP (catch)", +1.0),
    ]:
        bid   = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body)
        score = float(data.xmat[bid].reshape(3, 3)[:, 1][2])
        ok    = (score * target_up) > 0.7
        status = "OK" if ok else "WARNING"
        print(f"  {side} wrist — {label}: score={score:.3f}  [{status}]")


def print_selected_mapping(refs: dict[str, JointActuator]) -> None:
    print("right arm mapping:")
    for name in RIGHT_ARM_JOINTS:
        ref = refs[name]
        print(
            f"  {name}: actuator={ref.actuator_name}({ref.actuator_id}), "
            f"qpos={ref.qposadr}, qvel={ref.qveladr}"
        )
    print("right hand mapping:")
    for name in RIGHT_HAND_JOINTS:
        ref = refs[name]
        print(
            f"  {name}: actuator={ref.actuator_name}({ref.actuator_id}), "
            f"qpos={ref.qposadr}, qvel={ref.qveladr}"
        )


def run_demo(args: argparse.Namespace) -> None:
    scene_path = Path(args.scene)
    if not scene_path.exists():
        raise FileNotFoundError(
            f"{scene_path} not found. "
            "Get the model: git clone https://github.com/unitreerobotics/unitree_mujoco.git "
            "then pass --scene <path>/unitree_robots/g1/scene_29dof_with_hand.xml"
        )

    model = mujoco.MjModel.from_xml_path(str(scene_path))
    model.opt.timestep = SIMULATE_DT
    if not args.enable_gravity:
        model.opt.gravity[:] = 0.0
    data = mujoco.MjData(model)

    all_joint_names = get_actuated_joint_names(model)
    refs = {name: get_joint_actuator(model, name) for name in all_joint_names}
    print_selected_mapping(refs)
    if args.free_base:
        print("floating base: free (robot can fall without a balance controller)")
    else:
        print("floating base: locked for arm-only MuJoCo demo")
    if args.enable_gravity:
        print("gravity: enabled")
    else:
        print("gravity: disabled for stable arm-only demo")
    if args.dynamic:
        print("playback mode: dynamic PD simulation")
    else:
        print("playback mode: kinematic visual replay")

    hand_closed = clamp_targets(RIGHT_HAND_CLOSED_POSE, refs)
    hand_open = clamp_targets(RIGHT_HAND_OPEN_POSE, refs)

    left_basket_pose, left_target_pos, left_hand_pos, left_ik_error_m = (
        solve_left_basket_pose(model, data, refs)
    )
    set_joint_positions(data, refs, left_basket_pose)
    mujoco.mj_forward(model, data)

    rng = np.random.default_rng(args.seed)
    arm_start = sample_right_arm_start(model, data, rng, refs)
    (
        drop_pose,
        left_hand_pos,
        right_target_pos,
        right_elbow_pos,
        ik_error_m,
        right_elbow_error_m,
    ) = solve_right_arm_drop_pose(
        model, data, refs, left_basket_pose
    )

    # left arm starts at safe rest pose (shoulder_roll +0.30, arm away from leg)
    left_arm_rest = dict(LEFT_ARM_REST_POSE)
    set_joint_positions(data, refs, left_arm_rest)
    set_joint_positions(data, refs, arm_start)
    set_joint_positions(data, refs, hand_closed)
    mujoco.mj_forward(model, data)
    base_qpos = data.qpos[FREE_BASE_QPOS].copy()

    hold_targets = read_targets(data, refs, all_joint_names)
    phase1_duration_s = estimate_move_duration_s(left_arm_rest, left_basket_pose)
    phase2_duration_s = estimate_move_duration_s(arm_start, drop_pose)
    steps_per_s = round(1.0 / SIMULATE_DT)

    right_arm_zeros = {j: 0.0 for j in RIGHT_ARM_JOINTS}

    # Forward: left → basket, right → approach (safe waypoint), right → drop
    approach_duration_s = estimate_move_duration_s(arm_start,              RIGHT_ARM_APPROACH_POSE)
    left_to_basket      = ArmInterpolator(left_arm_rest,          left_basket_pose,        phase1_duration_s,   steps_per_s)
    right_to_approach   = ArmInterpolator(arm_start,              RIGHT_ARM_APPROACH_POSE, approach_duration_s, steps_per_s)
    right_to_drop       = ArmInterpolator(RIGHT_ARM_APPROACH_POSE, drop_pose,              phase2_duration_s,   steps_per_s)
    # Gripper animations: open for load, close holding, open for drop, close after drop
    hand_open_for_load    = ArmInterpolator(hand_closed, hand_open,   GRIPPER_OPEN_DURATION_S,  steps_per_s)
    hand_close_for_hold   = ArmInterpolator(hand_open,   hand_closed, GRIPPER_CLOSE_DURATION_S, steps_per_s)
    hand_open_for_drop    = ArmInterpolator(hand_closed, hand_open,   GRIPPER_OPEN_DURATION_S,  steps_per_s)
    hand_close_after_drop = ArmInterpolator(hand_open,   hand_closed, GRIPPER_CLOSE_DURATION_S, steps_per_s)
    gripper_hold_open_load_steps  = round(GRIPPER_HOLD_OPEN_LOAD_S    / SIMULATE_DT)
    gripper_hold_closed_steps     = round(GRIPPER_HOLD_CLOSED_S       / SIMULATE_DT)
    gripper_hold_at_basket_steps  = round(GRIPPER_HOLD_AT_BASKET_S    / SIMULATE_DT)
    gripper_hold_open_drop_steps  = round(GRIPPER_HOLD_OPEN_DROP_S    / SIMULATE_DT)
    # Return: drop → approach (retreat), right parks, left home, right home
    retreat_duration_s      = estimate_move_duration_s(drop_pose,             RIGHT_ARM_APPROACH_POSE)
    park_duration_s         = estimate_move_duration_s(RIGHT_ARM_APPROACH_POSE, RIGHT_ARM_PARK_POSE)
    left_safe_park_duration_s = estimate_move_duration_s(LEFT_ARM_L_SHAPE_POSE,    LEFT_ARM_SAFE_PARK_POSE)
    left_return_duration_s    = estimate_move_duration_s(LEFT_ARM_SAFE_PARK_POSE,  LEFT_ARM_BASKET_REST_POSE)
    right_return_duration_s   = estimate_move_duration_s(RIGHT_ARM_PARK_POSE,      RIGHT_ARM_NATURAL_REST_POSE)
    drop_to_approach   = ArmInterpolator(drop_pose,               RIGHT_ARM_APPROACH_POSE,   retreat_duration_s,        steps_per_s)
    right_to_park      = ArmInterpolator(RIGHT_ARM_APPROACH_POSE, RIGHT_ARM_PARK_POSE,       park_duration_s,           steps_per_s)
    left_to_safe_park  = ArmInterpolator(LEFT_ARM_L_SHAPE_POSE,   LEFT_ARM_SAFE_PARK_POSE,   left_safe_park_duration_s, steps_per_s)
    left_to_normal     = ArmInterpolator(LEFT_ARM_SAFE_PARK_POSE, LEFT_ARM_BASKET_REST_POSE, left_return_duration_s,    steps_per_s)
    right_to_normal    = ArmInterpolator(RIGHT_ARM_PARK_POSE,     RIGHT_ARM_NATURAL_REST_POSE, right_return_duration_s,  steps_per_s)
    wait_steps = round(WAIT_BEFORE_MOVE_S / SIMULATE_DT)
    done_hold_steps = round(HOLD_AFTER_DONE_S / SIMULATE_DT)

    print("right arm random start:")
    for name, value in arm_start.items():
        print(f"  {name}: {value:.4f}")
    print("left basket pose:")
    for name, value in left_basket_pose.items():
        print(f"  {name}: {value:.4f}")
    print(
        "left hand target: "
        f"({left_target_pos[0]:.3f}, {left_target_pos[1]:.3f}, {left_target_pos[2]:.3f})"
    )
    print(
        "left hand position: "
        f"({left_hand_pos[0]:.3f}, {left_hand_pos[1]:.3f}, {left_hand_pos[2]:.3f})"
    )
    print(f"left arm IK error: {left_ik_error_m:.4f} m")
    print(
        "right hand target: "
        f"({right_target_pos[0]:.3f}, {right_target_pos[1]:.3f}, {right_target_pos[2]:.3f})"
    )
    print(f"right arm IK error: {ik_error_m:.4f} m")
    right_elbow_rel = right_elbow_pos - base_qpos[:3]
    print(
        "right elbow position rel pelvis: "
        f"({right_elbow_rel[0]:.3f}, {right_elbow_rel[1]:.3f}, {right_elbow_rel[2]:.3f})"
    )
    print(f"right elbow secondary IK error: {right_elbow_error_m:.4f} m")
    print("right arm drop pose:")
    for name, value in drop_pose.items():
        print(f"  {name}: {value:.4f}")
    set_joint_positions(data, refs, drop_pose)
    set_joint_positions(data, refs, left_basket_pose)
    mujoco.mj_forward(model, data)
    print("wrist orientation check (at final pose):")
    check_wrist_orientations(model, data)
    set_joint_positions(data, refs, arm_start)
    mujoco.mj_forward(model, data)
    print(
        f"phase1 duration: {phase1_duration_s:.2f}s  phase2 duration: {phase2_duration_s:.2f}s "
        f"(max joint speed <= {MAX_JOINT_SPEED_RAD_S:.2f} rad/s)"
    )

    if args.no_viewer:
        run_loop(
            model, data, refs, hold_targets,
            hand_open_for_load, gripper_hold_open_load_steps,
            hand_close_for_hold, gripper_hold_closed_steps,
            right_to_approach, right_to_drop, left_to_basket,
            hand_open_for_drop, gripper_hold_at_basket_steps, gripper_hold_open_drop_steps,
            hand_close_after_drop,
            drop_to_approach, right_to_park, left_to_safe_park, left_to_normal, right_to_normal,
            wait_steps, done_hold_steps,
            base_qpos=base_qpos,
            lock_base=not args.free_base,
            dynamic=args.dynamic,
            viewer=None,
            real_time=False,
        )
        return

    with mujoco.viewer.launch_passive(model, data) as viewer:
        configure_camera(viewer)
        run_loop(
            model, data, refs, hold_targets,
            hand_open_for_load, gripper_hold_open_load_steps,
            hand_close_for_hold, gripper_hold_closed_steps,
            right_to_approach, right_to_drop, left_to_basket,
            hand_open_for_drop, gripper_hold_at_basket_steps, gripper_hold_open_drop_steps,
            hand_close_after_drop,
            drop_to_approach, right_to_park, left_to_safe_park, left_to_normal, right_to_normal,
            wait_steps, done_hold_steps=None,
            base_qpos=base_qpos,
            lock_base=not args.free_base,
            dynamic=args.dynamic,
            viewer=viewer,
            real_time=True,
        )


def _mujoco_safe_stop(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    refs: dict[str, JointActuator],
    base_qpos: np.ndarray,
    lock_base: bool,
    dynamic: bool,
    viewer,
) -> None:
    """Smoothly move all arms to safe rest position, then hold — called on Ctrl+C."""
    print("\n[SAFE STOP] Moving arms to rest position — do not touch the robot...")

    current = {name: float(data.qpos[ref.qposadr]) for name, ref in refs.items()}

    # Safe rest: left shoulder_roll +0.30 (arm away from leg), everything else zero
    safe_rest = {name: 0.0 for name in refs}
    safe_rest["left_shoulder_roll_joint"] = LEFT_ARM_REST_POSE["left_shoulder_roll_joint"]
    safe_rest.update(RIGHT_HAND_CLOSED_POSE)   # close fingers if open

    arm_names = [n for n in refs if any(x in n for x in ("shoulder", "elbow", "wrist"))]
    max_delta  = max(abs(safe_rest[n] - current[n]) for n in arm_names)
    duration_s = max(3.0, max_delta / SAFE_STOP_SPEED_RAD_S)
    steps_per_s = round(1.0 / SIMULATE_DT)

    interp = ArmInterpolator(current, safe_rest, duration_s, steps_per_s)
    print(f"[SAFE STOP] Duration: {duration_s:.1f}s")

    for step in range(interp.total_steps + 1):
        step_start = time.perf_counter()
        targets = interp.get_target(step)

        if dynamic:
            apply_pd_control(data, refs, targets)
            mujoco.mj_step(model, data)
        else:
            set_joint_positions(data, refs, targets)
            mujoco.mj_forward(model, data)

        if lock_base:
            data.qpos[FREE_BASE_QPOS] = base_qpos
            data.qvel[FREE_BASE_QVEL] = 0.0
            mujoco.mj_forward(model, data)

        if viewer is not None:
            viewer.sync()

        sleep_s = SIMULATE_DT - (time.perf_counter() - step_start)
        if sleep_s > 0:
            time.sleep(sleep_s)

    print("[SAFE STOP] Arms at safe rest position.")


def run_loop(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    refs: dict[str, JointActuator],
    hold_targets: dict[str, float],
    hand_open_for_load: ArmInterpolator,
    gripper_hold_open_load_steps: int,
    hand_close_for_hold: ArmInterpolator,
    gripper_hold_closed_steps: int,
    right_to_approach: ArmInterpolator,
    right_to_drop: ArmInterpolator,
    left_to_basket: ArmInterpolator,
    hand_open_for_drop: ArmInterpolator,
    gripper_hold_at_basket_steps: int,
    gripper_hold_open_drop_steps: int,
    hand_close_after_drop: ArmInterpolator,
    drop_to_approach: ArmInterpolator,
    right_to_park: ArmInterpolator,
    left_to_safe_park: ArmInterpolator,
    left_to_normal: ArmInterpolator,
    right_to_normal: ArmInterpolator,
    wait_steps: int,
    done_hold_steps: int | None,
    base_qpos: np.ndarray,
    lock_base: bool,
    dynamic: bool,
    viewer,
    real_time: bool,
) -> None:
    step = 0
    done_step = None
    last_viewer_sync = 0.0
    current_phase = "waiting"
    _zero_moment_active = [False]  # set True by Ctrl+C to release joints
    _completed_normally = False    # set True when all phases finish naturally

    # Intercept Ctrl+C with a flag so we can safe-stop before exiting
    _stop_requested = [False]
    _orig_sigint = signal.getsignal(signal.SIGINT)

    def _on_sigint(sig, frame):
        _stop_requested[0] = True

    signal.signal(signal.SIGINT, _on_sigint)

    while viewer is None or viewer.is_running():
        if _stop_requested[0]:
            # Ctrl+C: zero moment in same viewer — no new window
            print("\nCtrl+C — entering zero moment (gravity ON, all joints passive)...")
            model.opt.gravity[2] = -9.81
            signal.signal(signal.SIGINT, signal.SIG_IGN)
            zm_steps = round(3.0 / model.opt.timestep)
            for _ in range(zm_steps):
                data.ctrl[:] = 0.0
                mujoco.mj_step(model, data)  # no lock_base — let arms fall freely
                if viewer is not None:
                    viewer.sync()
                time.sleep(model.opt.timestep)
            print("Zero moment complete — exiting.")
            break

        step_start = time.perf_counter()
        targets, done, phase = build_targets(
            step, hold_targets,
            hand_open_for_load, gripper_hold_open_load_steps,
            hand_close_for_hold, gripper_hold_closed_steps,
            right_to_approach, right_to_drop, left_to_basket,
            hand_open_for_drop, gripper_hold_at_basket_steps, gripper_hold_open_drop_steps,
            hand_close_after_drop,
            drop_to_approach, right_to_park, left_to_safe_park, left_to_normal, right_to_normal,
            wait_steps,
        )

        if phase != current_phase:
            current_phase = phase
            if phase == "phase0a_gripper_open_load":
                print("→ Phase 0a: right gripper opening — place okra into hand")
            elif phase == "phase0b_hold_open_load":
                print("→ Phase 0b: holding open — okra being placed")
            elif phase == "phase0c_gripper_close_hold":
                print("→ Phase 0c: right gripper closing — holding okra")
            elif phase == "waiting":
                print("→ Ready: holding position before arm motion starts")
            elif phase == "phase1_left_to_basket":
                print("→ Phase 1: left arm moving to basket (L-shape)")
            elif phase == "phase2_right_to_approach":
                print("→ Phase 2: right arm moving to approach position (safe, away from left arm)")
            elif phase == "phase3_right_to_drop":
                print("→ Phase 3: right arm moving to drop position above basket")
            elif phase == "phase3b_hold_at_basket":
                print("→ Phase 3b: holding okra at basket position before releasing")
            elif phase == "phase4_gripper_open_drop":
                print("→ Phase 4: right gripper opening — okra drops into basket")
            elif phase == "phase4b_hold_open_drop":
                print("→ Phase 4b: holding open — okra falling")
            elif phase == "phase5_gripper_close_drop":
                print("→ Phase 5: right gripper closing after drop")
            elif phase == "phase6_right_retreat":
                print("→ Phase 6: right arm retreating to approach position (clearing left arm)")
            elif phase == "phase7_right_park":
                print("→ Phase 7: right arm parking to the right")
            elif phase == "phase8a_left_safe_park":
                print("→ Phase 8a: left arm moving to safe park (basket up, clear of leg)")
            elif phase == "phase8b_left_return":
                print("→ Phase 8b: left arm returning to rest (already outside leg zone)")
            elif phase == "phase9_right_return":
                print("→ Phase 9: right arm returning to normal")
            elif phase == "done":
                print("→ Done: all phases complete")

        if dynamic:
            apply_pd_control(data, refs, targets)
            mujoco.mj_step(model, data)
            if lock_base:
                lock_floating_base(model, data, base_qpos)
            sim_time = data.time
        else:
            set_joint_positions(data, refs, targets)
            if lock_base:
                data.qpos[FREE_BASE_QPOS] = base_qpos
                data.qvel[FREE_BASE_QVEL] = 0.0
            data.time = step * model.opt.timestep
            mujoco.mj_forward(model, data)
            sim_time = data.time

        if viewer is not None and sim_time - last_viewer_sync >= VIEWER_DT:
            viewer.sync()
            last_viewer_sync = sim_time

        if done and done_step is None:
            done_step = step
            print("drop motion complete; holding final pose")

        step += 1
        if done_step is not None and done_hold_steps is not None:
            if step - done_step >= done_hold_steps:
                _completed_normally = True
                break

        if real_time:
            sleep_s = model.opt.timestep - (time.perf_counter() - step_start)
            if sleep_s > 0:
                time.sleep(sleep_s)

    signal.signal(signal.SIGINT, _orig_sigint)

    # Auto zero moment after task completes normally — same viewer, no lock_base
    if _completed_normally:
        print("\n→ Task complete — entering zero moment (arms falling under gravity)...")
        model.opt.gravity[2] = -9.81
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        zm_steps = round(3.0 / model.opt.timestep)
        for _ in range(zm_steps):
            data.ctrl[:] = 0.0
            mujoco.mj_step(model, data)  # no lock_base — let arms fall freely
            if viewer is not None:
                viewer.sync()
            time.sleep(model.opt.timestep)
        print("Zero moment complete — exiting.")
        signal.signal(signal.SIGINT, _orig_sigint)


def lock_floating_base(
    model: mujoco.MjModel, data: mujoco.MjData, base_qpos: np.ndarray
) -> None:
    data.qpos[FREE_BASE_QPOS] = base_qpos
    data.qvel[FREE_BASE_QVEL] = 0.0
    mujoco.mj_forward(model, data)


def configure_camera(viewer) -> None:
    viewer.cam.lookat[:] = (0.15, 0.0, 0.75)
    viewer.cam.distance = 3.2
    viewer.cam.azimuth = 90
    viewer.cam.elevation = -12


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Move G1 right arm over the basket and open the right gripper in MuJoCo."
    )
    parser.add_argument(
        "--scene",
        type=str,
        default=DEFAULT_SCENE_XML,
        help="path to MuJoCo scene XML (default: %(default)s)",
    )
    parser.add_argument("--seed", type=int, default=None, help="random seed")
    parser.add_argument(
        "--no-viewer",
        action="store_true",
        help="run the simulation without opening the MuJoCo viewer",
    )
    parser.add_argument(
        "--free-base",
        action="store_true",
        help="do not lock the floating base; requires a balance controller to stay upright",
    )
    parser.add_argument(
        "--enable-gravity",
        action="store_true",
        help="enable gravity; the default disables it to avoid shaking in the arm-only demo",
    )
    parser.add_argument(
        "--dynamic",
        action="store_true",
        help="use PD torques and mj_step; default is kinematic visual replay to avoid jitter",
    )
    return parser.parse_args()


if __name__ == "__main__":
    run_demo(parse_args())
