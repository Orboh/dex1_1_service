from __future__ import annotations

import argparse
import json
from pathlib import Path

import mujoco

import drop_to_basket_mujoco as demo


DEFAULT_OUTPUT = Path(__file__).resolve().parent / "data" / "mujoco_right_arm_drop_pose.json"

MUJOCO_TO_REAL = {
    "right_shoulder_pitch_joint": "right_shoulder_pitch",
    "right_shoulder_roll_joint":  "right_shoulder_roll",
    "right_shoulder_yaw_joint":   "right_shoulder_yaw",
    "right_elbow_joint":          "right_elbow",
    "right_wrist_roll_joint":     "right_wrist_roll",
    "right_wrist_pitch_joint":    "right_wrist_pitch",
    "right_wrist_yaw_joint":      "right_wrist_yaw",
    "left_shoulder_pitch_joint":  "left_shoulder_pitch",
    "left_shoulder_roll_joint":   "left_shoulder_roll",
    "left_shoulder_yaw_joint":    "left_shoulder_yaw",
    "left_elbow_joint":           "left_elbow",
    "left_wrist_roll_joint":      "left_wrist_roll",
    "left_wrist_pitch_joint":     "left_wrist_pitch",
    "left_wrist_yaw_joint":       "left_wrist_yaw",
}


def _to_real(pose: dict[str, float]) -> dict[str, float]:
    return {MUJOCO_TO_REAL[k]: float(v) for k, v in pose.items()}


def compute_drop_pose(scene_path: Path) -> dict:
    if not scene_path.exists():
        raise FileNotFoundError(
            f"{scene_path} not found. "
            "Get the model: git clone https://github.com/unitreerobotics/unitree_mujoco.git "
            "then pass --scene <path>/unitree_robots/g1/scene_29dof_with_hand.xml"
        )

    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)
    refs = {
        name: demo.get_joint_actuator(model, name)
        for name in demo.get_actuated_joint_names(model)
    }

    left_basket_pose, left_target_pos, left_hand_pos, left_ik_error_m = (
        demo.solve_left_basket_pose(model, data, refs)
    )
    (
        drop_pose,
        _left_hand_pos_for_drop,
        right_target_pos,
        right_elbow_pos,
        right_ik_error_m,
        right_elbow_error_m,
    ) = demo.solve_right_arm_drop_pose(model, data, refs, left_basket_pose)

    right_center = _to_real(demo.RIGHT_ARM_CENTER_POSE)
    left_center  = _to_real(demo.LEFT_ARM_CENTER_POSE)

    return {
        "source_xml": str(scene_path),
        "right_arm_drop_pose":   _to_real(drop_pose),
        "left_basket_pose":      _to_real(left_basket_pose),
        "right_arm_body_center": right_center,
        "left_arm_body_center":  left_center,
        "mujoco_right_arm_drop_pose": {k: float(v) for k, v in drop_pose.items()},
        "ik": {
            "left_error_m":        float(left_ik_error_m),
            "right_error_m":       float(right_ik_error_m),
            "right_elbow_error_m": float(right_elbow_error_m),
        },
        "positions": {
            "left_target_xyz":  [float(v) for v in left_target_pos],
            "left_hand_xyz":    [float(v) for v in left_hand_pos],
            "right_target_xyz": [float(v) for v in right_target_pos],
            "right_elbow_xyz":  [float(v) for v in right_elbow_pos],
        },
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Export MuJoCo arm poses to JSON for real G1."
    )
    parser.add_argument(
        "--scene",
        type=Path,
        default=Path(demo.DEFAULT_SCENE_XML),
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    result = compute_drop_pose(args.scene)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    print(f"wrote {args.output}")
    print("right_arm_drop_pose:")
    for k, v in result["right_arm_drop_pose"].items():
        print(f"  {k:24s} {v: .5f} rad")
    print("left_basket_pose:")
    for k, v in result["left_basket_pose"].items():
        print(f"  {k:24s} {v: .5f} rad")
    print(
        f"ik errors: left={result['ik']['left_error_m']:.4f} m  "
        f"right={result['ik']['right_error_m']:.4f} m  "
        f"right_elbow={result['ik']['right_elbow_error_m']:.4f} m"
    )


if __name__ == "__main__":
    main()
