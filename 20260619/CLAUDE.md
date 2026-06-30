# Codebase Rules

## Locked Drop Positions

`LEFT_ARM_L_SHAPE_POSE` and `RECORDED_RIGHT_DROP_POSE` in `src/drop_to_basket_mujoco.py` are **locked** — do not modify their values under any circumstances.

- These are recorded from the real robot on 2026-06-25 16:34:52 (`src/data/taught_poses/000_20260625_163452_pose_000.json`)
- Recording a new position to the robot does **not** update these constants — they are hardcoded intentionally
- The script validates these values via checksum at startup and will crash if they are changed
- To intentionally update them, you must change both the pose values and the `_LOCKED_*` checksum lines together, and justify the change in the git commit message
