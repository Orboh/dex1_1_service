# G1 Okra Drop Task — Testing Summary
Date: 2026-06-25

---

## Task Overview
Move okra from right hand into a basket held by the left arm using the Unitree G1 robot.

---

## Scripts

| File | Purpose |
|---|---|
| `drop_to_basket_real.py` | Real robot — full 9-phase motion |
| `drop_to_basket_mujoco.py` | MuJoCo simulation |
| `g1_teach_pose.py` | Kinesthetic teaching (kp=0, kd=3 damping mode) |
| `data/mujoco_right_arm_drop_pose.json` | IK-calculated drop pose (MuJoCo only) |

---

## Recorded Positions (Real Robot)

### Left Arm — Basket (L-shape) `LEFT_ARM_L_SHAPE`
Recorded: 2026-06-25 via kinesthetic teaching

| Joint | Value (rad) |
|---|---|
| left_shoulder_pitch | -0.1170 |
| left_shoulder_roll | -0.0167 |
| left_shoulder_yaw | -0.3997 |
| left_elbow | 1.1330 |
| left_wrist_roll | 0.0834 |
| left_wrist_pitch | -1.0673 |
| left_wrist_yaw | -0.2355 |

### Right Arm — Drop Position `RIGHT_ARM_DROP_POSE`
Recorded: 2026-06-25 via kinesthetic teaching — **DO NOT CHANGE**

| Joint | Value (rad) |
|---|---|
| right_shoulder_pitch | -0.0556 |
| right_shoulder_roll | -0.1400 |
| right_shoulder_yaw | -0.0624 |
| right_elbow | -0.4139 |
| right_wrist_roll | -0.6738 |
| right_wrist_pitch | -0.8083 |
| right_wrist_yaw | 0.9169 |

---

## Motion Phases (Real Robot — 9 phases)

| Phase | Action |
|---|---|
| 0 | Right gripper OPENS → user places okra → gripper GRIP (q=4.8, gentle) |
| 1 | Left arm → basket (L-shape) |
| 2 | Right arm → approach position (safe intermediate, away from left arm) |
| 3 | Right arm → drop position (recorded) |
| 4 | Right gripper OPENS (q=5.2) → okra drops into basket → 3s wait |
| 5 | Right gripper CLOSES (q=4.4, fully closed) |
| 6 | Right arm retreats → approach position |
| 7 | Right arm parks to the right |
| 8 | Left arm returns SLOWLY via safe waypoint (avoid leg) |
| 9 | Right arm returns to natural |

---

## Gripper Values

| State | Position (rad) | Used in |
|---|---|---|
| OPEN | 5.2 | Phase 0, Phase 4 |
| GRIP (gentle) | 4.8 | Phase 0 close (holds okra without squeezing) |
| CLOSE (full) | 4.4 | Phase 5 (after drop, okra already released) |

---

## Safety Features

### Left Arm Rest Position `LEFT_ARM_REST_POSE`
- `left_shoulder_roll = +0.60 rad` (arm clearly out to the left, away from leg)
- Used as final position in Phase 8 and Ctrl+C safe stop

### Safe Stop (Ctrl+C at any point)
Both arms smoothly move to rest position at 0.3 rad/s before releasing arm_sdk.
```
Ctrl+C → [SAFE STOP] arms move to rest → arm_sdk released
```

### Left Arm Return (Phase 8) — 2-step slow path
- Speed: 0.2 rad/s (half of normal)
- Step 8a: de-rotate shoulder_yaw, start straightening elbow
- Step 8b: go to safe rest (shoulder_roll +0.60)

---

## Testing Results

### MuJoCo Simulation ✓
- All 9 phases run correctly
- Gripper open/close animation visible (Phase 0a/0b/0c and Phase 4/4b/5)
- Ctrl+C safe stop tested — arms move smoothly to rest position
- Left arm starts and ends at `LEFT_ARM_REST_POSE` (shoulder_roll +0.60)

### Real Robot Tests
| Test | Result |
|---|---|
| Left arm basket position | ✓ Correct (recorded and verified) |
| Right arm drop position | Recorded from real robot, hardcoded |
| Gripper open/close | ✗ First attempt failed — `reserve` field type error (fixed: `[0,0,0]` not `0`) |
| Full 9-phase motion | Not yet tested after drop position fix |

---

## Known Problems / Pending

1. **Gripper server** — `dex1_1_gripper_server` must be manually started on robot PC `192.168.123.164` before running the script. Path on robot PC not confirmed.

2. **Drop position not yet re-tested** — After hardcoding the recorded drop position, full real robot test has not been run yet.

3. **Approach position not verified on real robot** — `RIGHT_ARM_APPROACH_POSE` (shoulder_pitch=-0.70, shoulder_roll=-0.60, elbow=0.60) was taken from MuJoCo simulation. May need adjustment for real robot.

---

## TODO — Solve Next Day (2026-06-26)

### Problem 1: Gripper closing position
- Current gentle grip is `q = 4.8 rad` — needs real robot testing to confirm it holds okra firmly without squeezing
- May need to fine-tune the value (higher = looser, lower = tighter)
- **Action:** Test gripper grip on actual okra, adjust `GRIPPER_Q_GRIP` value in `drop_to_basket_real.py`

### Problem 2: Drop position for right hand and left hand
- Right arm drop position recorded but not fully tested with the new 9-phase flow
- Left arm basket position needs to be verified that it lines up correctly with the right arm drop position
- Okra may not fall accurately into the basket if the two arm positions are misaligned
- **Action:** Run full test, check if okra lands inside basket, re-record positions if needed

### Problem 3: Basket hitting into the body
- When the left arm holds the basket, the basket edge may be too close to the robot body
- Risk of contact with the torso during motion or when okra drops
- **Action:** Adjust `LEFT_ARM_L_SHAPE` — try increasing `left_shoulder_roll` (move basket further from body) or `left_shoulder_pitch` (tilt arm forward)

---

## Run Commands

### MuJoCo Simulation
```bash
conda activate unitree_sim_env
cd /home/techshare/user/yokote/20260619/src
python3 drop_to_basket_mujoco.py --scene /home/techshare/drl_kit/mujoco_ws/ts_mujoco-main/unitree_robots/g1/scene_29dof_with_hand.xml
```

### Real Robot
```bash
# Step 1: Remote controller
# L2+B → L2+UP → R1+X

# Step 2: Start gripper server on robot PC 192.168.123.164
ssh unitree@192.168.123.164
sudo ./dex1_1_gripper_server --network eth0

# Step 3: Run script on Techshare PC
conda activate unitree_sim_env
cd /home/techshare/user/yokote/20260619/src
python drop_to_basket_real.py --iface enp8s0
```

### Kinesthetic Teaching (record new positions)
```bash
conda activate unitree_sim_env
cd /home/techshare/user/yokote/20260619/src
python g1_teach_pose.py --iface enp8s0
```
