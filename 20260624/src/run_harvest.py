#!/usr/bin/env python3
"""
run_harvest.py — One command to run Isaac Sim + okra harvest together.

Usage:
    conda activate unitree_sim_env
    cd /home/techshare/user/yokote/20260624/src
    python run_harvest.py

    # Use full sequence script instead of demo:
    python run_harvest.py --script drop_to_basket_isaac.py

    # Skip pick phase (okra already in gripper):
    python run_harvest.py --skip-pick

What it does:
    1. Starts Isaac Sim in the background
    2. Waits until Isaac Sim shared memory is ready  (~30-60s)
    3. Waits a few more seconds for physics to settle
    4. Runs the harvest motion script automatically
    5. Isaac Sim keeps running after harvest (Ctrl+C to stop everything)
"""
from __future__ import annotations

import argparse
import subprocess
import sys
import time
from multiprocessing import shared_memory
from pathlib import Path

# ── Paths ──────────────────────────────────────────────────────────────────────
ISAAC_DIR      = Path("/home/techshare/xr_teleoperate/unitree_sim_isaaclab")
SRC_DIR        = Path(__file__).resolve().parent
DEFAULT_SCRIPT = SRC_DIR / "demo_okra_harvest.py"

# ── Isaac Sim launch command ───────────────────────────────────────────────────
ISAAC_CMD = [
    sys.executable, "sim_main.py",
    "--task",           "Isaac-G1-Custom-Room-Joint",
    "--action_source",  "dds",
    "--enable_dex1_dds",
    "--robot_type",     "g129",
    "--enable_cameras",
]

SHM_NAME  = "dds_robot_cmd"
SETTLE_S  = 5.0    # extra wait after shm appears (physics settling)
TIMEOUT_S = 180.0  # max seconds to wait for Isaac Sim to start


# ── Helpers ────────────────────────────────────────────────────────────────────

def wait_for_shm(name: str, timeout: float) -> bool:
    """Poll until the named shared memory segment appears."""
    start = time.monotonic()
    while time.monotonic() - start < timeout:
        try:
            shm = shared_memory.SharedMemory(name=name)
            shm.close()
            return True
        except FileNotFoundError:
            print(".", end="", flush=True)
            time.sleep(2)
    return False


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Launch Isaac Sim + okra harvest in one command."
    )
    p.add_argument(
        "--script", type=Path, default=DEFAULT_SCRIPT,
        help="harvest script to run (default: demo_okra_harvest.py)",
    )
    p.add_argument(
        "--skip-pick", action="store_true",
        help="pass --skip-pick to the harvest script",
    )
    p.add_argument(
        "--settle", type=float, default=SETTLE_S,
        help=f"extra seconds to wait after Isaac Sim is ready (default: {SETTLE_S})",
    )
    return p.parse_args()


# ── Main ───────────────────────────────────────────────────────────────────────

def main() -> None:
    args = parse_args()

    script = args.script.resolve()
    if not script.exists():
        print(f"ERROR: script not found: {script}")
        sys.exit(1)

    # ── Step 1: start Isaac Sim ────────────────────────────────────────────────
    print("=" * 60)
    print("  STEP 1: Starting Isaac Sim (background)")
    print("=" * 60)
    print(f"  Directory : {ISAAC_DIR}")
    print(f"  Command   : {' '.join(ISAAC_CMD)}")
    print()

    sim_proc = subprocess.Popen(ISAAC_CMD, cwd=str(ISAAC_DIR))
    print(f"  Isaac Sim PID: {sim_proc.pid}")

    # ── Step 2: wait for shared memory ────────────────────────────────────────
    print()
    print(f"  STEP 2: Waiting for Isaac Sim shared memory '{SHM_NAME}'")
    print(f"          (timeout {TIMEOUT_S:.0f}s — Isaac Sim takes ~30-60s to load)")
    print("  Progress: ", end="", flush=True)

    if not wait_for_shm(SHM_NAME, timeout=TIMEOUT_S):
        print(f"\n\nERROR: Isaac Sim did not start within {TIMEOUT_S:.0f}s.")
        print("  Check Isaac Sim terminal for errors.")
        sim_proc.terminate()
        sys.exit(1)

    print(" ready!")

    # ── Step 3: settle ────────────────────────────────────────────────────────
    print()
    print(f"  STEP 3: Waiting {args.settle:.0f}s for physics to settle...")
    for i in range(int(args.settle)):
        time.sleep(1)
        print(f"    {i + 1}/{int(args.settle)}s", end="\r", flush=True)
    print()

    # ── Step 4: run harvest script ────────────────────────────────────────────
    harvest_cmd = [sys.executable, str(script)]
    if args.skip_pick:
        harvest_cmd.append("--skip-pick")

    print()
    print("=" * 60)
    print("  STEP 4: Running harvest motion")
    print("=" * 60)
    print(f"  Script: {script.name}")
    print()

    try:
        subprocess.run(harvest_cmd, cwd=str(SRC_DIR), check=False)
    except KeyboardInterrupt:
        print("\n  Harvest stopped.")

    # ── Done ──────────────────────────────────────────────────────────────────
    print()
    print("=" * 60)
    print("  Harvest motion complete.")
    print("  Isaac Sim is still running — press Ctrl+C to stop it.")
    print("=" * 60)

    try:
        sim_proc.wait()
    except KeyboardInterrupt:
        print("\n  Stopping Isaac Sim...")
        sim_proc.terminate()
        sim_proc.wait()
        print("  Done.")


if __name__ == "__main__":
    main()
