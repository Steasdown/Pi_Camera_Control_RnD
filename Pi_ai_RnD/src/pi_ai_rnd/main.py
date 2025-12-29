#!/usr/bin/env python3
"""pi_ai_rnd.main — Stage 1 entry point (scaffold only)

This file intentionally does **not** access camera hardware yet.
It exists so:
- the repo structure can be validated on the Pi,
- CLI parsing/config loading patterns are locked in early.

Later prototypes will add, in order:
A) open camera + print output tensor shapes once
B) filter to person only + print bbox coords + confidence
C) draw red bbox overlay
D) add optional CSV logging + stable config

"""

from __future__ import annotations

import argparse
from pathlib import Path

from .config import AppConfig, load_config


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Pi AI Camera RnD — Stage 1 (scaffold)")
    p.add_argument("--config", default="configs/runtime.json",
                   help="Optional runtime config JSON (not required yet).")
    p.add_argument("--debug", action="store_true", help="Verbose debug output.")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    cfg: AppConfig = load_config(Path(args.config))

    print("Pi AI Camera RnD — Stage 1 (Path 3) scaffold running.")
    print(f"Config path: {args.config}")
    print(f"Model path (planned): {cfg.model_path}")
    print(f"Target class (planned): {cfg.target_class_name} (id={cfg.target_class_id})")
    print(f"Threshold (planned): {cfg.score_threshold}")

    if args.debug:
        print("[debug] This build does NOT start the camera yet.")
        print("[debug] Next commit: Prototype A (camera open + tensor shapes once).")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
