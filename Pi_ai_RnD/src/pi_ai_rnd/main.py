#!/usr/bin/env python3
"""pi_ai_rnd.main — Prototype A

Prototype A goals:
- Open IMX500 camera pipeline
- Capture ONE request
- Read inference outputs from request metadata
- Print output tensor shapes ONCE
- Exit cleanly

Notes:
- This will NOT run on Windows (no Picamera2/IMX500 there). That’s expected.
- It is guarded so PC runs will fail gracefully with a clear message.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any, Iterable

from .config import AppConfig, load_config


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Pi AI Camera RnD — Stage 1")
    p.add_argument("--config", default="configs/runtime.json",
                   help="Optional runtime config JSON (not required yet).")
    p.add_argument("--debug", action="store_true", help="Verbose debug output.")
    p.add_argument(
        "--probe-imx500",
        action="store_true",
        help="Prototype A: open camera and print IMX500 output tensor shapes once, then exit.",
    )
    p.add_argument("--probe-timeout-s", type=float, default=120.0,
                   help="Max seconds to wait for IMX500 outputs (Prototype A).")
    p.add_argument("--probe-poll-hz", type=float, default=10.0,
                   help="How often to sample requests while waiting (Prototype A).")

    return p.parse_args()


def _shape(x: Any) -> str:
    # Robust shape printer for numpy arrays / lists / scalars
    s = getattr(x, "shape", None)
    if s is not None:
        return str(s)
    if isinstance(x, (list, tuple)):
        return f"list(len={len(x)})"
    return f"{type(x).__name__}"


def _probe_imx500_once(cfg: AppConfig, debug: bool, timeout_s: float, poll_hz: float) -> int:
    """Open IMX500 camera, wait for inference outputs, print output shapes once, exit."""
    try:
        # Pi-only imports
        from picamera2 import Picamera2
        from picamera2.devices.imx500 import IMX500
    except Exception as e:
        print("[probe] Picamera2/IMX500 not available on this machine.")
        print(f"[probe] {type(e).__name__}: {e}")
        return 2

    import time

    model_path = cfg.model_path
    print(f"[probe] model_path: {model_path}")

    # Must create IMX500 before Picamera2
    imx500 = IMX500(model_path)
    picam2 = Picamera2(imx500.camera_num)

    # No preview window needed for Prototype A; we just need requests + metadata.
    config = picam2.create_preview_configuration(
        main={"size": (640, 480), "format": "RGB888"},
        buffer_count=4,
    )
    picam2.configure(config)

    if debug:
        print("[probe] configured preview pipeline")
        print(f"[probe] camera_num: {imx500.camera_num}")

    period_s = 1.0 / max(1.0, float(poll_hz))
    deadline = time.monotonic() + float(timeout_s)
    tries = 0
    last_status = 0.0

    try:
        picam2.start()

        while time.monotonic() < deadline:
            tries += 1

            req = picam2.capture_request()
            try:
                metadata = req.get_metadata()

                # API differs across versions; try add_batch=True then fallback.
                try:
                    outputs = imx500.get_outputs(metadata, add_batch=True)
                except TypeError:
                    outputs = imx500.get_outputs(metadata)

                if outputs is not None:
                    # Print shapes in a deterministic way, then exit.
                    if isinstance(outputs, dict):
                        print("[probe] outputs: dict")
                        for k in sorted(outputs.keys()):
                            print(f"[probe]  {k}: shape={_shape(outputs[k])}")
                    elif isinstance(outputs, (list, tuple)):
                        print(f"[probe] outputs: {type(outputs).__name__}(len={len(outputs)})")
                        for i, o in enumerate(outputs):
                            print(f"[probe]  [{i}] shape={_shape(o)} type={type(o).__name__}")
                    else:
                        print(f"[probe] outputs: {type(outputs).__name__} shape={_shape(outputs)}")

                    print(f"[probe] success after {tries} request(s)")
                    return 0

            finally:
                req.release()

            # Status heartbeat every ~2s so you can see progress over SSH
            now = time.monotonic()
            if debug and (now - last_status) > 2.0:
                remaining = max(0.0, deadline - now)
                print(f"[probe] waiting for outputs... tries={tries} remaining_s≈{remaining:.0f}")
                last_status = now

            time.sleep(period_s)

        print(f"[probe] timeout: no inference outputs after {tries} request(s) in {timeout_s:.1f}s")
        return 3

    finally:
        picam2.stop()
        picam2.close()
        if debug:
            print("[probe] camera stopped/closed")


def main() -> int:
    args = parse_args()
    cfg: AppConfig = load_config(Path(args.config))

    if args.probe_imx500:
        return _probe_imx500_once(cfg, args.debug, args.probe_timeout_s, args.probe_poll_hz)


    # Default behavior (still scaffold)
    print("Pi AI Camera RnD — Stage 1 scaffold running.")
    print(f"Config path: {args.config}")
    print(f"Model path (planned): {cfg.model_path}")
    print(f"Target class (planned): {cfg.target_class_name} (id={cfg.target_class_id})")
    print(f"Threshold (planned): {cfg.score_threshold}")
    print("Use --probe-imx500 for Prototype A tensor-shape probe.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
