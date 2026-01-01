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
    return p.parse_args()


def _shape(x: Any) -> str:
    # Robust shape printer for numpy arrays / lists / scalars
    s = getattr(x, "shape", None)
    if s is not None:
        return str(s)
    if isinstance(x, (list, tuple)):
        return f"list(len={len(x)})"
    return f"{type(x).__name__}"


def _probe_imx500_once(cfg: AppConfig, debug: bool) -> int:
    """Open IMX500 camera, capture one request, print output shapes, exit."""
    try:
        # Pi-only imports
        from picamera2 import Picamera2
        from picamera2.devices.imx500 import IMX500
    except Exception as e:
        print("[probe] Picamera2/IMX500 not available on this machine.")
        print(f"[probe] {type(e).__name__}: {e}")
        return 2

    model_path = cfg.model_path
    print(f"[probe] model_path: {model_path}")

    # Must create IMX500 before Picamera2
    imx500 = IMX500(model_path)
    picam2 = Picamera2(imx500.camera_num)

    # No preview window needed for Prototype A; we just need one request.
    # Use a small, valid stream format.
    config = picam2.create_preview_configuration(
        main={"size": (640, 480), "format": "RGB888"},
        buffer_count=2,
    )

    picam2.configure(config)

    if debug:
        print("[probe] configured preview pipeline")
        print(f"[probe] camera_num: {imx500.camera_num}")

    try:
        picam2.start()
        # Capture one request to access metadata
        req = picam2.capture_request()
        try:
            metadata = req.get_metadata()

            # API differs across versions; try add_batch=True then fallback.
            try:
                outputs = imx500.get_outputs(metadata, add_batch=True)
            except TypeError:
                outputs = imx500.get_outputs(metadata)

            if outputs is None:
                print("[probe] outputs: None (no inference outputs found in metadata)")
                return 3

            # Print shapes in a deterministic way
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

            return 0

        finally:
            req.release()

    finally:
        picam2.stop()
        picam2.close()
        if debug:
            print("[probe] camera stopped/closed")


def main() -> int:
    args = parse_args()
    cfg: AppConfig = load_config(Path(args.config))

    if args.probe_imx500:
        return _probe_imx500_once(cfg, args.debug)

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
