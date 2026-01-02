#!/usr/bin/env python3
"""
pi_ai_rnd.main — V1 (C2-only)

V1 scope:
- Open IMX500 camera pipeline
- Read IMX500 SSD outputs from request metadata
- Filter PERSON only (class id from config)
- Show OpenCV window with red bbox overlay
- Print bbox + confidence rate-limited (SSH-friendly logs)
"""

from __future__ import annotations

import argparse
import os
import time
from pathlib import Path
from typing import Any, Tuple

import numpy as np

from .config import AppConfig, load_config
from .imx500_adapter import normalize_ssd_outputs, build_detections, filter_by_class
from .overlay import clamp_xywh
from .logging_utils import hz_to_dt, now


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Pi AI Camera RnD — V1 (C2-only)")
    p.add_argument("--config", default="configs/runtime.json", help="Runtime config JSON.")
    p.add_argument("--debug", action="store_true", help="Verbose debug output.")
    p.add_argument("--run-seconds", type=float, default=0.0, help="0 = run until 'q' pressed.")
    p.add_argument("--view-size", default=None, help="Override view size WxH, e.g. 1270x720.")
    return p.parse_args()


def _get_outputs(imx500, metadata):
    # API differs across versions
    try:
        return imx500.get_outputs(metadata, add_batch=True)
    except TypeError:
        return imx500.get_outputs(metadata)


def _parse_view_size(view_size: str) -> Tuple[int, int]:
    w_str, h_str = view_size.lower().split("x")
    w, h = int(w_str), int(h_str)
    if w <= 0 or h <= 0:
        raise ValueError("non-positive view size")
    return w, h


def _rect_to_xywh(obj: Any, frame_w: int, frame_h: int) -> Tuple[int, int, int, int]:
    """
    Accept IMX500 convert_inference_coords return types:
    - object with attributes: x, y, width, height
    - tuple/list: (x,y,w,h) OR (x1,y1,x2,y2) (heuristic)
    """
    if hasattr(obj, "x") and hasattr(obj, "y") and hasattr(obj, "width") and hasattr(obj, "height"):
        return int(obj.x), int(obj.y), int(obj.width), int(obj.height)

    if isinstance(obj, (tuple, list)) and len(obj) == 4:
        a, b, c, d = (float(obj[0]), float(obj[1]), float(obj[2]), float(obj[3]))
        # Heuristic:
        # If a+c and b+d are plausibly within frame -> treat as xywh else x1y1x2y2
        if (a + c) <= (frame_w * 1.2) and (b + d) <= (frame_h * 1.2):
            x, y, w, h = a, b, c, d
        else:
            x1, y1, x2, y2 = a, b, c, d
            x, y, w, h = x1, y1, (x2 - x1), (y2 - y1)
        return int(x), int(y), int(w), int(h)

    raise TypeError(f"Unsupported rect type: {type(obj).__name__} {obj!r}")


def run_v1(cfg: AppConfig, debug: bool, run_seconds: float, view_size_override: str | None) -> int:
    """
    V1 live window + bbox overlay.
    """
    try:
        import cv2
        from picamera2 import Picamera2
        from picamera2.devices.imx500 import IMX500
    except Exception as e:
        print("[V1] Required modules not available.")
        print(f"[V1] {type(e).__name__}: {e}")
        return 2

    if os.environ.get("DISPLAY", "") == "":
        # OpenCV window requires a local display (desktop session / VNC).
        print("[V1] No DISPLAY found. Start from Pi desktop/VNC (not pure headless SSH).")
        return 2

    view_size = view_size_override or cfg.view_size
    try:
        frame_w, frame_h = _parse_view_size(view_size)
    except Exception:
        print(f"[V1] Bad view_size '{view_size}'. Use e.g. 1270x720")
        return 2

    print("[V1] starting (OpenCV window)")
    print(f"[V1] model_path: {cfg.model_path}")
    print(f"[V1] threshold (config): {cfg.score_threshold}")
    print(f"[V1] target_class_id: {cfg.target_class_id} ({cfg.target_class_name})")
    print(f"[V1] view: {frame_w}x{frame_h}")
    print("[V1] press 'q' in the window to quit")

    imx500 = IMX500(cfg.model_path)
    picam2 = Picamera2(imx500.camera_num)

    # Camera outputs RGB; convert to BGR for OpenCV display (channel order fix, not colourisation).
    config = picam2.create_preview_configuration(
        main={"size": (frame_w, frame_h), "format": "RGB888"},
        buffer_count=6,
    )
    picam2.configure(config)

    window_name = "Pi AI RnD (V1)"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    min_dt = hz_to_dt(cfg.print_hz)
    last_print = 0.0

    deadline = now() + float(run_seconds) if run_seconds and run_seconds > 0 else float("inf")
    frames = 0
    printed = 0

    hold_s = float(cfg.hold_s)
    alpha = float(cfg.ema_alpha)

    ema_xywh = None  # (x,y,w,h) floats
    last_seen_t = -1e9
    last_score = 0.0

    def ema_update(prev, new, a: float):
        if prev is None:
            return new
        return (
            prev[0] * (1.0 - a) + new[0] * a,
            prev[1] * (1.0 - a) + new[1] * a,
            prev[2] * (1.0 - a) + new[2] * a,
            prev[3] * (1.0 - a) + new[3] * a,
        )

    try:
        picam2.start()

        while time.monotonic() < deadline:
            req = picam2.capture_request()
            try:
                frames += 1
                metadata = req.get_metadata()

                # Frame must be taken before release
                frame = req.make_array("main")

                outputs = _get_outputs(imx500, metadata)
                norm = normalize_ssd_outputs(outputs)
            finally:
                req.release()

            t = time.monotonic()

            # Update latched bbox if we have a detection this frame
            if norm is not None:
                boxes, scores, classes, count = norm
                dets = build_detections(
                    boxes=boxes,
                    scores=scores,
                    classes=classes,
                    count=count,
                    threshold=cfg.score_threshold,  # SINGLE threshold source
                )
                persons = filter_by_class(dets, cfg.target_class_id)
                if persons:
                    d0 = max(persons, key=lambda d: d.score)

                    rect = imx500.convert_inference_coords(d0.box, metadata, picam2)
                    x, y, w, h = _rect_to_xywh(rect, frame_w, frame_h)
                    x, y, w, h = clamp_xywh(x, y, w, h, frame_w, frame_h)

                    ema_xywh = ema_update(ema_xywh, (float(x), float(y), float(w), float(h)), alpha)
                    last_seen_t = t
                    last_score = float(d0.score)

                    if (t - last_print) >= min_dt:
                        cx = ema_xywh[0] + ema_xywh[2] / 2.0
                        cy = ema_xywh[1] + ema_xywh[3] / 2.0
                        print(
                            f"[V1][person] score={last_score:.2f} "
                            f"px=(x={int(ema_xywh[0])},y={int(ema_xywh[1])},w={int(ema_xywh[2])},h={int(ema_xywh[3])}) "
                            f"center=({cx:.1f},{cy:.1f})"
                        )
                        printed += 1
                        last_print = t

            # Draw latched bbox for hold_s seconds
            if ema_xywh is not None and (t - last_seen_t) <= hold_s:
                x, y, w, h = (int(ema_xywh[0]), int(ema_xywh[1]), int(ema_xywh[2]), int(ema_xywh[3]))
                x, y, w, h = clamp_xywh(x, y, w, h, frame_w, frame_h)

                # Red bbox (BGR)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                cv2.putText(
                    frame,
                    f"person {last_score:.2f}",
                    (x + 5, max(20, y - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 0, 255),
                    2,
                )

            cv2.imshow(window_name, frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

        print(f"[V1] done: frames={frames} printed={printed}")
        return 0

    finally:
        picam2.stop()
        picam2.close()
        cv2.destroyAllWindows()
        if debug:
            print("[V1] camera stopped/closed")


def main() -> int:
    args = parse_args()
    cfg: AppConfig = load_config(Path(args.config))
    return run_v1(cfg, args.debug, args.run_seconds, args.view_size)


if __name__ == "__main__":
    raise SystemExit(main())
