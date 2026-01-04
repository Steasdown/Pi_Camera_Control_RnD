#!/usr/bin/env python3
"""pi_ai_rnd.main

Stage1:
- Prototype B: IMX500 detections printed over SSH
- Prototype C2: OpenCV preview + bbox overlay

Stage2 / Step 1 (Arduino integration):
- Display raw Arduino serial output as an on-screen panel.
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path
from typing import Any, Optional

from .config import AppConfig, load_config
from .imx500_adapter import build_detections, filter_by_class, normalize_ssd_outputs
from .logging_utils import hz_to_dt
from .overlay import clamp_xywh, draw_text_panel
from .serial_reader import SerialPoller


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Pi AI Camera RnD")
    p.add_argument("--config", default="configs/runtime.json", help="Runtime config JSON")
    p.add_argument("--debug", action="store_true", help="Verbose debug output")

    # Prototype B (SSH prints)
    p.add_argument("--run-b", action="store_true", help="Prototype B: print detections")
    p.add_argument("--run-seconds", type=float, default=10.0, help="How long to run")

    # Prototype C2 (OpenCV window)
    p.add_argument("--run-c2", action="store_true", help="Prototype C2: OpenCV preview")
    p.add_argument("--view-size", default=None, help="Override preview size WxH (e.g. 1280x720)")

    # Stage2/Step1: Serial overrides (optional)
    p.add_argument("--serial", action="store_true", help="Enable serial monitor")
    p.add_argument("--no-serial", action="store_true", help="Disable serial monitor")
    p.add_argument("--serial-port", default=None, help="Serial port or 'auto'")
    p.add_argument("--serial-baud", type=int, default=None, help="Serial baud")
    p.add_argument("--serial-lines", type=int, default=None, help="Ring buffer size")

    return p.parse_args()


def _shape(x: Any) -> str:
    s = getattr(x, "shape", None)
    if s is not None:
        return str(s)
    if isinstance(x, (list, tuple)):
        return f"list(len={len(x)})"
    return f"{type(x).__name__}"


def _get_outputs(imx500, metadata):
    try:
        return imx500.get_outputs(metadata, add_batch=True)
    except TypeError:
        return imx500.get_outputs(metadata)


def _rect_to_xywh(obj, frame_w: int, frame_h: int):
    if hasattr(obj, "x") and hasattr(obj, "y") and hasattr(obj, "width") and hasattr(obj, "height"):
        return int(obj.x), int(obj.y), int(obj.width), int(obj.height)

    if isinstance(obj, (tuple, list)) and len(obj) == 4:
        a, b, c, d = (float(obj[0]), float(obj[1]), float(obj[2]), float(obj[3]))
        if (a + c) <= (frame_w * 1.2) and (b + d) <= (frame_h * 1.2):
            x, y, w, h = a, b, c, d
        else:
            x1, y1, x2, y2 = a, b, c, d
            x, y, w, h = x1, y1, (x2 - x1), (y2 - y1)
        return int(x), int(y), int(w), int(h)

    raise TypeError(f"Unsupported rect type: {type(obj).__name__} {obj!r}")


def _run_prototype_b(cfg: AppConfig, debug: bool, run_seconds: float) -> int:
    try:
        from picamera2 import Picamera2
        from picamera2.devices.imx500 import IMX500
    except Exception as e:
        print("[B] Picamera2/IMX500 not available.")
        print(f"[B] {type(e).__name__}: {e}")
        return 2

    frame_w, frame_h = cfg.view_w, cfg.view_h

    print("[B] starting")
    print(f"[B] model_path: {cfg.model_path}")
    print(f"[B] threshold: {cfg.score_threshold}")
    print(f"[B] target_class_id: {cfg.target_class_id} ({cfg.target_class_name})")
    print(f"[B] run_seconds: {run_seconds}")

    imx500 = IMX500(cfg.model_path)
    picam2 = Picamera2(imx500.camera_num)

    config = picam2.create_preview_configuration(
        main={"size": (frame_w, frame_h), "format": cfg.camera_format},
        buffer_count=cfg.buffer_count,
    )
    picam2.configure(config)

    last_print = 0.0
    min_dt = hz_to_dt(cfg.print_hz)

    deadline = time.monotonic() + float(run_seconds)
    frames = 0
    printed = 0

    try:
        picam2.start()

        while time.monotonic() < deadline:
            req = picam2.capture_request()
            try:
                frames += 1
                metadata = req.get_metadata()
                outputs = _get_outputs(imx500, metadata)
                norm = normalize_ssd_outputs(outputs)
                if norm is None:
                    continue

                boxes, scores, classes, count = norm
                dets = build_detections(
                    boxes=boxes,
                    scores=scores,
                    classes=classes,
                    count=count,
                    threshold=cfg.score_threshold,
                )
                persons = filter_by_class(dets, cfg.target_class_id)

                now_t = time.monotonic()
                if persons and (now_t - last_print) >= min_dt:
                    d0 = max(persons, key=lambda d: d.score)
                    rect = imx500.convert_inference_coords(d0.box, metadata, picam2)
                    x, y, w, h = _rect_to_xywh(rect, frame_w, frame_h)
                    x, y, w, h = clamp_xywh(x, y, w, h, frame_w, frame_h)
                    cx, cy = x + w / 2.0, y + h / 2.0

                    print(
                        f"[B][person] score={d0.score:.2f} "
                        f"px=(x={x},y={y},w={w},h={h}) center=({cx:.1f},{cy:.1f}) raw={d0.box}"
                    )
                    printed += 1
                    last_print = now_t

                if debug and (frames % 60 == 0):
                    print(f"[B][debug] frames={frames} persons={len(persons)} count={count}")

            finally:
                req.release()

        print(f"[B] done: frames={frames} printed={printed}")
        return 0

    finally:
        picam2.stop()
        picam2.close()
        if debug:
            print("[B] camera stopped/closed")


def _run_prototype_c2(cfg: AppConfig, debug: bool, run_seconds: float, view_size: Optional[str]) -> int:
    try:
        import cv2
        import numpy as np
        from picamera2 import Picamera2
        from picamera2.devices.imx500 import IMX500
    except Exception as e:
        print("[C2] Required modules not available.")
        print(f"[C2] {type(e).__name__}: {e}")
        return 2

    frame_w, frame_h = cfg.view_w, cfg.view_h
    if view_size:
        try:
            w_str, h_str = view_size.lower().split("x")
            frame_w, frame_h = int(w_str), int(h_str)
            if frame_w <= 0 or frame_h <= 0:
                raise ValueError
        except Exception:
            print(f"[C2] Bad --view-size '{view_size}'. Use e.g. 1280x720")
            return 2

    print("[C2] starting")
    print(f"[C2] model_path: {cfg.model_path}")
    print(f"[C2] threshold: {cfg.score_threshold}")
    print(f"[C2] target_class_id: {cfg.target_class_id} ({cfg.target_class_name})")
    print(f"[C2] view: {frame_w}x{frame_h}")
    print("[C2] press 'q' in the window to quit")

    imx500 = IMX500(cfg.model_path)
    picam2 = Picamera2(imx500.camera_num)

    config = picam2.create_preview_configuration(
        main={"size": (frame_w, frame_h), "format": "RGB888"},
        buffer_count=max(4, int(cfg.buffer_count)),
    )
    picam2.configure(config)

    window_name = "Pi AI RnD (C2)"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    poller = SerialPoller(
        enabled=cfg.serial_enabled,
        port=cfg.serial_port,
        baud=cfg.serial_baud,
        max_lines=cfg.serial_lines,
        reconnect_s=cfg.serial_reconnect_s,
    )

    last_print = 0.0
    min_dt = hz_to_dt(cfg.print_hz)

    deadline = time.monotonic() + float(run_seconds) if run_seconds > 0 else float("inf")
    frames = 0
    printed = 0

    hold_s = 0.25
    last_seen_t = -1e9
    last_score = 0.0
    last_xywh = None

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

            serial_status, serial_lines = poller.poll()
            draw_text_panel(
                frame,
                title="Arduino serial (raw)",
                status=serial_status,
                lines=serial_lines,
                max_lines=cfg.serial_lines,
            )

            now_t = time.monotonic()

            if norm is not None:
                boxes, scores, classes, count = norm
                dets = build_detections(
                    boxes=boxes,
                    scores=scores,
                    classes=classes,
                    count=count,
                    threshold=cfg.score_threshold,
                )
                persons = filter_by_class(dets, cfg.target_class_id)
                if persons:
                    d0 = max(persons, key=lambda d: d.score)
                    rect = imx500.convert_inference_coords(d0.box, metadata, picam2)
                    x, y, w, h = _rect_to_xywh(rect, frame_w, frame_h)
                    x, y, w, h = clamp_xywh(x, y, w, h, frame_w, frame_h)

                    last_xywh = (x, y, w, h)
                    last_score = float(d0.score)
                    last_seen_t = now_t

                    if (now_t - last_print) >= min_dt:
                        cx, cy = x + w / 2.0, y + h / 2.0
                        print(
                            f"[C2][person] score={last_score:.2f} "
                            f"px=(x={x},y={y},w={w},h={h}) center=({cx:.1f},{cy:.1f})"
                        )
                        printed += 1
                        last_print = now_t

            if last_xywh is not None and (now_t - last_seen_t) <= hold_s:
                x, y, w, h = last_xywh
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

        print(f"[C2] done: frames={frames} printed={printed}")
        return 0

    finally:
        poller.close()
        picam2.stop()
        picam2.close()
        cv2.destroyAllWindows()
        if debug:
            print("[C2] camera stopped/closed")


def main() -> int:
    args = parse_args()
    cfg: AppConfig = load_config(Path(args.config))

    # CLI overrides for serial settings
    if args.serial:
        cfg.serial_enabled = True
    if args.no_serial:
        cfg.serial_enabled = False
    if args.serial_port is not None:
        cfg.serial_port = args.serial_port
    if args.serial_baud is not None:
        cfg.serial_baud = int(args.serial_baud)
    if args.serial_lines is not None:
        cfg.serial_lines = int(args.serial_lines)

    if args.run_b:
        return _run_prototype_b(cfg, args.debug, args.run_seconds)

    if args.run_c2:
        return _run_prototype_c2(cfg, args.debug, args.run_seconds, args.view_size)

    print("Pi AI Camera RnD")
    print("Use --run-b for Prototype B, or --run-c2 for Prototype C2")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
