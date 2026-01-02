#!/usr/bin/env python3
"""pi_ai_rnd.main — Prototype B

Prototype B goals:
- Open IMX500 camera pipeline
- Continuously read IMX500 outputs from request metadata
- Parse outputs using pi_ai_rnd.imx500_adapter
- Filter to PERSON only (class id from config, default 0)
- Print bbox + confidence at a rate limit (SSH-friendly)
- Exit cleanly after --run-seconds

Notes:
- No preview window yet (keeps it simple and stable over SSH).
- Boxes printed are still "raw inference box 4-vector" until Prototype C,
  where we will convert to output pixel coords and draw overlays.
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path
from typing import Any

from .config import AppConfig, load_config
from .imx500_adapter import normalize_ssd_outputs, build_detections, filter_by_class


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Pi AI Camera RnD — Stage 1")
    p.add_argument("--config", default="configs/runtime.json",
                   help="Optional runtime config JSON (not required yet).")
    p.add_argument("--debug", action="store_true", help="Verbose debug output.")

    # Prototype A
    p.add_argument("--probe-imx500", action="store_true",
                   help="Prototype A: print IMX500 output tensor shapes once, then exit.")
    p.add_argument("--probe-timeout-s", type=float, default=180.0,
                   help="Max seconds to wait for IMX500 outputs (Prototype A).")
    p.add_argument("--probe-poll-hz", type=float, default=10.0,
                   help="How often to sample requests while waiting (Prototype A).")

    # Prototype B
    p.add_argument("--run-b", action="store_true",
                   help="Prototype B: print person detections for --run-seconds then exit.")
    p.add_argument("--run-seconds", type=float, default=10.0,
                   help="How long to run Prototype B loop.")
    
    # Prototype C
    p.add_argument("--run-c2", action="store_true",
               help="Prototype C2: show a live window + red person bbox overlay.")
    p.add_argument("--view-size", default="1920x1080",
               help="Window stream size WxH, e.g. 1920x1080.")
    return p.parse_args()


def _shape(x: Any) -> str:
    s = getattr(x, "shape", None)
    if s is not None:
        return str(s)
    if isinstance(x, (list, tuple)):
        return f"list(len={len(x)})"
    return f"{type(x).__name__}"


def _get_outputs(imx500, metadata):
    # API differs across versions
    try:
        return imx500.get_outputs(metadata, add_batch=True)
    except TypeError:
        return imx500.get_outputs(metadata)


def _probe_imx500_once(cfg: AppConfig, debug: bool, timeout_s: float, poll_hz: float) -> int:
    """Prototype A: wait for outputs, print shapes once, exit."""
    try:
        from picamera2 import Picamera2
        from picamera2.devices.imx500 import IMX500
    except Exception as e:
        print("[probe] Picamera2/IMX500 not available on this machine.")
        print(f"[probe] {type(e).__name__}: {e}")
        return 2

    print(f"[probe] model_path: {cfg.model_path}")

    imx500 = IMX500(cfg.model_path)
    picam2 = Picamera2(imx500.camera_num)

    config = picam2.create_preview_configuration(
        main={"size": (1920, 1080), "format": "RGB888"},
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
                outputs = _get_outputs(imx500, metadata)
                if outputs is not None:
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
            
def _rect_to_xywh(obj, frame_w: int, frame_h: int):
    """Accept IMX500 convert_inference_coords return types and normalize to (x,y,w,h).

    Supports:
    - object with attributes: x, y, width, height
    - tuple/list: (x,y,w,h) OR (x1,y1,x2,y2) (heuristic)
    """
    # Attribute-style object
    if hasattr(obj, "x") and hasattr(obj, "y") and hasattr(obj, "width") and hasattr(obj, "height"):
        return int(obj.x), int(obj.y), int(obj.width), int(obj.height)

    # Tuple/list style
    if isinstance(obj, (tuple, list)) and len(obj) == 4:
        a, b, c, d = (float(obj[0]), float(obj[1]), float(obj[2]), float(obj[3]))

        # Heuristic:
        # If (a+c, b+d) is within frame bounds, treat as (x,y,w,h).
        # Otherwise treat as (x1,y1,x2,y2).
        if (a + c) <= (frame_w * 1.2) and (b + d) <= (frame_h * 1.2):
            x, y, w, h = a, b, c, d
        else:
            x1, y1, x2, y2 = a, b, c, d
            x, y, w, h = x1, y1, (x2 - x1), (y2 - y1)

        return int(x), int(y), int(w), int(h)

    raise TypeError(f"Unsupported rect type from convert_inference_coords: {type(obj).__name__} {obj!r}")


def _run_prototype_b(cfg: AppConfig, debug: bool, run_seconds: float) -> int:
    """Prototype B: person-only prints (no overlay), then exit."""
    try:
        from picamera2 import Picamera2
        from picamera2.devices.imx500 import IMX500
    except Exception as e:
        print("[B] Picamera2/IMX500 not available on this machine.")
        print(f"[B] {type(e).__name__}: {e}")
        return 2

    print("[B] starting")
    print(f"[B] model_path: {cfg.model_path}")
    print(f"[B] threshold: {cfg.score_threshold}")
    print(f"[B] target_class_id: {cfg.target_class_id} ({cfg.target_class_name})")
    print(f"[B] run_seconds: {run_seconds}")

    imx500 = IMX500(cfg.model_path)
    picam2 = Picamera2(imx500.camera_num)

    # No preview window; keep main stream small
    config = picam2.create_preview_configuration(
        main={"size": (frame_w, frame_h), "format": "BGR888"},
        buffer_count=6,
    )

    picam2.configure(config)

    # Rate limit prints for SSH
    last_print = 0.0
    min_dt = 1.0 / max(0.1, float(cfg.print_hz))

    deadline = time.monotonic() + float(run_seconds)
    frames = 0
    detections_printed = 0

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

                # Print at most cfg.print_hz
                now = time.monotonic()
                if persons and (now - last_print) >= min_dt:
                    # Print the highest-score person only (stable/low-noise)
                    persons_sorted = sorted(persons, key=lambda d: d.score, reverse=True)
                    d0 = persons_sorted[0]
                    # C1: convert inference coords -> pixel coords (main stream)
                    obj = imx500.convert_inference_coords(d0.box, metadata, picam2)
                    # Our main stream is 640x480 for Prototype B/C1
                    frame_w, frame_h = 640, 480
                    x, y, w, h = _rect_to_xywh(obj, frame_w, frame_h)

                    cx, cy = x + w / 2.0, y + h / 2.0

                    print(
                        f"[C1][person] score={d0.score:.2f} "
                        f"px=(x={x},y={y},w={w},h={h}) center=({cx:.1f},{cy:.1f}) "
                        f"raw={d0.box}"
                    )
                    detections_printed += 1
                    last_print = now

                    if debug:
                        print(f"[C1][debug] frames={frames} persons={len(persons)} count={count}")

            finally:
                req.release()

        print(f"[B] done: frames={frames} printed={detections_printed}")
        return 0

    finally:
        picam2.stop()
        picam2.close()
        if debug:
            print("[B] camera stopped/closed")
            
def _run_prototype_c2(cfg: AppConfig, debug: bool, run_seconds: float, view_size: str) -> int:
    """Prototype C2: OpenCV window with red person bbox overlay (single-request aligned).
    Improvements:
    - Draw bbox EVERY frame using "latched" last detection (hold_s).
    - Smooth bbox using EMA (alpha) to reduce jitter.
    - Keep terminal prints rate-limited (cfg.print_hz).
    """
    try:
        import cv2
        from picamera2 import Picamera2
        from picamera2.devices.imx500 import IMX500
    except Exception as e:
        print("[C2] Required modules not available.")
        print(f"[C2] {type(e).__name__}: {e}")
        return 2

    # Parse view size "WxH"
    try:
        w_str, h_str = view_size.lower().split("x")
        frame_w, frame_h = int(w_str), int(h_str)
        if frame_w <= 0 or frame_h <= 0:
            raise ValueError("non-positive size")
    except Exception:
        print(f"[C2] Bad --view-size '{view_size}'. Use e.g. 1280x720")
        return 2

    print("[C2] starting (OpenCV window)")
    print(f"[C2] model_path: {cfg.model_path}")
    print(f"[C2] threshold (config): {cfg.score_threshold}")
    print(f"[C2] target_class_id: {cfg.target_class_id} ({cfg.target_class_name})")
    print(f"[C2] view: {frame_w}x{frame_h}")
    print("[C2] press 'q' in the window to quit")

    imx500 = IMX500(cfg.model_path)
    picam2 = Picamera2(imx500.camera_num)

    # Use RGB stream; swap once for OpenCV BGR if your colours are still swapped.
    config = picam2.create_preview_configuration(
        main={"size": (frame_w, frame_h), "format": "RGB888"},
        buffer_count=6,
    )
    picam2.configure(config)

    window_name = "Pi AI RnD (C2)"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    # Rate-limit terminal prints (cfg.print_hz comes from config)
    last_print = 0.0
    min_dt = 1.0 / max(0.1, float(cfg.print_hz))

    deadline = time.monotonic() + float(run_seconds) if run_seconds > 0 else float("inf")
    frames = 0
    printed = 0

    # ---- smoothing / latching settings ----
    hold_s = 0.25          # keep last bbox visible for 250 ms after last detection
    alpha = 0.35           # EMA smoothing factor (0..1). Higher = more responsive, less smooth

    # Latched + smoothed bbox state (float coords)
    ema_xywh = None        # (x,y,w,h) floats
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

            now_t = time.monotonic()

            # Update latched bbox if we have a detection this frame
            if norm is not None:
                boxes, scores, classes, count = norm
                dets = build_detections(
                    boxes=boxes,
                    scores=scores,
                    classes=classes,
                    count=count,
                    threshold=cfg.score_threshold,  # ONLY threshold source
                )
                persons = filter_by_class(dets, cfg.target_class_id)
                if persons:
                    d0 = max(persons, key=lambda d: d.score)

                    rect = imx500.convert_inference_coords(d0.box, metadata, picam2)
                    x, y, w, h = _rect_to_xywh(rect, frame_w, frame_h)

                    # Clamp
                    x = max(0, min(x, frame_w - 1))
                    y = max(0, min(y, frame_h - 1))
                    w = max(1, min(w, frame_w - x))
                    h = max(1, min(h, frame_h - y))

                    ema_xywh = ema_update(ema_xywh, (float(x), float(y), float(w), float(h)), alpha)
                    last_seen_t = now_t
                    last_score = float(d0.score)

                    # Rate-limited console print
                    if (now_t - last_print) >= min_dt:
                        cx, cy = ema_xywh[0] + ema_xywh[2] / 2.0, ema_xywh[1] + ema_xywh[3] / 2.0
                        print(
                            f"[C2][person] score={last_score:.2f} "
                            f"px=(x={int(ema_xywh[0])},y={int(ema_xywh[1])},w={int(ema_xywh[2])},h={int(ema_xywh[3])}) "
                            f"center=({cx:.1f},{cy:.1f})"
                        )
                        printed += 1
                        last_print = now_t

            # Draw EVERY frame while "held"
            if ema_xywh is not None and (now_t - last_seen_t) <= hold_s:
                x, y, w, h = (int(ema_xywh[0]), int(ema_xywh[1]), int(ema_xywh[2]), int(ema_xywh[3]))
                x = max(0, min(x, frame_w - 1))
                y = max(0, min(y, frame_h - 1))
                w = max(1, min(w, frame_w - x))
                h = max(1, min(h, frame_h - y))

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
        picam2.stop()
        picam2.close()
        cv2.destroyAllWindows()
        if debug:
            print("[C2] camera stopped/closed")




def main() -> int:
    args = parse_args()
    cfg: AppConfig = load_config(Path(args.config))

    if args.probe_imx500:
        return _probe_imx500_once(cfg, args.debug, args.probe_timeout_s, args.probe_poll_hz)

    if args.run_b:
        return _run_prototype_b(cfg, args.debug, args.run_seconds)
    
    if args.run_c2:
        return _run_prototype_c2(cfg, args.debug, args.run_seconds, args.view_size)


    print("Pi AI Camera RnD — Stage 1 scaffold running.")
    print(f"Config path: {args.config}")
    print(f"Model path (planned): {cfg.model_path}")
    print(f"Target class (planned): {cfg.target_class_name} (id={cfg.target_class_id})")
    print(f"Threshold (planned): {cfg.score_threshold}")
    print("Use --probe-imx500 for Prototype A, or --run-b for Prototype B.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
