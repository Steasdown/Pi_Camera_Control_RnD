from __future__ import annotations

import argparse
import time
from pathlib import Path
from typing import Optional, Tuple

import cv2

from .config import AppConfig, load_config
from .imx500_adapter import build_detections, filter_by_class, normalize_ssd_outputs
from .overlay import draw_overlay
from .serial_reader import SerialReader


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--config", default="configs/runtime.json")
    p.add_argument("--debug", action="store_true")
    p.add_argument("--run-c2", action="store_true")
    p.add_argument("--run-seconds", type=float, default=0.0)
    p.add_argument("--view-size", default="1270x720")
    return p.parse_args()


def _get_outputs(imx500, metadata):
    try:
        return imx500.get_outputs(metadata, add_batch=True)
    except TypeError:
        return imx500.get_outputs(metadata)


def _rect_to_xywh(obj, frame_w: int, frame_h: int) -> Tuple[int, int, int, int]:
    # picamera2 versions return different rect types
    if hasattr(obj, "x") and hasattr(obj, "y") and hasattr(obj, "width") and hasattr(obj, "height"):
        x, y, w, h = int(obj.x), int(obj.y), int(obj.width), int(obj.height)
    elif isinstance(obj, (tuple, list)) and len(obj) == 4:
        a, b, c, d = (float(obj[0]), float(obj[1]), float(obj[2]), float(obj[3]))
        # heuristic: treat as (x,y,w,h) if sums are plausible
        if (a + c) <= (frame_w * 1.2) and (b + d) <= (frame_h * 1.2):
            x, y, w, h = int(a), int(b), int(c), int(d)
        else:
            x1, y1, x2, y2 = a, b, c, d
            x, y, w, h = int(x1), int(y1), int(x2 - x1), int(y2 - y1)
    else:
        raise TypeError(f"Unsupported rect type: {type(obj).__name__} {obj!r}")

    # clamp
    x = max(0, min(x, frame_w - 1))
    y = max(0, min(y, frame_h - 1))
    w = max(1, min(w, frame_w - x))
    h = max(1, min(h, frame_h - y))
    return x, y, w, h


def _pan_command_from_dx(cfg: AppConfig, dx_px: float) -> float:
    # deadband
    if abs(dx_px) <= float(cfg.pan_deadband_px):
        return 0.0

    spd = float(cfg.pan_kp) * float(dx_px)
    if cfg.pan_invert:
        spd = -spd

    # enforce min magnitude outside deadband
    if spd > 0:
        spd = max(spd, float(cfg.pan_min_speed))
    else:
        spd = min(spd, -float(cfg.pan_min_speed))

    # cap
    spd = max(-float(cfg.pan_max_speed), min(float(cfg.pan_max_speed), spd))
    return spd


def _run_c2(cfg: AppConfig, debug: bool, run_seconds: float, view_size: str) -> int:
    try:
        from picamera2 import Picamera2
        from picamera2.devices.imx500 import IMX500
    except Exception as e:
        print("[C2] Picamera2/IMX500 not available.")
        print(f"[C2] {type(e).__name__}: {e}")
        return 2

    # view size
    try:
        w_str, h_str = view_size.lower().split("x")
        frame_w, frame_h = int(w_str), int(h_str)
        if frame_w <= 0 or frame_h <= 0:
            raise ValueError
    except Exception:
        print(f"[C2] Bad --view-size '{view_size}' (use WxH).")
        return 2

    print("[C2] starting")
    print(f"[C2] model_path: {cfg.model_path}")
    print(f"[C2] threshold: {cfg.score_threshold}")
    print(f"[C2] serial: {cfg.serial_port}@{cfg.serial_baud}")
    print(f"[C2] view: {frame_w}x{frame_h}")

    # Serial
    ser = SerialReader(cfg.serial_port, cfg.serial_baud, keep_lines=50)
    ser.start()

    # Camera
    imx500 = IMX500(cfg.model_path)
    picam2 = Picamera2(imx500.camera_num)

    # Use RGB888 from camera, convert to BGR once for OpenCV
    cam_cfg = picam2.create_preview_configuration(
        main={"size": (frame_w, frame_h), "format": "RGB888"},
        buffer_count=6,
    )
    picam2.configure(cam_cfg)

    window_name = "Pi AI RnD (C2)"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    # latch last detection briefly to avoid “blink”
    hold_s = 0.25
    last_det_t = -1e9
    last_bbox: Optional[Tuple[int, int, int, int]] = None
    last_score = 0.0
    last_dx = 0.0

    # PANSPD send rate limit
    send_min_dt = 1.0 / max(1.0, float(cfg.pan_send_hz))
    last_send_t = 0.0
    last_sent_speed: Optional[float] = None

    deadline = time.monotonic() + float(run_seconds) if run_seconds > 0 else float("inf")
    frames = 0

    try:
        picam2.start()

        while time.monotonic() < deadline:
            req = picam2.capture_request()
            try:
                frames += 1
                metadata = req.get_metadata()
                frame_rgb = req.make_array("main")
                outputs = _get_outputs(imx500, metadata)
                norm = normalize_ssd_outputs(outputs)
            finally:
                req.release()

            # OpenCV expects BGR
            frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

            now_t = time.monotonic()

            # update detection
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
                    # largest bbox by area in pixel space (after conversion)
                    best_bbox = None
                    best_score = 0.0
                    best_area = -1

                    for d in persons:
                        rect = imx500.convert_inference_coords(d.box, metadata, picam2)
                        x, y, w, h = _rect_to_xywh(rect, frame_w, frame_h)
                        area = w * h
                        if area > best_area:
                            best_area = area
                            best_bbox = (x, y, w, h)
                            best_score = float(d.score)

                    if best_bbox is not None:
                        x, y, w, h = best_bbox
                        bbox_cx = x + w / 2.0
                        frame_cx = frame_w / 2.0
                        last_dx = bbox_cx - frame_cx

                        last_bbox = best_bbox
                        last_score = best_score
                        last_det_t = now_t

            # detection latch expiry
            bbox_for_draw = last_bbox if (last_bbox is not None and (now_t - last_det_t) <= hold_s) else None
            dx_for_draw = last_dx if bbox_for_draw is not None else 0.0
            score_for_draw = last_score if bbox_for_draw is not None else 0.0

            # PI_CONTROL detect + indicator
            pi_control = ser.is_pi_control()
            serial_lines = ser.get_last_lines(3)

            # Tracking -> PANSPD (ONLY when Arduino is in PI_CONTROL)
            cmd_speed = 0.0
            if pi_control and bbox_for_draw is not None:
                cmd_speed = _pan_command_from_dx(cfg, dx_for_draw)

            # send command (rate-limited + avoid duplicates)
            if pi_control:
                if (now_t - last_send_t) >= send_min_dt:
                    to_send: Optional[str] = None

                    # STOP in deadband
                    if cmd_speed == 0.0:
                        # only send STOP once if previously moving
                        if last_sent_speed is None or abs(last_sent_speed) > 1e-3:
                            to_send = "STOP"
                            last_sent_speed = 0.0
                    else:
                        # PANSPD value
                        # only send if materially changed
                        if last_sent_speed is None or abs(cmd_speed - last_sent_speed) >= 10.0:
                            to_send = f"PANSPD {cmd_speed:.1f}"
                            last_sent_speed = cmd_speed

                    if to_send:
                        ser.write_line(to_send)
                        if debug:
                            print(f"[TRACK] dx={dx_for_draw:+.1f}px -> {to_send}")

                    last_send_t = now_t
            else:
                # if not in PI_CONTROL, don’t push commands
                last_sent_speed = None

            # draw GUI
            draw_overlay(
                frame_bgr,
                pi_control=pi_control,
                bbox_xywh=bbox_for_draw,
                score=score_for_draw,
                dx_px=dx_for_draw,
                serial_lines=serial_lines,
            )

            cv2.imshow(window_name, frame_bgr)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

        return 0

    finally:
        try:
            picam2.stop()
            picam2.close()
        except Exception:
            pass
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        try:
            ser.stop()
        except Exception:
            pass


def main() -> int:
    args = parse_args()
    cfg: AppConfig = load_config(Path(args.config))

    if args.run_c2:
        return _run_c2(cfg, args.debug, args.run_seconds, args.view_size)

    print("Use --run-c2")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
