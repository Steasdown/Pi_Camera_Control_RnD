#!/usr/bin/env python3
"""
Stage 1 (Path C): people detection + bbox logging scaffolding
Architecture: capture -> infer -> decide -> act -> log  (keep stable for later stages)

- Overlay: red bounding box when detecting PERSON
- Output: prints bbox (x,y,w,h) + center + confidence
- Hooks included: CSV logging, distance proxy, hysteresis flag, future GPIO outputs
"""

from __future__ import annotations

import argparse
import csv
import os
import queue
import signal
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple, List

import cv2
import numpy as np
from picamera2 import MappedArray, Picamera2, Preview
from picamera2.devices.imx500 import IMX500  # must be instantiated before Picamera2

# --- Stage-1 defaults (MobileNet SSD on IMX500) ---
DEFAULT_MODEL = "/usr/share/imx500-models/imx500_network_ssd_mobilenetv2_fpnlite_320x320_pp.rpk"


@dataclass(frozen=True)
class Detection:
    category: int
    conf: float
    box: Tuple[int, int, int, int]  # x,y,w,h in ISP output coords


class CSVLogger:
    def __init__(self, csv_path: Optional[Path]):
        self.csv_path = csv_path
        self._fh = None
        self._writer = None

    def open(self) -> None:
        if not self.csv_path:
            return
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self._fh = open(self.csv_path, "a", newline="")
        self._writer = csv.writer(self._fh)
        if self._fh.tell() == 0:
            self._writer.writerow(["ts_unix", "conf", "x", "y", "w", "h"])

    def log_bbox(self, ts_unix: float, conf: float, box: Tuple[int, int, int, int]) -> None:
        if not self._writer:
            return
        x, y, w, h = box
        self._writer.writerow([f"{ts_unix:.6f}", f"{conf:.4f}", x, y, w, h])
        self._fh.flush()

    def close(self) -> None:
        if self._fh:
            self._fh.close()
            self._fh = None
            self._writer = None


class HysteresisFlag:
    """Stage-1 hook: separate trigger/clear thresholds to reduce flicker."""
    def __init__(self, trigger: float, clear: float):
        assert clear <= trigger, "clear should be <= trigger"
        self.trigger = trigger
        self.clear = clear
        self.state = False

    def update(self, value: float) -> bool:
        if not self.state and value >= self.trigger:
            self.state = True
        elif self.state and value <= self.clear:
            self.state = False
        return self.state


class DistanceProxy:
    """Stage-1 hook: distance proxy via bbox height or area; calibrate later."""
    def __init__(self, mode: str = "height"):
        self.mode = mode  # "height" or "area"

    def proxy(self, box: Tuple[int, int, int, int]) -> float:
        _, _, w, h = box
        return float(h) if self.mode == "height" else float(w * h)

    # TODO(Stage1): implement calibration table + interpolation (3–5 known distances)


def load_labels_fallback() -> List[str]:
    # COCO 80-class labels (common for SSD MobileNet pipelines)
    return [
        "person","bicycle","car","motorcycle","airplane","bus","train","truck","boat","traffic light",
        "fire hydrant","stop sign","parking meter","bench","bird","cat","dog","horse","sheep","cow",
        "elephant","bear","zebra","giraffe","backpack","umbrella","handbag","tie","suitcase","frisbee",
        "skis","snowboard","sports ball","kite","baseball bat","baseball glove","skateboard","surfboard","tennis racket","bottle",
        "wine glass","cup","fork","knife","spoon","bowl","banana","apple","sandwich","orange",
        "broccoli","carrot","hot dog","pizza","donut","cake","chair","couch","potted plant","bed",
        "dining table","toilet","tv","laptop","mouse","remote","keyboard","cell phone","microwave","oven",
        "toaster","sink","refrigerator","book","clock","vase","scissors","teddy bear","hair drier","toothbrush",
    ]


def _get_imx500_outputs(imx500: IMX500, metadata, add_batch: bool = True):
    """
    IMX500 outputs can vary by model / API version.
    This function returns outputs with minimal assumptions.
    """
    try:
        outputs = imx500.get_outputs(metadata, add_batch=add_batch)
    except TypeError:
        # Older signature
        outputs = imx500.get_outputs(metadata)

    return outputs


def _normalize_ssd_outputs(outputs):
    """
    Robustly normalize boxes/scores/classes to:
      boxes:  (N,4)
      scores: (N,)
      classes:(N,)
    Accepts a variety of nested/batched shapes.
    """
    if outputs is None:
        return None

    # outputs often list/tuple of arrays; sometimes nested.
    if isinstance(outputs, dict):
        # Best-effort key mapping
        boxes = outputs.get("boxes") or outputs.get("bboxes") or outputs.get("box")
        scores = outputs.get("scores") or outputs.get("score")
        classes = outputs.get("classes") or outputs.get("class_ids") or outputs.get("class")
        if boxes is None or scores is None or classes is None:
            return None
    else:
        if not isinstance(outputs, (list, tuple)) or len(outputs) < 3:
            return None
        boxes, scores, classes = outputs[0], outputs[1], outputs[2]

    boxes = np.asarray(boxes)
    scores = np.asarray(scores)
    classes = np.asarray(classes)

    # Peel leading singleton dimensions (batch-like)
    # e.g. (1,N,4) or (1,1,N,4) etc.
    while boxes.ndim > 2 and boxes.shape[0] == 1:
        boxes = boxes[0]
    while scores.ndim > 1 and scores.shape[0] == 1:
        scores = scores[0]
    while classes.ndim > 1 and classes.shape[0] == 1:
        classes = classes[0]

    # Handle single detection edge cases (box becomes (4,))
    if boxes.ndim == 1 and boxes.size == 4:
        boxes = boxes.reshape(1, 4)

    boxes = np.atleast_2d(boxes)
    scores = np.atleast_1d(scores)
    classes = np.atleast_1d(classes)

    # Final sanity: if boxes last dim not 4, bail (prevents crashing)
    if boxes.shape[-1] != 4:
        return None

    # Trim to shared length
    n = min(len(boxes), len(scores), len(classes))
    return boxes[:n], scores[:n], classes[:n]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default=DEFAULT_MODEL)
    ap.add_argument("--threshold", type=float, default=0.55, help="min confidence for detection")
    ap.add_argument("--print_hz", type=float, default=10.0, help="limit console prints")
    ap.add_argument("--csv", default="", help="optional CSV log path (e.g., logs/stage1.csv)")
    ap.add_argument("--preview", choices=["qtgl", "drm"], default="qtgl")
    ap.add_argument("--debug_shapes", action="store_true", help="print output tensor shapes (once)")
    args = ap.parse_args()

    model_file = args.model
    if not os.path.exists(model_file):
        raise FileNotFoundError(f"Model not found: {model_file}")

    # Must be created before Picamera2
    imx500 = IMX500(model_file)

    picam2 = Picamera2(imx500.camera_num)
    main_cfg = {"format": "RGB888"}
    config = picam2.create_preview_configuration(main_cfg, buffer_count=8)
    picam2.configure(config)

    labels = load_labels_fallback()
    person_label = "person"

    det_queue: "queue.SimpleQueue[Tuple[float, Detection]]" = queue.SimpleQueue()
    logger = CSVLogger(Path(args.csv) if args.csv else None)
    logger.open()

    stop = False
    printed_shapes = False

    def _sigint(_sig, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, _sigint)

    def parse_and_draw(request, stream="main"):
        nonlocal printed_shapes

        # Prevent preview thread abort if something unexpected happens
        try:
            metadata = request.get_metadata()
            outputs = _get_imx500_outputs(imx500, metadata, add_batch=True)

            if args.debug_shapes and not printed_shapes and outputs is not None:
                try:
                    shapes = []
                    for o in outputs if isinstance(outputs, (list, tuple)) else [outputs]:
                        shapes.append(getattr(o, "shape", None))
                    print(f"[debug] raw output shapes: {shapes}")
                except Exception:
                    pass
                printed_shapes = True

            norm = _normalize_ssd_outputs(outputs)
            if norm is None:
                return
            boxes, scores, classes = norm

            detections: List[Detection] = []
            for box, score, category in zip(boxes, scores, classes):
                conf = float(score)
                if conf < args.threshold:
                    continue

                cat = int(category)

                # Convert tensor coords to ISP output coords.
                # box should be (4,) in inference coord space for convert_inference_coords.
                obj_scaled = imx500.convert_inference_coords(box, metadata, picam2)
                x, y = int(obj_scaled.x), int(obj_scaled.y)
                w, h = int(obj_scaled.width), int(obj_scaled.height)

                # Reject weird zero/negative boxes
                if w <= 0 or h <= 0:
                    continue

                detections.append(Detection(category=cat, conf=conf, box=(x, y, w, h)))

            # Draw only PERSON in red + enqueue for main loop prints/log
            with MappedArray(request, stream) as m:
                for d in detections:
                    label = labels[d.category] if 0 <= d.category < len(labels) else str(d.category)

                    # Most COCO models: person == 0, but keep label check too.
                    if label != person_label and d.category != 0:
                        continue

                    x, y, w, h = d.box
                    cv2.rectangle(m.array, (x, y), (x + w, y + h), (0, 0, 255), 2)
                    cv2.putText(
                        m.array,
                        f"person {d.conf:.2f}",
                        (x + 5, max(15, y - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 0, 255),
                        1,
                    )
                    det_queue.put((time.time(), d))

        except Exception as e:
            # Keep running even if a frame causes trouble
            print(f"[parse_and_draw] {type(e).__name__}: {e}")
            return

    picam2.pre_callback = parse_and_draw

    if args.preview == "qtgl":
        picam2.start_preview(Preview.QTGL)
    else:
        picam2.start_preview(Preview.DRM)

    picam2.start()

    last_print = 0.0
    try:
        while not stop:
            now = time.time()

            try:
                ts, det = det_queue.get(timeout=0.05)
                if (now - last_print) >= (1.0 / max(0.1, args.print_hz)):
                    x, y, w, h = det.box
                    cx, cy = x + w / 2.0, y + h / 2.0
                    print(f"[person] conf={det.conf:.2f} box=(x={x}, y={y}, w={w}, h={h}) center=({cx:.1f},{cy:.1f})")
                    last_print = now

                logger.log_bbox(ts, det.conf, det.box)

                # --- Stage-1 hooks (do nothing yet) ---
                # dist_proxy = DistanceProxy(mode="height").proxy(det.box)
                # in_range = HysteresisFlag(trigger=..., clear=...).update(dist_proxy)
                # TODO(Stage1): GPIO/LED/buzzer outputs when in_range toggles

            except queue.Empty:
                pass

    finally:
        logger.close()
        picam2.stop()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
