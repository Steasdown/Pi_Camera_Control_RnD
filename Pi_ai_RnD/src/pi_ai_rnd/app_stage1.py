#!/usr/bin/env python3
"""
Stage 1 (Path C): people detection + bbox logging scaffolding
Architecture: capture -> infer -> decide -> act -> log  (keep stable for later stages)

- Overlay: red bounding box for PERSON
- Output: prints bbox (x,y,w,h) + center + confidence
- Hooks included: CSV logging, distance proxy, hysteresis flag, future GPIO outputs
"""

from __future__ import annotations

import argparse
import csv
import os
import queue
import signal
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple, List

import cv2
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


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default=DEFAULT_MODEL)
    ap.add_argument("--threshold", type=float, default=0.55, help="min confidence for detection")
    ap.add_argument("--print_hz", type=float, default=10.0, help="limit console prints")
    ap.add_argument("--csv", default="", help="optional CSV log path (e.g., logs/stage1.csv)")
    ap.add_argument("--preview", choices=["qtgl", "drm"], default="qtgl")
    args = ap.parse_args()

    model_file = args.model
    if not os.path.exists(model_file):
        raise FileNotFoundError(f"Model not found: {model_file}")

    # Must be created before Picamera2 (per IMX500 docs) :contentReference[oaicite:4]{index=4}
    imx500 = IMX500(model_file)

    picam2 = Picamera2(imx500.camera_num)
    # Simple preview pipeline; IMX500 outputs are read from request metadata. :contentReference[oaicite:5]{index=5}
    main_cfg = {"format": "RGB888"}
    config = picam2.create_preview_configuration(main_cfg, buffer_count=8)
    picam2.configure(config)

    labels = load_labels_fallback()
    person_label = "person"

    det_queue: "queue.SimpleQueue[Tuple[float, Detection]]" = queue.SimpleQueue()
    logger = CSVLogger(Path(args.csv) if args.csv else None)
    logger.open()

    stop = False

    def _sigint(_sig, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, _sigint)

    def parse_and_draw(request, stream="main"):
        """Runs in camera thread: parse outputs + draw overlay onto ISP output. :contentReference[oaicite:6]{index=6}"""
        metadata = request.get_metadata()
        outputs = imx500.get_outputs(metadata)
        # SSD-style outputs: boxes, scores, classes (as in RPi docs snippet) :contentReference[oaicite:7]{index=7}
        boxes, scores, classes = outputs[0][0], outputs[1][0], outputs[2][0]

        detections: List[Detection] = []
        for box, score, category in zip(boxes, scores, classes):
            conf = float(score)
            if conf < args.threshold:
                continue
            cat = int(category)
            # Convert tensor coords to ISP output coords. :contentReference[oaicite:8]{index=8}
            obj_scaled = imx500.convert_inference_coords(box, metadata, picam2)
            x, y, w, h = int(obj_scaled.x), int(obj_scaled.y), int(obj_scaled.width), int(obj_scaled.height)
            detections.append(Detection(category=cat, conf=conf, box=(x, y, w, h)))

        # Draw only PERSON in red + enqueue for main loop prints/log
        with MappedArray(request, stream) as m:
            for d in detections:
                label = labels[d.category] if 0 <= d.category < len(labels) else str(d.category)
                if label != person_label and d.category != 0:
                    continue
                x, y, w, h = d.box
                cv2.rectangle(m.array, (x, y), (x + w, y + h), (0, 0, 255), 2)
                cv2.putText(m.array, f"person {d.conf:.2f}", (x + 5, max(15, y - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                det_queue.put((time.time(), d))

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
            # Drain queue; print/log at limited rate
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
