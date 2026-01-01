"""pi_ai_rnd.imx500_adapter — IMX500 parsing & coordinate conversion

Prototype A confirmed IMX500 SSD output tensor shapes:
- boxes:   (1, 100, 4)
- scores:  (1, 100)
- classes: (1, 100)
- count:   (1, 1)   (may represent number of valid detections)

This module converts the raw IMX500 outputs into a predictable Python structure.

Important:
- This module does NOT open the camera.
- It does NOT depend on Picamera2 objects.
- It is purely parsing utilities + small data structures.

Next step (Prototype B):
- main.py will open camera, fetch outputs, call normalize_ssd_outputs(), then
  filter to person and print bbox coords + confidence.

Note:
- Coordinate conversion (inference -> ISP output coords) is done by:
    imx500.convert_inference_coords(box, metadata, picam2)
  That call requires IMX500 instance + metadata + Picamera2. We keep that in main.py
  (or a future runtime module), not here.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, List, Optional, Sequence, Tuple

import numpy as np


@dataclass(frozen=True)
class DetectedObject:
    """A normalized detection record.

    Coordinates are *not* filled in by this module (yet), because coordinate conversion
    needs IMX500 + metadata + Picamera2. For Prototype B we only print raw boxes in
    inference coordinate space first, then we will switch to converted coords.

    Fields:
        class_id: COCO class id (often person==0 for SSD MobileNet)
        score: confidence score 0..1
        box: tuple of 4 floats as provided by model. Meaning depends on model (often
             [ymin, xmin, ymax, xmax] or similar). We will treat it as an opaque 4-vector
             until we confirm mapping during Prototype B/C.
    """
    class_id: int
    score: float
    box: Tuple[float, float, float, float]


def normalize_ssd_outputs(outputs: Any) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray, int]]:
    """Normalize SSD-style IMX500 outputs to (boxes, scores, classes, count).

    Expected (from Prototype A):
        outputs is list/tuple length >= 4:
            outputs[0] -> boxes   (1, N, 4)
            outputs[1] -> scores  (1, N)
            outputs[2] -> classes (1, N)
            outputs[3] -> count   (1, 1)  (or sometimes scalar/shape variants)

    Returns:
        boxes:   np.ndarray (N,4) float32/float64
        scores:  np.ndarray (N,)  float32/float64
        classes: np.ndarray (N,)  int
        count:   int number of valid detections (0..N)

    If outputs doesn't look like SSD outputs, return None.
    """
    if outputs is None:
        return None

    # Some versions might return dict-like outputs; support basic keys if present.
    if isinstance(outputs, dict):
        boxes = outputs.get("boxes") or outputs.get("bboxes") or outputs.get("box")
        scores = outputs.get("scores") or outputs.get("score")
        classes = outputs.get("classes") or outputs.get("class_ids") or outputs.get("class")
        count = outputs.get("count") or outputs.get("num") or outputs.get("num_dets")
        if boxes is None or scores is None or classes is None:
            return None
    else:
        if not isinstance(outputs, (list, tuple)) or len(outputs) < 3:
            return None
        boxes = outputs[0]
        scores = outputs[1]
        classes = outputs[2]
        count = outputs[3] if len(outputs) >= 4 else None

    boxes = np.asarray(boxes)
    scores = np.asarray(scores)
    classes = np.asarray(classes)

    # Peel leading singleton dims (batch)
    # boxes: (1,N,4) -> (N,4)
    while boxes.ndim > 2 and boxes.shape[0] == 1:
        boxes = boxes[0]
    # scores/classes: (1,N) -> (N,)
    while scores.ndim > 1 and scores.shape[0] == 1:
        scores = scores[0]
    while classes.ndim > 1 and classes.shape[0] == 1:
        classes = classes[0]

    # Sanity
    boxes = np.atleast_2d(boxes)
    scores = np.atleast_1d(scores)
    classes = np.atleast_1d(classes)

    if boxes.shape[-1] != 4:
        return None

    n = min(len(boxes), len(scores), len(classes))
    boxes = boxes[:n]
    scores = scores[:n]
    classes = classes[:n].astype(int, copy=False)

    # Count parsing:
    # Typically (1,1) ndarray; sometimes scalar. If missing, assume N.
    det_count = n
    if count is not None:
        c = np.asarray(count)
        try:
            det_count = int(c.reshape(-1)[0])
            det_count = max(0, min(det_count, n))
        except Exception:
            det_count = n

    return boxes, scores, classes, det_count


def build_detections(
    boxes: np.ndarray,
    scores: np.ndarray,
    classes: np.ndarray,
    count: int,
    threshold: float,
) -> List[DetectedObject]:
    """Build a list of DetectedObject filtered by score threshold.

    Uses only first 'count' entries (model may pad to N=100).
    """
    dets: List[DetectedObject] = []
    for i in range(int(count)):
        s = float(scores[i])
        if s < float(threshold):
            continue
        cid = int(classes[i])
        b = tuple(float(x) for x in boxes[i].tolist())  # 4-vector
        dets.append(DetectedObject(class_id=cid, score=s, box=b))  # raw inference coords for now
    return dets


def filter_by_class(dets: List[DetectedObject], class_id: int) -> List[DetectedObject]:
    """Return detections matching class_id."""
    return [d for d in dets if d.class_id == int(class_id)]
