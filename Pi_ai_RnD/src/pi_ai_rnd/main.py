"""
pi_ai_rnd.imx500_adapter

Purpose
-------
Pure parsing utilities for IMX500 SSD-style outputs.

This module:
- DOES NOT open the camera.
- DOES NOT depend on Picamera2 objects.
- DOES NOT do coordinate conversion (that requires IMX500 + metadata + Picamera2).

It only takes raw IMX500 network outputs and normalizes them into predictable
numpy arrays and lightweight Python records.

Observed on your Pi (Prototype A):
- boxes:   (1, 100, 4)
- scores:  (1, 100)
- classes: (1, 100)
- count:   (1, 1)  (may be valid-detection count, or may be unused/padded)

Notes
-----
- The 4-vector box format is model-dependent. For many SSD MobileNet pipelines,
  it is often [ymin, xmin, ymax, xmax] in normalized inference coordinates.
  Until we explicitly confirm, this module treats the box as an opaque 4-float tuple.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, List, Optional, Sequence, Tuple

import numpy as np


@dataclass(frozen=True)
class DetectedObject:
    """Normalized detection record (model/inference coordinate space).

    Fields:
        class_id: integer class id (often person==0 for COCO SSD MobileNet)
        score: confidence score (0..1)
        box: 4-vector from the model output (opaque until format is confirmed)
    """
    class_id: int
    score: float
    box: Tuple[float, float, float, float]


def normalize_ssd_outputs(outputs: Any) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray, int]]:
    """Normalize raw IMX500 SSD-like outputs to (boxes, scores, classes, count).

    Accepts either:
      A) list/tuple: [boxes, scores, classes, count?]
      B) dict-like (best-effort): keys like boxes/scores/classes/count

    Returns:
        boxes   : (N,4) float ndarray
        scores  : (N,)  float ndarray
        classes : (N,)  int ndarray
        count   : int   number of valid detections in [0..N]

    If the structure does not look like SSD outputs, returns None.
    """
    if outputs is None:
        return None

    # --- Extract raw tensors ---
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

    # --- Convert to numpy ---
    boxes = np.asarray(boxes)
    scores = np.asarray(scores)
    classes = np.asarray(classes)

    # --- Remove leading batch dims, if present ---
    # boxes:   (1,N,4) -> (N,4)
    # scores:  (1,N)   -> (N,)
    # classes: (1,N)   -> (N,)
    while boxes.ndim > 2 and boxes.shape[0] == 1:
        boxes = boxes[0]
    while scores.ndim > 1 and scores.shape[0] == 1:
        scores = scores[0]
    while classes.ndim > 1 and classes.shape[0] == 1:
        classes = classes[0]

    boxes = np.atleast_2d(boxes)
    scores = np.atleast_1d(scores)
    classes = np.atleast_1d(classes)

    if boxes.shape[-1] != 4:
        return None

    # --- Align lengths safely ---
    n = min(len(boxes), len(scores), len(classes))
    boxes = boxes[:n]
    scores = scores[:n]
    classes = classes[:n].astype(int, copy=False)

    # --- Parse count (valid detections) ---
    # Some pipelines provide count as (1,1) tensor; if absent, assume all N.
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
    """Build DetectedObject list from normalized arrays.

    - Only examines the first `count` entries (many models pad to N=100).
    - Applies score threshold.
    """
    dets: List[DetectedObject] = []
    max_i = int(count)

    for i in range(max_i):
        s = float(scores[i])
        if s < float(threshold):
            continue
        cid = int(classes[i])
        b = tuple(float(x) for x in boxes[i].tolist())  # opaque 4-vector
        dets.append(DetectedObject(class_id=cid, score=s, box=b))

    return dets


def filter_by_class(dets: List[DetectedObject], class_id: int) -> List[DetectedObject]:
    """Filter detections by class id."""
    target = int(class_id)
    return [d for d in dets if d.class_id == target]
