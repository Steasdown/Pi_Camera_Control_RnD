from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Iterable, Optional, Tuple


@dataclass(frozen=True)
class Detection:
    """
    Single SSD detection.

    box: raw inference 4-vector in model output space (as provided by IMX500 metadata)
    score: confidence
    class_id: integer class index
    """
    box: tuple[float, float, float, float]
    score: float
    class_id: int


def normalize_ssd_outputs(outputs: Any) -> Optional[Tuple[Any, Any, Any, int]]:
    """
    Normalize IMX500 SSD outputs into (boxes, scores, classes, count).

    Expected common shape (from your probe):
      boxes   (1,100,4)
      scores  (1,100)
      classes (1,100)
      count   (1,1)

    This function accepts outputs in:
    - list/tuple length 4: [boxes, scores, classes, count]
    - dict: tries common key names
    """
    if outputs is None:
        return None

    # dict-style
    if isinstance(outputs, dict):
        # Try common keys (varies by versions/models)
        boxes = outputs.get("boxes") or outputs.get("bbox") or outputs.get("box")
        scores = outputs.get("scores") or outputs.get("score")
        classes = outputs.get("classes") or outputs.get("class") or outputs.get("labels")
        count = outputs.get("count") or outputs.get("num") or outputs.get("n")
        if boxes is None or scores is None or classes is None or count is None:
            return None
        return boxes, scores, classes, int(_to_scalar(count))

    # list/tuple-style
    if isinstance(outputs, (list, tuple)) and len(outputs) >= 4:
        boxes, scores, classes, count = outputs[0], outputs[1], outputs[2], outputs[3]
        return boxes, scores, classes, int(_to_scalar(count))

    return None


def _to_scalar(x: Any) -> float:
    """
    Convert numpy-ish scalars/arrays into a python scalar.
    """
    try:
        # numpy scalar
        return float(x)
    except Exception:
        pass

    # numpy array with one element, or nested
    try:
        # e.g. shape (1,1)
        return float(x.reshape(-1)[0])
    except Exception:
        pass

    # list/tuple nested
    if isinstance(x, (list, tuple)) and x:
        return _to_scalar(x[0])

    raise TypeError(f"Cannot coerce to scalar: {type(x).__name__}")


def build_detections(
    boxes: Any,
    scores: Any,
    classes: Any,
    count: int,
    threshold: float,
) -> list[Detection]:
    """
    Build detections list applying the SINGLE threshold (caller passes cfg.score_threshold).

    boxes: (1,N,4) or (N,4)
    scores/classes: (1,N) or (N,)
    count: number of valid detections (may be <= N)
    """
    # Squeeze batch dimension if present
    try:
        if getattr(boxes, "ndim", 0) == 3:
            boxes_n = boxes[0]
        else:
            boxes_n = boxes
        if getattr(scores, "ndim", 0) == 2:
            scores_n = scores[0]
        else:
            scores_n = scores
        if getattr(classes, "ndim", 0) == 2:
            classes_n = classes[0]
        else:
            classes_n = classes
    except Exception:
        boxes_n, scores_n, classes_n = boxes, scores, classes

    dets: list[Detection] = []
    n = int(count) if int(count) > 0 else 0

    for i in range(n):
        try:
            s = float(scores_n[i])
        except Exception:
            continue
        if s < float(threshold):
            continue

        try:
            c = int(classes_n[i])
        except Exception:
            c = int(float(classes_n[i]))

        b = boxes_n[i]
        # Ensure plain python floats
        box4 = (float(b[0]), float(b[1]), float(b[2]), float(b[3]))
        dets.append(Detection(box=box4, score=s, class_id=c))

    return dets


def filter_by_class(dets: Iterable[Detection], class_id: int) -> list[Detection]:
    return [d for d in dets if d.class_id == int(class_id)]
