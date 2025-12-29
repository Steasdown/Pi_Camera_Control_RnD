"""pi_ai_rnd.imx500_adapter — IMX500 parsing & coordinate conversion (scaffold)

Purpose:
- Isolate IMX500 output handling here so model/library differences don't cascade.

Planned:
- Read outputs from request metadata via IMX500 helper(s)
- Normalize outputs to a common form: (boxes, scores, class_ids)
- Convert inference coords -> ISP output coords

No hardware access in this commit.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple


@dataclass(frozen=True)
class DetectedObject:
    class_id: int
    score: float
    # Planned: ISP output coordinates
    x: int
    y: int
    w: int
    h: int


def normalize_ssd_outputs(outputs) -> Optional[Tuple[Sequence, Sequence, Sequence]]:
    """Normalize SSD-style outputs into (boxes, scores, classes).

    Implement during Prototype A/B once we inspect actual tensor shapes.
    """
    _ = outputs
    return None


def filter_by_class(dets: List[DetectedObject], class_id: int) -> List[DetectedObject]:
    return [d for d in dets if d.class_id == class_id]
