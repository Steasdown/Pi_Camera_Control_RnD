"""pi_ai_rnd.overlay — drawing helpers (scaffold)

Goal:
- Keep drawing code isolated so preview/GUI choices can change later.

Prototype C will implement:
- red bbox for person
- label + confidence
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Box:
    x: int
    y: int
    w: int
    h: int


def draw_person_box(frame, box: Box, score: float) -> None:
    """Draw a red bbox + label onto a frame buffer.

    Parameters:
        frame: OpenCV image buffer (numpy array)
        box: output coordinates (x,y,w,h)
        score: confidence 0..1

    TODO(Prototype C):
        - import cv2
        - cv2.rectangle(...)
        - cv2.putText(...)
    """
    _ = frame, box, score
