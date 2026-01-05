from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np


@dataclass
class OverlayState:
    # last detection (latched)
    last_xywh: Optional[Tuple[int, int, int, int]] = None
    last_score: float = 0.0
    last_dx_px: float = 0.0


def _draw_filled_circle(img, center, r, bgr):
    cv2.circle(img, center, r, bgr, thickness=-1, lineType=cv2.LINE_AA)


def _draw_serial_box(img: np.ndarray, lines: List[str], max_lines: int = 3) -> None:
    h, w = img.shape[:2]
    lines = (lines or [])[-max_lines:]

    pad = 10
    line_h = 22
    box_h = pad * 2 + line_h * max(1, len(lines))
    y0 = h - box_h
    if y0 < 0:
        y0 = 0

    # solid black box
    cv2.rectangle(img, (0, y0), (w, h), (0, 0, 0), thickness=-1)

    y = y0 + pad + line_h - 6
    for s in lines:
        cv2.putText(img, s[:120], (pad, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
        y += line_h


def draw_overlay(
    frame_bgr: np.ndarray,
    *,
    pi_control: bool,
    bbox_xywh: Optional[Tuple[int, int, int, int]],
    score: float,
    dx_px: float,
    serial_lines: List[str],
) -> None:
    """
    Draws:
      - green circle when pi_control True
      - bbox + label (if bbox provided)
      - dx indicator (pixels from centre)
      - serial box at bottom with last 3 lines
    """
    h, w = frame_bgr.shape[:2]

    # PI_CONTROL indicator (top-left)
    if pi_control:
        _draw_filled_circle(frame_bgr, (24, 24), 10, (0, 255, 0))
        cv2.putText(frame_bgr, "PICTRL", (42, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv2.LINE_AA)
    else:
        cv2.circle(frame_bgr, (24, 24), 10, (120, 120, 120), thickness=2, lineType=cv2.LINE_AA)
        cv2.putText(frame_bgr, "PICTRL", (42, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (160, 160, 160), 1, cv2.LINE_AA)

    # centre line
    cx = w // 2
    cv2.line(frame_bgr, (cx, 0), (cx, h), (80, 80, 80), 1, cv2.LINE_AA)

    # bbox
    if bbox_xywh is not None:
        x, y, bw, bh = bbox_xywh
        cv2.rectangle(frame_bgr, (x, y), (x + bw, y + bh), (0, 0, 255), 2, cv2.LINE_AA)
        cv2.putText(
            frame_bgr,
            f"person {score:.2f}",
            (x + 6, max(24, y - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )

    # dx readout (top-centre)
    cv2.putText(
        frame_bgr,
        f"dx_px={dx_px:+.1f}",
        (cx - 80, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )

    _draw_serial_box(frame_bgr, serial_lines, max_lines=3)
