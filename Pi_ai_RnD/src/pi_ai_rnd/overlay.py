"""pi_ai_rnd.overlay — drawing and geometry helpers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Tuple


@dataclass(frozen=True)
class Rect:
    x: int
    y: int
    w: int
    h: int


def clamp_xywh(x: int, y: int, w: int, h: int, frame_w: int, frame_h: int) -> Tuple[int, int, int, int]:
    x = max(0, min(int(x), frame_w - 1))
    y = max(0, min(int(y), frame_h - 1))
    w = max(1, min(int(w), frame_w - x))
    h = max(1, min(int(h), frame_h - y))
    return x, y, w, h


def draw_text_panel(
    frame,
    title: str,
    status: str,
    lines: Iterable[str],
    *,
    origin: Tuple[int, int] = (10, 25),
    max_lines: int = 30,
    line_height: int = 18,
) -> None:
    """
    Draw a simple text panel (solid background) in the top-left.
    - frame: OpenCV image (BGR)
    """
    try:
        import cv2
    except Exception:
        return

    x0, y0 = origin

    # Collect lines (cap to max_lines)
    lines_list = list(lines)[-max_lines:]
    header = [title, status]
    all_lines = header + lines_list

    # Measure panel size
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.5
    thickness = 1

    max_w = 0
    for t in all_lines:
        (tw, th), _ = cv2.getTextSize(t, font, font_scale, thickness)
        max_w = max(max_w, tw)

    pad = 8
    panel_w = max_w + pad * 2
    panel_h = len(all_lines) * line_height + pad * 2

    # Background box
    cv2.rectangle(frame, (x0, y0 - 18), (x0 + panel_w, y0 - 18 + panel_h), (0, 0, 0), -1)

    # Header
    y = y0
    for i, t in enumerate(all_lines):
        color = (255, 255, 255) if i < 2 else (200, 200, 200)
        cv2.putText(frame, t, (x0 + pad, y), font, font_scale, color, thickness, cv2.LINE_AA)
        y += line_height
