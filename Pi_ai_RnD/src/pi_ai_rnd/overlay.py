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
    anchor: str = "top_left",   # "top_left" | "bottom_left"
    margin: int = 10,
    max_lines: int = 30,
    line_height: int = 18,
) -> None:
    """
    Draw a simple text panel (solid background).

    anchor:
      - "top_left": panel starts near the top-left
      - "bottom_left": panel sits above the bottom edge (doesn't grow downward)
    """
    try:
        import cv2
    except Exception:
        return

    frame_h, frame_w = frame.shape[:2]

    # Collect lines (cap to max_lines)
    lines_list = list(lines)[-max_lines:]
    header = [title, status]
    all_lines = header + lines_list

    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.5
    thickness = 1

    max_w = 0
    for t in all_lines:
        (tw, _), _ = cv2.getTextSize(t, font, font_scale, thickness)
        max_w = max(max_w, tw)

    pad = 8
    panel_w = max_w + pad * 2
    panel_h = len(all_lines) * line_height + pad * 2

    if anchor == "bottom_left":
        x0 = margin
        y_top = max(margin, frame_h - margin - panel_h)
    else:
        x0 = margin
        y_top = margin

    # Background
    cv2.rectangle(frame, (x0, y_top), (x0 + panel_w, y_top + panel_h), (0, 0, 0), -1)

    # Text
    y = y_top + pad + line_height - 6
    for i, t in enumerate(all_lines):
        color = (255, 255, 255) if i < 2 else (200, 200, 200)
        cv2.putText(frame, t, (x0 + pad, y), font, font_scale, color, thickness, cv2.LINE_AA)
        y += line_height

