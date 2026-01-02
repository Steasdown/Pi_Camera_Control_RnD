from __future__ import annotations

from typing import Tuple


def clamp_xywh(x: int, y: int, w: int, h: int, frame_w: int, frame_h: int) -> Tuple[int, int, int, int]:
    x = max(0, min(int(x), frame_w - 1))
    y = max(0, min(int(y), frame_h - 1))
    w = max(1, min(int(w), frame_w - x))
    h = max(1, min(int(h), frame_h - y))
    return x, y, w, h
