"""pi_ai_rnd.logging_utils — stdout + CSV helpers (scaffold)

Targets:
- SSH-friendly stdout logging
- Optional CSV logging (timestamp, score, bbox)
"""

from __future__ import annotations

import csv
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple


@dataclass
class CsvLogger:
    path: Path
    enabled: bool = False
    _fh: Optional[object] = None
    _writer: Optional[csv.writer] = None

    def open(self) -> None:
        if not self.enabled:
            return
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._fh = open(self.path, "a", newline="")
        self._writer = csv.writer(self._fh)
        if self._fh.tell() == 0:
            self._writer.writerow(["ts_unix", "score", "x", "y", "w", "h"])

    def log(self, score: float, box: Tuple[int, int, int, int]) -> None:
        if not self._writer:
            return
        x, y, w, h = box
        self._writer.writerow([f"{time.time():.6f}", f"{score:.4f}", x, y, w, h])
        self._fh.flush()

    def close(self) -> None:
        if self._fh:
            self._fh.close()
        self._fh = None
        self._writer = None


def rate_limited_print(msg: str, hz: float, state: dict) -> None:
    now = time.time()
    min_dt = 1.0 / max(0.1, hz)
    last = state.get("last_t", 0.0)
    if (now - last) >= min_dt:
        print(msg)
        state["last_t"] = now
