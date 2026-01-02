"""pi_ai_rnd.logging_utils — stdout + CSV helpers

Targets:
- SSH-friendly stdout logging (rate-limited)
- Optional CSV logging (timestamp, score, bbox)
- Compatibility helpers expected by main.py: now(), hz_to_dt()
"""

from __future__ import annotations

import csv
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple


# ----------------------------
# Time / rate helpers
# ----------------------------

def now() -> float:
    """Monotonic timestamp in seconds (use for measuring intervals)."""
    return time.monotonic()


def hz_to_dt(hz: float, min_hz: float = 0.1) -> float:
    """Convert a frequency (Hz) into a minimum interval (seconds)."""
    try:
        hz_f = float(hz)
    except Exception:
        hz_f = float(min_hz)
    hz_f = max(float(min_hz), hz_f)
    return 1.0 / hz_f


def rate_limited_print(msg: str, hz: float, state: dict) -> None:
    """Print at most `hz` times per second.

    `state` is a mutable dict that stores timing between calls, e.g.:
        state = {}
        rate_limited_print("hello", 5.0, state)
    """
    t = now()
    min_dt = hz_to_dt(hz)
    last = float(state.get("last_t", -1e9))
    if (t - last) >= min_dt:
        print(msg, flush=True)
        state["last_t"] = t


# ----------------------------
# CSV logger
# ----------------------------

@dataclass
class CsvLogger:
    """Append-only CSV logger for detections.

    Columns:
        ts_unix, score, x, y, w, h

    Notes:
    - Uses UNIX time for logs (time.time), because it's convenient for later correlation.
    - Uses monotonic time for runtime rate limiting elsewhere (now()).
    """
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
            self._fh.flush()

    def log(self, score: float, box: Tuple[int, int, int, int]) -> None:
        if not self._writer or not self._fh:
            return
        x, y, w, h = box
        self._writer.writerow([f"{time.time():.6f}", f"{float(score):.4f}", int(x), int(y), int(w), int(h)])
        self._fh.flush()

    def close(self) -> None:
        if self._fh:
            try:
                self._fh.close()
            finally:
                self._fh = None
                self._writer = None
