"""
pi_ai_rnd.serial_reader — non-blocking Arduino serial line capture.

Stage2 / Step 1: display *raw* Arduino serial output on the Pi GUI.

Key behaviours:
- Non-blocking poll() that can be called from the camera loop.
- Automatic reconnect when the port disappears (Arduino reset / unplug).
- Ring buffer for the last N lines.

This module purposely does *not* parse or interpret the Arduino protocol.
"""

from __future__ import annotations

import glob
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, List, Optional, Tuple


def _auto_ports_linux() -> List[str]:
    """
    Prefer stable-by-id paths first, then fall back to common tty names.
    """
    ports: List[str] = []
    ports.extend(sorted(glob.glob("/dev/serial/by-id/*")))
    ports.extend([p for p in ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0", "/dev/ttyUSB1"] if p])
    # Dedup, keep order
    seen = set()
    out = []
    for p in ports:
        if p not in seen:
            seen.add(p)
            out.append(p)
    return out


@dataclass
class SerialState:
    connected: bool = False
    port: str = ""
    baud: int = 115200
    last_line_age_s: Optional[float] = None
    last_error: Optional[str] = None


class SerialPoller:
    """
    Non-blocking serial line reader with reconnect.

    Usage:
        poller = SerialPoller(enabled=True, port="auto", baud=115200, max_lines=30)
        status, lines = poller.poll()   # call every frame
        poller.close()
    """

    def __init__(
        self,
        enabled: bool,
        port: str,
        baud: int = 115200,
        max_lines: int = 30,
        reconnect_s: float = 1.0,
    ) -> None:
        self.enabled = bool(enabled)
        self.port = port or "auto"
        self.baud = int(baud)
        self.max_lines = max(1, int(max_lines))
        self.reconnect_s = max(0.2, float(reconnect_s))

        self._buf: Deque[str] = deque(maxlen=self.max_lines)
        self._ser = None  # pyserial Serial instance
        self._last_attempt_t = 0.0
        self._last_line_t: Optional[float] = None
        self._state = SerialState(connected=False, port=self.port, baud=self.baud)

        # Defer importing pyserial until actually needed
        self._serial_mod = None
        if self.enabled:
            try:
                import serial  # type: ignore
                self._serial_mod = serial
            except Exception as e:
                self.enabled = False
                self._state.last_error = f"pyserial not available ({type(e).__name__})"

    def _pick_port(self) -> Optional[str]:
        if self.port and self.port.lower() != "auto":
            return self.port

        # Linux auto-scan (Pi)
        for p in _auto_ports_linux():
            return p
        return None

    def _try_open(self) -> None:
        if not self.enabled or self._serial_mod is None:
            return

        now_t = time.monotonic()
        if (now_t - self._last_attempt_t) < self.reconnect_s:
            return
        self._last_attempt_t = now_t

        port = self._pick_port()
        self._state.port = port or "auto"
        if not port:
            self._state.connected = False
            self._state.last_error = "no serial port found"
            return

        try:
            # timeout=0 makes reads non-blocking
            self._ser = self._serial_mod.Serial(port=port, baudrate=self.baud, timeout=0)
            self._state.connected = True
            self._state.last_error = None
        except Exception as e:
            self._ser = None
            self._state.connected = False
            self._state.last_error = f"open failed: {type(e).__name__}: {e}"

    def close(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None
        self._state.connected = False

    def poll(self) -> Tuple[str, List[str]]:
        """
        Returns (status_string, lines_list).
        - status_string includes connection + age of last line.
        - lines_list is newest-last, capped to max_lines.
        """
        if not self.enabled:
            return ("DISABLED", list(self._buf))

        if self._ser is None or not getattr(self._ser, "is_open", False):
            self._try_open()
            if self._ser is None:
                age = None
                self._state.last_line_age_s = age
                err = self._state.last_error or "disconnected"
                return (f"DISCONNECTED ({err})", list(self._buf))

        # Read all available lines without blocking
        try:
            while True:
                raw = self._ser.readline()
                if not raw:
                    break
                line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
                if line:
                    self._buf.append(line)
                    self._last_line_t = time.monotonic()
        except Exception as e:
            # Port likely went away; close and report, will reconnect later
            self._state.last_error = f"read failed: {type(e).__name__}: {e}"
            self.close()
            return (f"DISCONNECTED ({self._state.last_error})", list(self._buf))

        # Build status text
        age_s: Optional[float] = None
        if self._last_line_t is not None:
            age_s = max(0.0, time.monotonic() - self._last_line_t)
        self._state.last_line_age_s = age_s

        port_name = getattr(self._ser, "port", self.port)
        if age_s is None:
            return (f"CONNECTED {port_name} @ {self.baud} (no lines yet)", list(self._buf))
        return (f"CONNECTED {port_name} @ {self.baud} (last {age_s:.1f}s)", list(self._buf))
