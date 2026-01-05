from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import List, Optional

try:
    import serial  # pyserial
except Exception:  # pragma: no cover
    serial = None


@dataclass
class SerialStatus:
    connected: bool
    port: str
    baud: int
    last_line: str = ""
    state: str = ""          # e.g. "PICTRL", "MANUAL"
    state_ts: float = 0.0    # monotonic timestamp of last state update


class SerialReader:
    """
    Background serial reader.
    - Keeps a rolling buffer of the last N lines for GUI
    - Tracks Arduino state by parsing:
        - Telemetry:  T,<STATE>,...
        - State print: [STATE] -> <STATE>
    - Provides write_line() for sending commands to Arduino
    """

    def __init__(self, port: str, baud: int = 115200, keep_lines: int = 50, connect_timeout_s: float = 2.0):
        self.port = port
        self.baud = int(baud)
        self.keep_lines = int(max(5, keep_lines))
        self.connect_timeout_s = float(connect_timeout_s)

        self._lock = threading.Lock()
        self._lines: List[str] = []
        self._status = SerialStatus(connected=False, port=self.port, baud=self.baud)

        self._ser = None
        self._stop = False
        self._thr: Optional[threading.Thread] = None

    def start(self) -> None:
        self._stop = False
        self._thr = threading.Thread(target=self._run, name="SerialReader", daemon=True)
        self._thr.start()

    def stop(self) -> None:
        self._stop = True
        if self._thr:
            self._thr.join(timeout=1.0)
        self._close()

    # ---------- public API ----------
    def get_last_lines(self, n: int = 3) -> List[str]:
        n = int(max(1, n))
        with self._lock:
            return self._lines[-n:].copy()

    def status(self) -> SerialStatus:
        with self._lock:
            return SerialStatus(**self._status.__dict__)

    def is_pi_control(self) -> bool:
        st = self.status().state.upper()
        return st in ("PICTRL", "PI_CONTROL", "AUTO_PI")

    def write_line(self, s: str) -> bool:
        """
        Send a single line (adds newline if missing).
        Returns True if written, False if not connected.
        """
        if not s:
            return False
        if not s.endswith("\n"):
            s = s + "\n"

        with self._lock:
            ser = self._ser
        if ser is None:
            return False

        try:
            ser.write(s.encode("utf-8"))
            return True
        except Exception:
            self._close()
            return False

    # ---------- internals ----------
    def _append_line(self, line: str) -> None:
        line = line.strip()
        if not line:
            return

        now_t = time.monotonic()
        new_state = self._parse_state(line)

        with self._lock:
            self._lines.append(line)
            if len(self._lines) > self.keep_lines:
                self._lines = self._lines[-self.keep_lines :]

            self._status.last_line = line

            if new_state:
                self._status.state = new_state
                self._status.state_ts = now_t

    @staticmethod
    def _parse_state(line: str) -> str:
        # Telemetry: "T,PICTRL,3,1,1,1234,12.3"
        if line.startswith("T,"):
            parts = line.split(",")
            if len(parts) >= 2:
                return parts[1].strip()

        # State print: "[STATE] -> PICTRL"
        if "[STATE]" in line and "->" in line:
            # take the part after "->"
            try:
                rhs = line.split("->", 1)[1].strip()
                # first token only
                return rhs.split()[0].strip()
            except Exception:
                return ""

        return ""

    def _connect(self) -> None:
        if serial is None:
            with self._lock:
                self._status.connected = False
            return

        try:
            ser = serial.Serial(self.port, self.baud, timeout=0.1, write_timeout=0.2)
            # settle
            time.sleep(0.15)
            with self._lock:
                self._ser = ser
                self._status.connected = True
        except Exception:
            self._close()

    def _close(self) -> None:
        with self._lock:
            ser = self._ser
            self._ser = None
            self._status.connected = False
        try:
            if ser is not None:
                ser.close()
        except Exception:
            pass

    def _run(self) -> None:
        # (re)connect loop
        last_try = 0.0

        while not self._stop:
            with self._lock:
                ser = self._ser

            if ser is None:
                now = time.monotonic()
                if (now - last_try) > self.connect_timeout_s:
                    last_try = now
                    self._connect()
                time.sleep(0.05)
                continue

            try:
                raw = ser.readline()
                if not raw:
                    time.sleep(0.005)
                    continue
                line = raw.decode("utf-8", errors="replace").strip()
                self._append_line(line)
            except Exception:
                self._close()
                time.sleep(0.1)
