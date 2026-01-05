"""
pi_ai_rnd.config — load runtime configuration.

Stage2 additions (Arduino integration Step 1):
- serial_enabled / serial_port / serial_baud
- serial_lines (ring buffer size)
- serial_reconnect_s (retry period)
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict


@dataclass
class AppConfig:
    # Model / inference
    model_path: str = "/usr/share/imx500-models/imx500_network_ssd_mobilenetv2_fpnlite_320x320_pp.rpk"
    score_threshold: float = 0.55
    print_hz: float = 5.0
    target_class_id: int = 0
    target_class_name: str = "person"

    # Camera / preview
    camera_num: int = 0
    view_w: int = 1280
    view_h: int = 720
    camera_format: str = "RGB888"
    buffer_count: int = 6

    # Stage2 Step1: Serial monitor (raw)
    serial_enabled: bool = False
    serial_port: str = "auto"       # "auto" or "/dev/ttyACM0" or "/dev/serial/by-id/..."
    serial_baud: int = 115200
    serial_lines: int = 30          # 20–50 recommended
    serial_reconnect_s: float = 1.0


def _as_dict(x: Any) -> Dict[str, Any]:
    if isinstance(x, dict):
        return x
    raise TypeError("config JSON must be an object")


def load_config(path: Path) -> AppConfig:
    cfg = AppConfig()
    if not path.exists():
        return cfg

    data = _as_dict(json.loads(path.read_text(encoding="utf-8")))
    for k, v in data.items():
        if not hasattr(cfg, k):
            continue
        setattr(cfg, k, v)

    # basic sanitation
    cfg.score_threshold = float(cfg.score_threshold)
    cfg.print_hz = float(cfg.print_hz)
    cfg.target_class_id = int(cfg.target_class_id)
    cfg.camera_num = int(cfg.camera_num)
    cfg.view_w = int(cfg.view_w)
    cfg.view_h = int(cfg.view_h)
    cfg.buffer_count = int(cfg.buffer_count)

    cfg.serial_enabled = bool(cfg.serial_enabled)
    cfg.serial_port = str(cfg.serial_port)
    cfg.serial_baud = int(cfg.serial_baud)
    cfg.serial_lines = int(cfg.serial_lines)
    cfg.serial_reconnect_s = float(cfg.serial_reconnect_s)

    return cfg
