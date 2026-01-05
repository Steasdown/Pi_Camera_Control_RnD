from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict


@dataclass
class AppConfig:
    # IMX500 / detection
    model_path: str = "/usr/share/imx500-models/imx500_network_ssd_mobilenetv2_fpnlite_320x320_pp.rpk"
    score_threshold: float = 0.55
    target_class_id: int = 0
    target_class_name: str = "person"
    print_hz: float = 5.0

    # Serial / Arduino
    serial_port: str = "/dev/ttyACM0"
    serial_baud: int = 115200

    # Tracking -> PANSPD
    pan_deadband_px: int = 50
    pan_kp: float = 4.0            # steps/s per pixel of dx
    pan_max_speed: float = 900.0   # cap steps/s
    pan_min_speed: float = 120.0   # minimum magnitude when outside deadband (overcome stiction)
    pan_send_hz: float = 10.0      # command update rate
    pan_invert: bool = False       # invert sign if motor direction is reversed


def _get(d: Dict[str, Any], k: str, default: Any) -> Any:
    return d[k] if k in d else default


def load_config(path: Path) -> AppConfig:
    if not path.exists():
        return AppConfig()

    data = json.loads(path.read_text(encoding="utf-8"))
    cfg = AppConfig(
        model_path=str(_get(data, "model_path", AppConfig.model_path)),
        score_threshold=float(_get(data, "score_threshold", AppConfig.score_threshold)),
        target_class_id=int(_get(data, "target_class_id", AppConfig.target_class_id)),
        target_class_name=str(_get(data, "target_class_name", AppConfig.target_class_name)),
        print_hz=float(_get(data, "print_hz", AppConfig.print_hz)),

        serial_port=str(_get(data, "serial_port", AppConfig.serial_port)),
        serial_baud=int(_get(data, "serial_baud", AppConfig.serial_baud)),

        pan_deadband_px=int(_get(data, "pan_deadband_px", AppConfig.pan_deadband_px)),
        pan_kp=float(_get(data, "pan_kp", AppConfig.pan_kp)),
        pan_max_speed=float(_get(data, "pan_max_speed", AppConfig.pan_max_speed)),
        pan_min_speed=float(_get(data, "pan_min_speed", AppConfig.pan_min_speed)),
        pan_send_hz=float(_get(data, "pan_send_hz", AppConfig.pan_send_hz)),
        pan_invert=bool(_get(data, "pan_invert", AppConfig.pan_invert)),
    )
    return cfg
