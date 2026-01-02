from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict


@dataclass(frozen=True)
class AppConfig:
    """
    Runtime configuration for V1.

    Notes:
    - `score_threshold` is applied once when building detections.
    - `view_size` is the *display stream* size (also used for pixel bbox conversion).
    - `ema_alpha` and `hold_s` control overlay stability:
        - ema_alpha: 0..1  (higher = more responsive, less smooth)
        - hold_s: seconds to keep last bbox drawn after last detection
    """
    model_path: str = "/usr/share/imx500-models/imx500_network_ssd_mobilenetv2_fpnlite_320x320_pp.rpk"
    score_threshold: float = 0.55
    print_hz: float = 5.0

    target_class_id: int = 0
    target_class_name: str = "person"

    view_size: str = "1270x720"

    ema_alpha: float = 0.35
    hold_s: float = 0.25


def _coerce_float(v: Any, default: float) -> float:
    try:
        return float(v)
    except Exception:
        return default


def _coerce_int(v: Any, default: int) -> int:
    try:
        return int(v)
    except Exception:
        return default


def load_config(path: Path) -> AppConfig:
    """
    Load config JSON. Unknown keys are ignored.
    Missing file -> defaults.
    """
    if not path.exists():
        return AppConfig()

    data: Dict[str, Any] = json.loads(path.read_text(encoding="utf-8"))

    return AppConfig(
        model_path=str(data.get("model_path", AppConfig.model_path)),
        score_threshold=_coerce_float(data.get("score_threshold", AppConfig.score_threshold), AppConfig.score_threshold),
        print_hz=_coerce_float(data.get("print_hz", AppConfig.print_hz), AppConfig.print_hz),
        target_class_id=_coerce_int(data.get("target_class_id", AppConfig.target_class_id), AppConfig.target_class_id),
        target_class_name=str(data.get("target_class_name", AppConfig.target_class_name)),
        view_size=str(data.get("view_size", AppConfig.view_size)),
        ema_alpha=_coerce_float(data.get("ema_alpha", AppConfig.ema_alpha), AppConfig.ema_alpha),
        hold_s=_coerce_float(data.get("hold_s", AppConfig.hold_s), AppConfig.hold_s),
    )
