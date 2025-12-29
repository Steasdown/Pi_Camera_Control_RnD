"""pi_ai_rnd.config — central configuration (scaffold)

Rules:
- Avoid absolute paths in Python code where possible.
- Scripts are responsible for repo-root resolution and venv activation.
- Keep thresholds, class filters, and logging flags here.

This loader is deliberately forgiving for scaffolding:
- missing config file -> defaults
- invalid JSON -> defaults
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import json


@dataclass(frozen=True)
class AppConfig:
    # Default IMX500 model path on Pi (installed by imx500-all)
    model_path: str = "/usr/share/imx500-models/imx500_network_ssd_mobilenetv2_fpnlite_320x320_pp.rpk"

    # COCO: person is often class id 0 in SSD MobileNet pipelines (verify later).
    target_class_name: str = "person"
    target_class_id: int = 0

    # Detection threshold
    score_threshold: float = 0.55

    # Output controls (planned)
    print_hz: float = 10.0
    enable_csv: bool = False
    csv_path: str = "logs/detections.csv"


def load_config(config_path: Path) -> AppConfig:
    cfg = AppConfig()
    if not config_path.exists():
        return cfg

    try:
        data = json.loads(config_path.read_text(encoding="utf-8"))
    except Exception:
        return cfg

    return AppConfig(
        model_path=data.get("model_path", cfg.model_path),
        target_class_name=data.get("target_class_name", cfg.target_class_name),
        target_class_id=int(data.get("target_class_id", cfg.target_class_id)),
        score_threshold=float(data.get("score_threshold", cfg.score_threshold)),
        print_hz=float(data.get("print_hz", cfg.print_hz)),
        enable_csv=bool(data.get("enable_csv", cfg.enable_csv)),
        csv_path=data.get("csv_path", cfg.csv_path),
    )
