#!/usr/bin/env bash
set -euo pipefail

# Non-destructive smoke test (no camera access):
# - prints environment info
# - lists key folders/files
# - checks common IMX500 model + rpicam JSON paths

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

echo "=== smoke_test ==="
echo "[env] user: $(whoami)"
echo "[env] host: $(hostname)"
echo "[env] pwd : $(pwd)"
echo "[env] date: $(date)"
echo

echo "[ls] repo root:"
ls -lah
echo

echo "[ls] src/pi_ai_rnd:"
ls -lah src/pi_ai_rnd || true
echo

MODEL="/usr/share/imx500-models/imx500_network_ssd_mobilenetv2_fpnlite_320x320_pp.rpk"
PP="/usr/share/rpi-camera-assets/imx500_mobilenet_ssd.json"

echo "[check] model: $MODEL"
[[ -f "$MODEL" ]] && echo "OK" || echo "WARN missing (install imx500-all)"
echo

echo "[check] rpicam postprocess: $PP"
[[ -f "$PP" ]] && echo "OK" || echo "WARN missing"
echo

echo "[check] repo reference json:"
[[ -f "configs/postprocess/reference_imx500_mobilenet_ssd.json" ]] && echo "OK" || echo "WARN missing"
echo "=== done ==="
