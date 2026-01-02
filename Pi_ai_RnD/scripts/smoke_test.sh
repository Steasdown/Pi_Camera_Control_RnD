#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR"

echo "=== smoke_test (V1) ==="
echo "[env] user: $(whoami)"
echo "[env] host: $(hostname)"
echo "[env] pwd : $(pwd)"
echo "[env] date: $(date)"
echo

echo "[ls] repo root:"
ls -al
echo

echo "[check] config:"
test -f "configs/runtime.json" && echo "OK configs/runtime.json" || (echo "MISSING configs/runtime.json" && exit 2)

echo "[check] python compile:"
python3 -m py_compile src/pi_ai_rnd/main.py
python3 -m py_compile src/pi_ai_rnd/imx500_adapter.py
python3 -m py_compile src/pi_ai_rnd/config.py
echo "OK py_compile"
echo

echo "[note] V1 window requires DISPLAY (desktop/VNC)."
echo "=== done ==="
