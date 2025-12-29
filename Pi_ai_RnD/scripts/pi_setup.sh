#!/usr/bin/env bash
set -euo pipefail

# One-time setup for Raspberry Pi:
# - installs IMX500 + OpenCV deps
# - creates a venv that can see system packages (picamera2 is usually apt-installed)

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

echo "[pi_setup] repo: $REPO_ROOT"

sudo apt update
sudo apt full-upgrade -y
sudo apt install -y imx500-all python3-opencv python3-munkres git

python3 -m venv .venv --system-site-packages
# shellcheck disable=SC1091
source .venv/bin/activate

echo "[pi_setup] python: $(which python)"
python -V
pip -V
echo "[pi_setup] done"
