#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

# System deps (idempotent)
sudo apt update && sudo apt full-upgrade -y
sudo apt install -y imx500-all python3-opencv python3-munkres

# Project venv uses system-site packages so it can see apt-installed picamera2/opencv
python3 -m venv .venv --system-site-packages
source .venv/bin/activate
python -V
which python
