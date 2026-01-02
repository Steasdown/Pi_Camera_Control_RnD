#!/usr/bin/env bash
set -euo pipefail

echo "[pi_setup] updating apt + installing runtime deps for V1"
sudo apt update

# Most IMX500 + Picamera2 images already have these; harmless if already installed.
sudo apt install -y \
  python3 \
  python3-numpy \
  python3-opencv \
  python3-picamera2

echo "[pi_setup] done"
echo "[pi_setup] reboot recommended if camera stack was updated"
