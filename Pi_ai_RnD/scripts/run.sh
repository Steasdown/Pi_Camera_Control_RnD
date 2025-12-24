#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

if [[ -f ".venv/bin/activate" ]]; then
  source .venv/bin/activate
fi

python3 -u src/pi_ai_rnd/app_stage1.py
