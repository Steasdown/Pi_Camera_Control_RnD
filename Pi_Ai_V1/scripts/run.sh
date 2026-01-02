#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR"

mkdir -p logs

echo "[run] repo: $REPO_DIR"
echo "[run] python: $(command -v python3)"
python3 --version

# Pass all args through to V1
python3 -u -m pi_ai_rnd.main "$@" 2>&1 | tee logs/run.log
