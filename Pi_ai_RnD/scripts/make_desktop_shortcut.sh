#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

REPO_ROOT="$(pwd)"
TEMPLATE="assets/Pi_AI_RnD.desktop.in"
OUT="${HOME}/Desktop/Pi_AI_RnD.desktop"

sed "s|__REPO_ROOT__|${REPO_ROOT}|g" "${TEMPLATE}" > "${OUT}"
chmod +x "${OUT}"
echo "Created: ${OUT}"
