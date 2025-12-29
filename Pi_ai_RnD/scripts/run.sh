#!/usr/bin/env bash
set -euo pipefail

# Run wrapper:
# - activates venv if present
# - sets PYTHONPATH to repo/src
# - tees logs to logs/run.log

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

mkdir -p logs

if [[ -f ".venv/bin/activate" ]]; then
  # shellcheck disable=SC1091
  source .venv/bin/activate
fi

export PYTHONPATH="$REPO_ROOT/src"

echo "[run] repo: $REPO_ROOT"
echo "[run] python: $(which python3)"
python3 -V

python3 -u -m pi_ai_rnd.main "$@" 2>&1 | tee logs/run.log
