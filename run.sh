#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT_DIR"

if [[ ! -d venv ]]; then
  echo "venv not found. Run: bash setup_env.sh"
  exit 1
fi

# shellcheck disable=SC1091
source venv/bin/activate

# Optional runtime overrides
if [[ -f .env ]]; then
  # Export all assignments loaded from .env so child processes (python) can read them.
  set -a
  # shellcheck disable=SC1091
  source .env
  set +a
fi

export PYTHONUNBUFFERED=1
python -m app.main
