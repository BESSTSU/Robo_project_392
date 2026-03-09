#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT_DIR"

PYTHON_BIN="${PYTHON_BIN:-python3}"
VENV_DIR="${VENV_DIR:-venv}"
INSTALL_PROFILE="${INSTALL_PROFILE:-cpu}"  # cpu | full

echo "[1/4] Creating virtual environment: $VENV_DIR"
$PYTHON_BIN -m venv "$VENV_DIR"

# shellcheck disable=SC1091
source "$VENV_DIR/bin/activate"

echo "[2/4] Upgrading pip"
pip install --upgrade pip
pip install --upgrade setuptools wheel

echo "[3/4] Installing Python dependencies (profile: $INSTALL_PROFILE)"

# Install non-ML core dependencies first
pip install flask numpy opencv-python pyserial pupil-apriltags

if [[ "$INSTALL_PROFILE" == "cpu" ]]; then
  echo "      Installing CPU PyTorch (to avoid large CUDA packages)..."
  if ! pip install --index-url https://download.pytorch.org/whl/cpu torch torchvision; then
    echo "      CPU index unavailable, fallback to default PyPI torch packages..."
    pip install torch torchvision
  fi
else
  echo "      Installing default PyTorch packages..."
  pip install torch torchvision
fi

# Install ultralytics after torch is available
pip install ultralytics

echo "[4/4] Setup complete"
echo "Run with: bash run.sh"
echo "Tip: INSTALL_PROFILE=full bash setup_env.sh"
