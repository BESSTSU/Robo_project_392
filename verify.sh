#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT_DIR"

echo "== AmazingRobot verification =="

echo "[Check] Required files"
for f in app/main.py app/config.py app/orchestration/mission_orchestrator.py firmware/drive_esp32/main.cpp firmware/stepper_esp32/main.cpp; do
  [[ -f "$f" ]] || { echo "Missing: $f"; exit 1; }
  echo "  ok: $f"
done

echo "[Check] Python"
python3 --version

echo "[Check] Serial devices"
ls /dev/ttyUSB* 2>/dev/null || echo "No /dev/ttyUSB* found (connect ESP32 devices)"

echo "[Check] Cameras"
ls /dev/video* 2>/dev/null || echo "No /dev/video* found"

echo "Verification complete"
