#!/usr/bin/env bash
# LiftMate Control & Diagnostic Center - Startup Script
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$PROJECT_DIR"

HOST="127.0.0.1"
PORT="8000"
SIM_MODE="false"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --lan)
      HOST="0.0.0.0"
      shift
      ;;
    --port)
      PORT="$2"
      shift 2
      ;;
    --sim|--simulation)
      SIM_MODE="true"
      shift
      ;;
    *)
      shift
      ;;
  esac
done

if [[ -f ".venv/bin/activate" ]]; then
  source .venv/bin/activate
fi

if [[ "$SIM_MODE" == "true" ]]; then
  python3 -c "
from backend.app.config import ConfigManager
cfg = ConfigManager()
cfg.update_section('system', {'simulation_mode': True})
"
  echo "[LiftMate] Simulation mode enabled."
fi

export PYTHONPATH="$PROJECT_DIR"
echo "=========================================================="
echo " LiftMate Control & Diagnostic Center"
echo " URL: http://${HOST}:${PORT}"
echo "=========================================================="
exec python3 -m uvicorn backend.app.main:app --host "$HOST" --port "$PORT"
