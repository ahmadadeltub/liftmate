#!/usr/bin/env bash
# LiftMate Control & Diagnostic Center - Raspberry Pi OS / macOS Installation Script
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$PROJECT_DIR"

echo "=========================================================="
echo " LiftMate Control & Diagnostic Center - Installer"
echo "=========================================================="

if [[ "$(uname -s)" == "Linux" ]]; then
  echo "[1/4] Installing Raspberry Pi OS / Debian system packages..."
  sudo apt-get update
  sudo apt-get install -y \
    python3-venv \
    python3-pip \
    python3-picamera2 \
    python3-libcamera \
    python3-opencv \
    rpicam-apps \
    v4l-utils \
    i2c-tools

  echo "[2/4] Configuring dialout & video group permissions for serial & camera..."
  sudo usermod -a -G dialout,video,i2c "$USER" || true
else
  echo "[1/4] Detected macOS / non-Linux host — skipping apt system packages."
fi

echo "[3/4] Creating Python virtual environment (.venv --system-site-packages for Picamera2)..."
python3 -m venv --system-site-packages .venv
source .venv/bin/activate

echo "[4/4] Installing Python requirements..."
pip install --upgrade pip
pip install -r requirements.txt

mkdir -p logs/captures

echo "=========================================================="
echo " Installation Complete!"
echo " Start the application with: ./run.sh"
echo " Or in simulation mode with: ./run.sh --sim"
echo "=========================================================="
