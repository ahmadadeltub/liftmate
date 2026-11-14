#!/usr/bin/env bash
# LiftMate Clean Power-Off Script (Raspberry Pi 3 OS + USB Bus Power Cut if supported)
set -u

CUT_USB="${1:-1}"

# Allow FastAPI a brief window to deliver the JSON confirmation over Wi-Fi AP
sleep 1.2

systemctl stop liftmate-camera.service 2>/dev/null || true
systemctl stop liftmate-port80.service 2>/dev/null || true

if [ "$CUT_USB" = "1" ] && command -v uhubctl >/dev/null 2>&1; then
  # Cut 5V VBUS on Raspberry Pi 3B/3B+ USB hub ports (powers down USB Arduino Mega & camera)
  uhubctl -l 1-1 -a 0 >/dev/null 2>&1 || true
  uhubctl -a 0 >/dev/null 2>&1 || true
fi

sync
if command -v systemctl >/dev/null 2>&1; then
  exec systemctl poweroff
fi
exec shutdown -h now
