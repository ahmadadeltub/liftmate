#!/usr/bin/env bash
# LiftMate Standalone Raspberry Pi 3 Wi-Fi Access Point Setup (hostapd + dnsmasq)
# Automatically starts SSID "LiftMate" (password "12345678", IP 10.42.0.1) on every power-on.
set -euo pipefail

echo "=========================================================="
echo " LiftMate Standalone Wi-Fi AP Setup (Pi 3 - hostapd)"
echo " Target SSID: LiftMate  |  Password: 12345678"
echo " Dashboard URL: http://10.42.0.1 (or :8000)"
echo "=========================================================="

sudo apt-get update
sudo apt-get install -y hostapd dnsmasq
sudo systemctl unmask hostapd || true
sudo systemctl stop hostapd dnsmasq || true
sudo systemctl disable hostapd dnsmasq || true

sudo tee /etc/hostapd/liftmate.conf > /dev/null << "EOF"
interface=wlan0
driver=nl80211
ssid=LiftMate
hw_mode=g
channel=6
wmm_enabled=0
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0
wpa=2
wpa_passphrase=12345678
wpa_key_mgmt=WPA-PSK
wpa_pairwise=CCMP
rsn_pairwise=CCMP
EOF

sudo tee /etc/dnsmasq.d/liftmate.conf > /dev/null << "EOF"
interface=wlan0
bind-dynamic
domain-needed
bogus-priv
dhcp-range=10.42.0.10,10.42.0.200,255.255.255.0,12h
dhcp-option=3,10.42.0.1
dhcp-option=6,10.42.0.1
EOF

sudo tee /usr/local/bin/liftmate-wifi-fallback.sh > /dev/null << "EOF"
#!/bin/bash
for _ in $(seq 1 15); do
  [ -e /sys/class/net/wlan0 ] && break
  sleep 1
done
/usr/sbin/rfkill unblock wifi 2>/dev/null || true
nmcli connection delete LiftMate 2>/dev/null || true
nmcli device set wlan0 managed no 2>/dev/null || true
nmcli device disconnect wlan0 2>/dev/null || true
killall wpa_supplicant 2>/dev/null || true
killall hostapd 2>/dev/null || true
sleep 1
ip link set wlan0 down || true
ip addr flush dev wlan0 || true
ip link set wlan0 up
ip addr replace 10.42.0.1/24 dev wlan0
systemctl restart dnsmasq
exec /usr/sbin/hostapd /etc/hostapd/liftmate.conf
EOF
sudo chmod +x /usr/local/bin/liftmate-wifi-fallback.sh

sudo tee /etc/systemd/system/liftmate-wifi.service > /dev/null << "EOF"
[Unit]
Description=LiftMate Standalone Wi-Fi Hotspot (hostapd)
After=network.target NetworkManager.service
Wants=NetworkManager.service

[Service]
Type=simple
ExecStart=/usr/local/bin/liftmate-wifi-fallback.sh
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable --now liftmate-wifi.service
echo "[OK] LiftMate AP enabled automatically on boot at 10.42.0.1"
