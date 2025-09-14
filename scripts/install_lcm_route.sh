#!/bin/bash
set -e

# Create systemd service file
sudo tee /etc/systemd/system/lcm-mcast-route.service >/dev/null <<'EOF'
[Unit]
Description=Add multicast route for LCM
After=network-online.target
Wants=network-online.target

[Service]
Type=oneshot
ExecStart=/usr/sbin/ip route replace 239.255.0.0/16 dev wlan0
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

# Enable service
sudo systemctl daemon-reload
sudo systemctl enable --now lcm-mcast-route.service