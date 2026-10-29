#!/bin/bash

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

if [ "$(id -u)" -ne 0 ]; then
    echo -e "${RED}Error: Please run this script with root or sudo${NC}"
    exit 1
fi

if ! command -v cpufreq-set &> /dev/null; then
    echo -e "${YELLOW}cpufrequtils not found, installing...${NC}"
    sudo dpkg -i libcpufreq0_008-1build1_amd64.deb &> /dev/null
    sudo dpkg -i cpufrequtils_008-1build1_amd64.deb &> /dev/null
    if [ $? -ne 0 ]; then
        echo -e "${RED}Installation failed, please check your network source${NC}"
        exit 1
    fi
fi

SERVICE_FILE="/etc/systemd/system/cpu-performance.service"

cat > "$SERVICE_FILE" << EOF
[Unit]
Description=Set CPU Governor to Performance Mode
After=multi-user.target
Wants=network.target

[Service]
Type=oneshot
ExecStart=/bin/bash -c 'for cpu in /sys/devices/system/cpu/cpu[0-9]*/cpufreq/scaling_governor; do echo performance > \$cpu; done'
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable --now cpu-performance.service

if systemctl is-active --quiet cpu-performance.service; then
    echo -e "${GREEN}Success: CPU performance mode service installed and started${NC}"
else
    echo -e "${RED}Failed: Service start failed, check journalctl -u cpu-performance.service${NC}"
    exit 1
fi

echo -e "\nCurrent CPU scaling governor:"
cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor | uniq
echo ""

echo -e "${GREEN}Done: Performance mode will auto apply after reboot${NC}"



