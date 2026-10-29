#!/bin/bash
# Script: set_mtu_9000.sh
# Description: Set MTU=9000 for all physical NICs persistently without changing IPs
# Requirements: Run as root

set -e

# Check for root privileges
if [ "$EUID" -ne 0 ]; then
    echo "ERROR: Please run as root or with sudo."
    exit 1
fi

# Helper script that will be called by udev
HELPER_SCRIPT="/usr/local/bin/set-mtu-phys.sh"

# Create helper script to set MTU on physical interfaces only
cat > "$HELPER_SCRIPT" << 'EOF'
#!/bin/bash
# Helper: set MTU 9000 for a given interface if it is physical
IFACE="$1"
if [ -z "$IFACE" ]; then
    exit 0
fi

# Exclude loopback and common virtual interface names
if [ "$IFACE" = "lo" ]; then
    exit 0
fi
case "$IFACE" in
    docker*|veth*|br-*|virbr*|vnet*|tap*|vl*|bond*|team*|tun*|wg*|zt*)
        exit 0 ;;
esac

# Check if it is a physical device (has a 'device' symlink)
if [ -L "/sys/class/net/$IFACE/device" ] || [ -d "/sys/class/net/$IFACE/device" ]; then
    ip link set dev "$IFACE" mtu 9000 2>/dev/null || echo "Warning: Failed to set MTU on $IFACE (maybe not supported)"
fi
EOF

chmod +x "$HELPER_SCRIPT"

# Set MTU on currently present physical interfaces
echo "Applying MTU 9000 to existing physical interfaces..."
for iface in /sys/class/net/*; do
    iface_name=$(basename "$iface")
    "$HELPER_SCRIPT" "$iface_name"
done

# Create udev rule for persistence (triggers on every net device addition)
UDEV_RULE="/etc/udev/rules.d/99-mtu9000.rules"
cat > "$UDEV_RULE" << EOF
# Set MTU 9000 for physical network interfaces as soon as they appear
SUBSYSTEM=="net", ACTION=="add", RUN+="$HELPER_SCRIPT %k"
EOF

# Reload udev rules and trigger for existing devices
udevadm control --reload-rules
udevadm trigger --action=add --subsystem-match=net

echo "Done. MTU 9000 will be automatically applied to all physical interfaces on boot or hotplug."
echo "Existing IP configurations are untouched."

ip link | grep mtu
