#!/bin/bash
# Setup network for GigE Vision cameras (FLIR LWIR)
# Auto-detects interface or accepts it as argument

# Auto-detect interface: look for interface with 169.254.x.x (link-local, typical for FLIR)
if [ -n "$1" ]; then
    IFACE="$1"
else
    # Find interface with link-local IP (169.254.x.x) - typical for FLIR cameras
    IFACE=$(ip -4 addr show | grep -B2 "169\.254\." | grep -oP '^\d+: \K[^:@]+' | head -1)
    
    if [ -z "$IFACE" ]; then
        echo "[GigE Setup] WARNING: Could not auto-detect GigE interface (no 169.254.x.x found)"
        echo "[GigE Setup] Skipping network setup. Pass interface as argument if needed."
        exit 0
    fi
fi

echo "[GigE Setup] Configuring interface $IFACE for GigE Vision..."

# Set MTU to jumbo frames (9000 bytes)
sudo ip link set $IFACE mtu 9000 2>/dev/null && \
    echo "[GigE Setup] MTU set to 9000" || \
    echo "[GigE Setup] WARNING: Could not set MTU (need sudo?)"

# Increase socket receive buffer
sudo sysctl -w net.core.rmem_max=26214400 >/dev/null 2>&1 && \
    echo "[GigE Setup] rmem_max set to 26214400" || \
    echo "[GigE Setup] WARNING: Could not set rmem_max"

sudo sysctl -w net.core.rmem_default=26214400 >/dev/null 2>&1 && \
    echo "[GigE Setup] rmem_default set to 26214400"

# Show current config
echo "[GigE Setup] Current MTU: $(cat /sys/class/net/$IFACE/mtu 2>/dev/null || echo 'unknown')"
echo "[GigE Setup] Current rmem_max: $(sysctl -n net.core.rmem_max 2>/dev/null || echo 'unknown')"

echo "[GigE Setup] Done!"
