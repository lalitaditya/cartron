#!/bin/bash
# run_aria_usb_stream.sh
# Complete wrapper that handles USB NCM interface setup and streaming.
#
# Usage: sudo bash run_aria_usb_stream.sh
# (Must run as root so we can configure network interfaces quickly)

set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

# Get the real user (when run via sudo)
REAL_USER="${SUDO_USER:-$(whoami)}"

if [ "$(id -u)" -ne 0 ]; then
    echo "ERROR: Must run as root. Use: sudo bash $0"
    exit 1
fi

echo "=== Aria USB Streaming Launcher ==="
echo "Running as root, real user: $REAL_USER"
echo ""

echo "[1/2] Starting NCM interface watcher in background..."
echo "  (Will auto-configure any new USB NCM interface)"

# Background watcher: tight loop monitoring for new USB network interfaces
(
    LAST_CONFIGURED=""
    while true; do
        # Look for any interface matching USB NCM naming patterns
        for IFACE in $(ip -brief link 2>/dev/null | grep -oP '^\S+' | grep -E '^(usb|enx)'); do
            STATE=$(cat /sys/class/net/$IFACE/operstate 2>/dev/null || echo "unknown")
            HAS_IP=$(ip -brief addr show "$IFACE" 2>/dev/null | grep -c '192.168.42' || true)
            
            if [ "$STATE" != "up" ] || [ "$HAS_IP" = "0" ]; then
                if [ "$IFACE" != "$LAST_CONFIGURED" ]; then
                    echo "  [NCM-watcher] Detected $IFACE (state=$STATE, has_ip=$HAS_IP), configuring..."
                    ip link set "$IFACE" up 2>/dev/null || true
                    sleep 0.3
                    # Remove any existing IP first, then add
                    ip addr flush dev "$IFACE" 2>/dev/null || true
                    ip addr add 192.168.42.100/24 dev "$IFACE" 2>/dev/null || true
                    sleep 0.3
                    NEW_STATE=$(cat /sys/class/net/$IFACE/operstate 2>/dev/null || echo "?")
                    NEW_IP=$(ip -brief addr show "$IFACE" 2>/dev/null)
                    echo "  [NCM-watcher] $IFACE -> state=$NEW_STATE, addr: $NEW_IP"
                    LAST_CONFIGURED="$IFACE"
                    # Quick ping test
                    if ping -c 1 -W 1 192.168.42.129 &>/dev/null; then
                        echo "  [NCM-watcher] 192.168.42.129 is REACHABLE!"
                    fi
                fi
            fi
        done
        sleep 0.2
    done
) &
NCM_WATCHER_PID=$!
echo "  Watcher PID: $NCM_WATCHER_PID"

# Cleanup on exit
cleanup() {
    echo ""
    echo "Cleaning up..."
    kill $NCM_WATCHER_PID 2>/dev/null || true
    wait $NCM_WATCHER_PID 2>/dev/null || true
    echo "Done."
}
trap cleanup EXIT INT TERM

echo ""
echo "[2/2] Starting Aria streaming as user $REAL_USER..."
echo "  (Make sure 'usbipd attach --auto-attach' is running on Windows!)"
echo ""

# Run the streaming script as the real user (not as root)
su - "$REAL_USER" -c "cd '$SCRIPT_DIR' && source aria_env/bin/activate && python aria_debug_stream.py"
