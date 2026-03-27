#!/bin/bash
# run_aria_hand_tracker.sh
# Launches the AriaHandTracker with USB NCM auto-configuration.
#
# Usage: sudo bash run_aria_hand_tracker.sh [--viz]
# (Must run as root for NCM interface configuration)

set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

REAL_USER="${SUDO_USER:-$(whoami)}"
EXTRA_ARGS="$@"

if [ "$(id -u)" -ne 0 ]; then
    echo "ERROR: Must run as root. Use: sudo bash $0 $@"
    exit 1
fi

echo "=== Aria Hand Tracker Launcher ==="
echo "Running as root, real user: $REAL_USER"
echo ""

echo "[1/2] Starting NCM interface watcher in background..."

# Background watcher: tight loop monitoring for new USB network interfaces
(
    LAST_CONFIGURED=""
    while true; do
        for IFACE in $(ip -brief link 2>/dev/null | grep -oP '^\S+' | grep -E '^(usb|enx)'); do
            STATE=$(cat /sys/class/net/$IFACE/operstate 2>/dev/null || echo "unknown")
            HAS_IP=$(ip -brief addr show "$IFACE" 2>/dev/null | grep -c '192.168.42' || true)
            
            if [ "$STATE" != "up" ] || [ "$HAS_IP" = "0" ]; then
                if [ "$IFACE" != "$LAST_CONFIGURED" ]; then
                    echo "  [NCM-watcher] Detected $IFACE (state=$STATE), configuring..."
                    ip link set "$IFACE" up 2>/dev/null || true
                    sleep 0.3
                    ip addr flush dev "$IFACE" 2>/dev/null || true
                    ip addr add 192.168.42.100/24 dev "$IFACE" 2>/dev/null || true
                    sleep 0.3
                    echo "  [NCM-watcher] $(ip -brief addr show $IFACE 2>/dev/null)"
                    LAST_CONFIGURED="$IFACE"
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

cleanup() {
    echo ""
    echo "Cleaning up NCM watcher..."
    kill $NCM_WATCHER_PID 2>/dev/null || true
    wait $NCM_WATCHER_PID 2>/dev/null || true
    echo "Done."
}
trap cleanup EXIT INT TERM

echo ""
echo "[2/2] Starting Aria Hand Tracker as user $REAL_USER..."
echo "  (Make sure 'usbipd attach --busid 1-20 --auto-attach --wsl' is running on Windows!)"
echo ""

# Run the hand tracker as the real user
su - "$REAL_USER" -c "cd '$SCRIPT_DIR' && source aria_env/bin/activate && python -m aria_teleop.aria_hand_tracker $EXTRA_ARGS"
