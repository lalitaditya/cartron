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
        # If we can already ping the glasses, we are done
        if ping -c 1 -W 0.5 192.168.42.129 &>/dev/null; then
            sleep 2
            continue
        fi

        for IFACE in $(ip -brief link 2>/dev/null | grep -oP '^\S+' | grep -E '^(usb|enx|eth)'); do
            if [[ "$IFACE" == "eth0" ]] || [[ "$IFACE" == "eth1" ]]; then continue; fi

            MAC=$(cat /sys/class/net/$IFACE/address 2>/dev/null || echo "")
            if [[ "$MAC" != *"f2:3c"* ]]; then continue; fi
            
            STATE=$(cat /sys/class/net/$IFACE/operstate 2>/dev/null || echo "unknown")
            HAS_IP=$(ip -brief addr show "$IFACE" 2>/dev/null | grep -c '192.168.42' || true)
            
            if [ "$HAS_IP" = "0" ]; then
                echo "  [NCM-watcher] Attempting to link $IFACE (state=$STATE)..."
                ip link set "$IFACE" up 2>/dev/null || true
                sleep 0.5
                ip addr flush dev "$IFACE" 2>/dev/null || true
                ip addr add 192.168.42.100/24 dev "$IFACE" 2>/dev/null || true
                
                # Check if this interface actually reaches the glasses
                if ping -I "$IFACE" -c 1 -W 1 192.168.42.129 &>/dev/null; then
                    echo "  [NCM-watcher] SUCCESS: $IFACE is the Aria link!"
                    break # Stop looking, we found it!
                else
                    echo "  [NCM-watcher] $IFACE failed ping, reverting..."
                    ip addr flush dev "$IFACE" 2>/dev/null || true
                fi
            fi
        done
        sleep 1
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

# Ensure DISPLAY is set for WSLg visuals
export DISPLAY=${DISPLAY:-:0}
export WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-wayland-0}

# Run the hand tracker as the real user
su - "$REAL_USER" -c "cd '$SCRIPT_DIR' && export DISPLAY='$DISPLAY' && export WAYLAND_DISPLAY='$WAYLAND_DISPLAY' && source aria_env/bin/activate && python -m aria_teleop.aria_hand_tracker $EXTRA_ARGS"
