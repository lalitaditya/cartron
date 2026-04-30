#!/bin/bash
# Launch the Aria EyeTrack viewer with USB-NCM auto-configuration.
#
# Usage:
#   sudo bash run_aria_eye_viewer.sh
#   sudo bash run_aria_eye_viewer.sh --gaze-source open-model --gaze-model-device cpu
#
# Keep watch_aria_usb.ps1 running in an Administrator PowerShell so Windows
# attaches both the normal USB device and the UsbNcm streaming device to WSL.

set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

REAL_USER="${SUDO_USER:-$(whoami)}"
EXTRA_ARGS="$*"

if [ "$(id -u)" -ne 0 ]; then
    echo "ERROR: Must run as root. Use: sudo bash $0"
    exit 1
fi

echo "=== Aria EyeTrack Viewer Launcher ==="
echo "Running as root, real user: $REAL_USER"
echo ""

echo "[0/2] Opening Linux firewall ports for DDS..."
iptables -C INPUT -p udp -m udp --dport 6000:9000 -j ACCEPT 2>/dev/null || \
    iptables -A INPUT -p udp -m udp --dport 6000:9000 -j ACCEPT
iptables -C INPUT -p tcp -m tcp --dport 6000:9000 -j ACCEPT 2>/dev/null || \
    iptables -A INPUT -p tcp -m tcp --dport 6000:9000 -j ACCEPT
iptables -C INPUT -s 192.168.42.129 -p udp -j ACCEPT 2>/dev/null || \
    iptables -A INPUT -s 192.168.42.129 -p udp -j ACCEPT
iptables -C INPUT -s 192.168.42.129 -p tcp -j ACCEPT 2>/dev/null || \
    iptables -A INPUT -s 192.168.42.129 -p tcp -j ACCEPT

echo "[1/2] Starting NCM interface watcher in background..."
(
    declare -A LAST_LOGGED
    while true; do
        for IFACE in $(ip -brief link 2>/dev/null | grep -oP '^\S+' | grep -E '^(usb|enx)'); do
            STATE=$(cat /sys/class/net/$IFACE/operstate 2>/dev/null || echo "unknown")
            HAS_IP=$(ip -brief addr show "$IFACE" 2>/dev/null | grep -c '192.168.42' || true)
            CAN_PING=0
            if ping -c 1 -W 1 192.168.42.129 &>/dev/null; then
                CAN_PING=1
            fi

            if [ "$STATE" != "up" ] || [ "$HAS_IP" = "0" ] || [ "$CAN_PING" = "0" ]; then
                KEY="${STATE}-${HAS_IP}-${CAN_PING}"
                if [ "${LAST_LOGGED[$IFACE]:-}" != "$KEY" ]; then
                    echo "  [NCM-watcher] Configuring $IFACE (state=$STATE, has_ip=$HAS_IP, reachable=$CAN_PING)..."
                    LAST_LOGGED[$IFACE]="$KEY"
                fi
                ip link set dev "$IFACE" down 2>/dev/null || true
                sleep 0.1
                ip addr flush dev "$IFACE" 2>/dev/null || true
                ip link set dev "$IFACE" up 2>/dev/null || true
                sleep 0.2
                ip addr replace 192.168.42.100/24 dev "$IFACE" 2>/dev/null || true
                ip route replace 192.168.42.129 dev "$IFACE" 2>/dev/null || true
                ip route flush cache 2>/dev/null || true
                sleep 0.4
                NEW_STATE=$(cat /sys/class/net/$IFACE/operstate 2>/dev/null || echo "?")
                NEW_IP=$(ip -brief addr show "$IFACE" 2>/dev/null)
                if ping -c 1 -W 1 192.168.42.129 &>/dev/null; then
                    echo "  [NCM-watcher] SUCCESS: $IFACE is reachable at 192.168.42.129 ($NEW_IP)"
                elif [ "$NEW_STATE" = "down" ]; then
                    echo "  [NCM-watcher] $IFACE is still DOWN after configure; will retry..."
                fi
            else
                LAST_LOGGED[$IFACE]="ok"
            fi
        done
        sleep 0.2
    done
) &
NCM_WATCHER_PID=$!
echo "  Watcher PID: $NCM_WATCHER_PID"

cat > "$SCRIPT_DIR/fastdds_aria_unicast.generated.xml" <<'XML'
<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <profiles>
    <participant profile_name="aria_wsl_participant" is_default_profile="true">
      <rtps>
        <builtin>
          <metatrafficUnicastLocatorList>
            <locator>
              <udpv4>
                <address>192.168.42.100</address>
              </udpv4>
            </locator>
          </metatrafficUnicastLocatorList>
          <metatrafficMulticastLocatorList>
            <locator>
              <udpv4>
                <address>239.255.0.1</address>
              </udpv4>
            </locator>
          </metatrafficMulticastLocatorList>
          <initialPeersList>
            <locator>
              <udpv4>
                <address>192.168.42.129</address>
              </udpv4>
            </locator>
          </initialPeersList>
        </builtin>
        <defaultUnicastLocatorList>
          <locator>
            <udpv4>
              <address>192.168.42.100</address>
            </udpv4>
          </locator>
        </defaultUnicastLocatorList>
        <defaultMulticastLocatorList>
          <locator>
            <udpv4>
              <address>239.255.0.1</address>
            </udpv4>
          </locator>
        </defaultMulticastLocatorList>
      </rtps>
    </participant>
  </profiles>
</dds>
XML
export FASTRTPS_DEFAULT_PROFILES_FILE="$SCRIPT_DIR/fastdds_aria_unicast.generated.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$SCRIPT_DIR/fastdds_aria_unicast.generated.xml"
echo "Using generated FastDDS profile: $FASTRTPS_DEFAULT_PROFILES_FILE"

cleanup() {
    echo ""
    echo "Cleaning up..."
    kill $NCM_WATCHER_PID 2>/dev/null || true
    wait $NCM_WATCHER_PID 2>/dev/null || true
    echo "Done."
}
trap cleanup EXIT INT TERM

echo ""
echo "[2/2] Starting EyeTrack viewer as user $REAL_USER..."
echo ""

export DISPLAY=${DISPLAY:-:0}
export WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-wayland-0}
export MPLCONFIGDIR="/tmp/aria_matplotlib"
mkdir -p "$MPLCONFIGDIR"
chown "$REAL_USER":"$REAL_USER" "$MPLCONFIGDIR" 2>/dev/null || true
if [ -d /tmp/aria_model_site ]; then
    export PYTHONPATH="/tmp/aria_model_site:${PYTHONPATH:-}"
fi

su - "$REAL_USER" -c "cd '$SCRIPT_DIR' && export DISPLAY='$DISPLAY' && export WAYLAND_DISPLAY='$WAYLAND_DISPLAY' && export MPLCONFIGDIR='$MPLCONFIGDIR' && export PYTHONPATH='$PYTHONPATH' && export FASTRTPS_DEFAULT_PROFILES_FILE='$FASTRTPS_DEFAULT_PROFILES_FILE' && export FASTDDS_DEFAULT_PROFILES_FILE='$FASTDDS_DEFAULT_PROFILES_FILE' && source aria_env/bin/activate && python aria_teleop/aria_eye_viewer.py --display matplotlib --ephemeral-certs $EXTRA_ARGS"
