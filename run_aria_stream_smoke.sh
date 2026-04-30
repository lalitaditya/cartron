#!/bin/bash
# Minimal non-GUI Aria streaming callback smoke test with USB-NCM setup.

set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

REAL_USER="${SUDO_USER:-$(whoami)}"
NCM_IFACE_FILE="$SCRIPT_DIR/.aria_ncm_iface"

if [ "$(id -u)" -ne 0 ]; then
    echo "ERROR: Must run as root. Use: sudo bash $0"
    exit 1
fi

echo "=== Aria Stream Smoke Launcher ==="
rm -f "$NCM_IFACE_FILE"

echo "[0/2] Opening Linux firewall ports for DDS..."
iptables -C INPUT -p udp -m udp --dport 6000:9000 -j ACCEPT 2>/dev/null || \
    iptables -A INPUT -p udp -m udp --dport 6000:9000 -j ACCEPT
iptables -C INPUT -p tcp -m tcp --dport 6000:9000 -j ACCEPT 2>/dev/null || \
    iptables -A INPUT -p tcp -m tcp --dport 6000:9000 -j ACCEPT
iptables -C INPUT -s 192.168.42.129 -p udp -j ACCEPT 2>/dev/null || \
    iptables -A INPUT -s 192.168.42.129 -p udp -j ACCEPT
iptables -C INPUT -s 192.168.42.129 -p tcp -j ACCEPT 2>/dev/null || \
    iptables -A INPUT -s 192.168.42.129 -p tcp -j ACCEPT

(
    LAST_CONFIGURED=""
    while true; do
        for IFACE in $(ip -brief link 2>/dev/null | grep -oP '^\S+' | grep -E '^(usb|enx)'); do
            STATE=$(cat /sys/class/net/$IFACE/operstate 2>/dev/null || echo "unknown")
            HAS_IP=$(ip -brief addr show "$IFACE" 2>/dev/null | grep -c '192.168.42' || true)

            if [ "$STATE" != "up" ] || [ "$HAS_IP" = "0" ]; then
                if [ "$IFACE" != "$LAST_CONFIGURED" ]; then
                    echo "  [NCM-watcher] Detected $IFACE (state=$STATE, has_ip=$HAS_IP), configuring..."
                    ip link set "$IFACE" up 2>/dev/null || true
                    sleep 0.3
                    ip addr flush dev "$IFACE" 2>/dev/null || true
                    ip addr add 192.168.42.100/24 dev "$IFACE" 2>/dev/null || true
                    ip route replace 192.168.42.129 dev "$IFACE" 2>/dev/null || true
                    sleep 0.3
                    ip -brief addr show "$IFACE" 2>/dev/null || true
                    echo "$IFACE" > "$NCM_IFACE_FILE"
                    LAST_CONFIGURED="$IFACE"
                fi
            fi
        done
        sleep 0.2
    done
) &
NCM_WATCHER_PID=$!

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

(
    for _ in $(seq 1 120); do
        if [ -s "$NCM_IFACE_FILE" ]; then
            IFACE=$(cat "$NCM_IFACE_FILE")
            echo "Starting UDP DDS packet monitor on $IFACE..."
            if command -v tcpdump >/dev/null 2>&1; then
                timeout 35 tcpdump -n -i "$IFACE" 'udp and (host 192.168.42.129 or net 239.255.0.0/16)' -c 60
            else
                echo "tcpdump not installed; install with: sudo apt-get install tcpdump"
            fi
            exit 0
        fi
        sleep 0.5
    done
) &
TCPDUMP_PID=$!

cleanup() {
    echo ""
    echo "Cleaning up..."
    kill $NCM_WATCHER_PID 2>/dev/null || true
    kill $TCPDUMP_PID 2>/dev/null || true
    wait $NCM_WATCHER_PID 2>/dev/null || true
    echo "Done."
}
trap cleanup EXIT INT TERM

su - "$REAL_USER" -c "cd '$SCRIPT_DIR' && export FASTRTPS_DEFAULT_PROFILES_FILE='$FASTRTPS_DEFAULT_PROFILES_FILE' && export FASTDDS_DEFAULT_PROFILES_FILE='$FASTDDS_DEFAULT_PROFILES_FILE' && source aria_env/bin/activate && python aria_teleop/aria_stream_smoke.py --client-mode standalone --data rgb-eye --ephemeral-certs"
