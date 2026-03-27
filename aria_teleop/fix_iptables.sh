#!/bin/bash
# Fix firewall/iptables for Aria streaming in WSL
echo "Opening UDP ports 6000-9000 for Aria streaming..."
sudo iptables -A INPUT -p udp -m udp --dport 6000:9000 -j ACCEPT
sudo iptables -A INPUT -p tcp -m tcp --dport 6000:9000 -j ACCEPT
echo "Done. Checking iptables rules:"
sudo iptables -L INPUT -n | grep -E "6[0-9]{3}|7[0-9]{3}|8[0-9]{3}|9000"
echo ""
echo "Now try running the streamer again."
