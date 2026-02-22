#!/bin/bash
echo "Setting up Dual CAN Interfaces..."

# Setup can0
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000
echo "can0: UP"

# Setup can1
sudo ip link set can1 down
sudo ip link set can1 up type can bitrate 1000000
echo "can1: UP"

echo "Done! Connect Master Arm to CAN0 and Slave Arm to CAN1."
