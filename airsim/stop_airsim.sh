#!/bin/bash
# Stop AirSim and the virtual display.
pkill -f "Binaries/Linux/AirSimNH"; pkill -f "LinuxNoEditor/AirSimNH.sh"
for i in $(seq 1 10); do pgrep -f "Binaries/Linux/AirSimNH" >/dev/null || break; sleep 1; done
pkill -9 -f "Binaries/Linux/AirSimNH" 2>/dev/null
pkill -f "^Xvfb :99"
echo "AirSim stopped"
