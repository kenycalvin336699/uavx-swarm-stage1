#!/bin/bash
# Launch three ArduPilot SITL instances for UAV-X demo.

set -e
ARDUPILOT_DIR="${HOME}/ardupilot/ArduCopter"

if [ ! -d "$ARDUPILOT_DIR" ]; then
    echo "ERROR: ArduPilot not found at $ARDUPILOT_DIR"
    exit 1
fi

cd "$ARDUPILOT_DIR"

if command -v gnome-terminal &> /dev/null; then
    gnome-terminal --title="SITL Drone 1" -- bash -c \
        "../Tools/autotest/sim_vehicle.py -v ArduCopter -I0 --sysid 1 \
         --console --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551; exec bash"
    sleep 2
    gnome-terminal --title="SITL Drone 2" -- bash -c \
        "../Tools/autotest/sim_vehicle.py -v ArduCopter -I1 --sysid 2 \
         --console --out=udp:127.0.0.1:14560 --out=udp:127.0.0.1:14561; exec bash"
    sleep 2
    gnome-terminal --title="SITL Drone 3" -- bash -c \
        "../Tools/autotest/sim_vehicle.py -v ArduCopter -I2 --sysid 3 \
         --console --out=udp:127.0.0.1:14570 --out=udp:127.0.0.1:14571; exec bash"
else
    echo "gnome-terminal not found. Open three terminals manually:"
    echo ""
    echo "T1: cd ~/ardupilot/ArduCopter && ../Tools/autotest/sim_vehicle.py \\"
    echo "    -v ArduCopter -I0 --sysid 1 --console \\"
    echo "    --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551"
    echo "T2: cd ~/ardupilot/ArduCopter && ../Tools/autotest/sim_vehicle.py \\"
    echo "    -v ArduCopter -I1 --sysid 2 --console \\"
    echo "    --out=udp:127.0.0.1:14560 --out=udp:127.0.0.1:14561"
    echo "T3: cd ~/ardupilot/ArduCopter && ../Tools/autotest/sim_vehicle.py \\"
    echo "    -v ArduCopter -I2 --sysid 3 --console \\"
    echo "    --out=udp:127.0.0.1:14570 --out=udp:127.0.0.1:14571"
fi
