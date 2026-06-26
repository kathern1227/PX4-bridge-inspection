#!/bin/bash
# Start PX4 SITL Gazebo simulation and takeoff to 10m

PX4_DIR="/home/cby/PX4-Autopilot"
cd "$PX4_DIR"

# Start simulation in background
make px4_sitl gazebo &
SIM_PID=$!

echo "Waiting for PX4 to initialize (30s)..."
sleep 30

# Send commands through PX4 shell via UDP mavlink console
# PX4 SITL listens on UDP 14540 for mavlink connections
# Use mavlink_shell.py if available, otherwise try direct approach

if command -v python3 &> /dev/null; then
    pip3 install --user pymavlink 2>/dev/null
    if python3 -c "import pymavlink" 2>/dev/null; then
        echo "Using mavlink_shell.py to send commands..."
        python3 "$PX4_DIR/Tools/mavlink_shell.py" 127.0.0.1:14540 --command "param set MIS_TAKEOFF_ALT 10" 2>/dev/null
        sleep 2
        python3 "$PX4_DIR/Tools/mavlink_shell.py" 127.0.0.1:14540 --command "commander arm" 2>/dev/null
        sleep 3
        python3 "$PX4_DIR/Tools/mavlink_shell.py" 127.0.0.1:14540 --command "commander takeoff" 2>/dev/null
        echo "Takeoff commands sent via mavlink_shell!"
        wait $SIM_PID
        exit 0
    fi
fi

# Fallback: try to send commands via the PX4 UNIX socket
echo "Trying socket approach..."
for cmd in "param set MIS_TAKEOFF_ALT 10" "commander arm" "commander takeoff"; do
    echo "$cmd" | socat - UNIX-CONNECT:/tmp/px4-sock-0 2>/dev/null
    sleep 2
done
echo "Commands sent via socket!"

wait $SIM_PID
