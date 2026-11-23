#!/usr/bin/env bash
################################################################################
# Launch Tutorial 4: Follow The Gap Controller in AutoDRIVE Simulator
################################################################################

set -e

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
bash "$DIR/stop_autodrive.sh"

echo "=== Sourcing ROS 2 and Virtual Environment ==="
source /opt/ros/humble/setup.bash
source "$DIR/venv/bin/activate"
source "$DIR/install/setup.bash"

export DISPLAY=${DISPLAY:-:0}
unset RMW_IMPLEMENTATION

echo "=== Starting AutoDRIVE ROS 2 Bridge (Port 4567) ==="
ros2 launch autodrive_f1tenth simulator_bringup_headless.launch.py > /tmp/autodrive_bridge.log 2>&1 &
BRIDGE_PID=$!

sleep 3

echo "=== Starting Follow The Gap Controller (Tutorial 4) ==="
ros2 run gap_follow reactive_node > /tmp/autodrive_gap_follow.log 2>&1 &
GAP_PID=$!

sleep 2

echo "=== Launching AutoDRIVE Simulator 3D Window ==="
SIM_BIN="$HOME/Downloads/AutoDRIVE_Sim/AutoDRIVE Simulator [Linux]/AutoDRIVE Simulator.x86_64"
"$SIM_BIN" -screen-fullscreen 0 -screen-width 1280 -screen-height 720 > /tmp/sim.log 2>&1 &
SIM_PID=$!

cat << 'EOF'

==========================================================================
 AUTODRIVE SIMULATOR (TUTORIAL 4 FOLLOW THE GAP) IS RUNNING!
==========================================================================
In the AutoDRIVE Simulator window on your screen:

 1. Look at the button below "Port Number" (it shows "Disconnected"):
    Click it! -> Status changes to "Connected".

 2. Click the [Manual] compass icon below "Connected":
    -> Status changes to "Autonomous".
    -> Car immediately begins driving autonomously with Follow The Gap!

To stop everything when done, run:
 ~/autodrive_ws/stop_autodrive.sh
==========================================================================
EOF

wait $SIM_PID
