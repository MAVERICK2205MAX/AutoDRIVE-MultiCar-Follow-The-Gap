#!/usr/bin/env bash
################################################################################
# Launch AutoDRIVE 2026 (RoboRacer Sim Racing Edition) + Follow The Gap
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

echo "=== Starting 2026 AutoDRIVE RoboRacer Bridge (Port 4567) ==="
ros2 launch autodrive_roboracer bringup_headless.launch.py > /tmp/autodrive_roboracer_bridge.log 2>&1 &
BRIDGE_PID=$!

sleep 3

echo "=== Starting Follow The Gap Controller for RoboRacer 2026 ==="
ros2 run gap_follow reactive_node_multicar --ros-args \
  -p car_id:=roboracer_1 \
  -p lidar_topic:=/autodrive/roboracer_1/lidar \
  -p throttle_topic:=/autodrive/roboracer_1/throttle_command \
  -p steering_topic:=/autodrive/roboracer_1/steering_command \
  -p processed_scan_topic:=/roboracer_1/processed_scan \
  -p marker_topic:=/roboracer_1/visualization_marker \
  > /tmp/autodrive_gap_follow_2026.log 2>&1 &
GAP_PID=$!

sleep 2

# Default to practice simulator if available, else compete
SIM_BIN="$HOME/Downloads/AutoDRIVE_Sim_Practice/autodrive_simulator/AutoDRIVE Simulator.x86_64"
if [ ! -f "$SIM_BIN" ]; then
    SIM_BIN="$HOME/Downloads/AutoDRIVE_Sim_Compete/autodrive_simulator/AutoDRIVE Simulator.x86_64"
fi

echo "=== Launching 2026 AutoDRIVE Simulator 3D Window ==="
"$SIM_BIN" -screen-fullscreen 0 -screen-width 1280 -screen-height 720 > /tmp/sim_2026.log 2>&1 &
SIM_PID=$!

cat << 'EOF'

==========================================================================
 2026 AUTODRIVE ROBORACER (SIM RACING EDITION) IS RUNNING!
==========================================================================
Simulator: 2026 IROS / RoboRacer Edition
ROS 2 Bridge: autodrive_roboracer (Port 4567)
Controller: Follow The Gap (gap_follow) -> /autodrive/roboracer_1/

In the 2026 Simulator Window on your screen:
 1. Click [Disconnected] (below Port Number) -> turns to [Connected].
 2. Click [Manual] (compass icon) -> turns to [Autonomous].
    The car will drive autonomously using Follow The Gap!

To stop everything when done, run:
 ~/autodrive_ws/stop_autodrive.sh
==========================================================================
EOF

wait $SIM_PID
