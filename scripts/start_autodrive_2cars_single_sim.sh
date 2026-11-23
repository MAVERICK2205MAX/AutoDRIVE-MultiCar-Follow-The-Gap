#!/usr/bin/env bash
################################################################################
# Launch TWO F1TENTH Cars in ONE SINGLE AutoDRIVE Simulator Window
# Follow The Gap Controller Running on Both Cars Simultaneously in Trackcam Mode
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

echo "=== Starting Follow The Gap Controller for Car 1 (f1tenth_1) ==="
ros2 run gap_follow reactive_node_multicar --ros-args \
  -r __node:=gap_follow_car1 \
  -p car_id:=f1tenth_1 \
  -p lidar_topic:=/autodrive/f1tenth_1/lidar \
  -p throttle_topic:=/autodrive/f1tenth_1/throttle_command \
  -p steering_topic:=/autodrive/f1tenth_1/steering_command \
  -p processed_scan_topic:=/f1tenth_1/processed_scan \
  -p marker_topic:=/f1tenth_1/visualization_marker \
  -p straight_throttle:=0.22 \
  -p turning_throttle:=0.12 \
  > /tmp/gap_car1.log 2>&1 &

echo "=== Starting Follow The Gap Controller for Car 2 (f1tenth_2) ==="
ros2 run gap_follow reactive_node_multicar --ros-args \
  -r __node:=gap_follow_car2 \
  -p car_id:=f1tenth_2 \
  -p lidar_topic:=/autodrive/f1tenth_2/lidar \
  -p throttle_topic:=/autodrive/f1tenth_2/throttle_command \
  -p steering_topic:=/autodrive/f1tenth_2/steering_command \
  -p processed_scan_topic:=/f1tenth_2/processed_scan \
  -p marker_topic:=/f1tenth_2/visualization_marker \
  -p straight_throttle:=0.18 \
  -p turning_throttle:=0.10 \
  > /tmp/gap_car2.log 2>&1 &

sleep 2

SIM_BIN="$HOME/Downloads/AutoDRIVE_Sim_MARL_F1TENTH/AutoDRIVE Simulator.x86_64"

echo "=== Launching 2-Car AutoDRIVE Simulator Window ==="
"$SIM_BIN" -screen-fullscreen 0 -screen-width 1280 -screen-height 720 > /tmp/sim_marl.log 2>&1 &
SIM_PID=$!

echo "=== Auto-interacting with Simulator GUI (Connect, Autonomous & Trackcam) ==="
"$DIR/venv/bin/python" "$DIR/auto_interact_marl.py" > /tmp/auto_interact.log 2>&1 &

cat << 'EOF'

==========================================================================
 TWO CARS IN ONE SINGLE AUTODRIVE SIMULATOR ARE NOW RUNNING!
==========================================================================
Scene: AutoDRIVE MRDT-MARL F1TENTH (Single Window)
Mode: Trackcam (Overhead Track Mode)
Car 1: f1tenth_1 (Black & Red) -> Follow The Gap Leader
Car 2: f1tenth_2 (Blue)        -> Follow The Gap Chaser (Collision-Free Pacing)

To record a video anytime, run:
  bash ~/autodrive_ws/record_video.sh [duration_in_seconds]

==========================================================================
EOF

wait $SIM_PID
