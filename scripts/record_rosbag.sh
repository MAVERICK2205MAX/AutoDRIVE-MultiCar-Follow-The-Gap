#!/usr/bin/env bash
################################################################################
# ROS 2 Bag Recording Script for AutoDRIVE Multi-Car F1TENTH Racing
# Records all active simulation & controller topics:
#   - LiDAR scans (/autodrive/f1tenth_1/lidar, /autodrive/f1tenth_2/lidar)
#   - Vehicle commands (/autodrive/f1tenth_*/throttle_command, steering_command)
#   - Odometry & IPS positions (/autodrive/f1tenth_*/ips, /imu)
#   - FTG processed scans & visualization markers
################################################################################

set -e

# Default output directory and duration
OUTPUT_NAME="${1:-f1tenth_run_01}"
DURATION="${2:-15}" # seconds (set 0 for manual Ctrl+C)
BASE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
ROSBAG_DIR="$BASE_DIR/rosbags"

mkdir -p "$ROSBAG_DIR"
TARGET_PATH="$ROSBAG_DIR/$OUTPUT_NAME"

source /opt/ros/humble/setup.bash
WS_DIR="${AUTODRIVE_WS:-$HOME/autodrive_ws}"
if [ -f "$WS_DIR/install/setup.bash" ]; then
  source "$WS_DIR/install/setup.bash"
fi

if [ -d "$TARGET_PATH" ]; then
  echo "⚠️ Target directory $TARGET_PATH already exists. Backing up..."
  mv "$TARGET_PATH" "${TARGET_PATH}_backup_$(date +%s)"
fi

echo "=========================================================================="
echo " Starting ROS 2 Bag Recording: $OUTPUT_NAME"
echo " Destination: $TARGET_PATH"
echo " Duration:    ${DURATION}s (or press Ctrl+C to stop earlier)"
echo "=========================================================================="

if [ "$DURATION" -gt 0 ]; then
  # Record for specified duration with graceful SIGINT
  timeout -s SIGINT "${DURATION}s" ros2 bag record -a -o "$TARGET_PATH" || true
else
  # Record until user presses Ctrl+C
  ros2 bag record -a -o "$TARGET_PATH"
fi

echo ""
echo "=========================================================================="
echo " Recording complete! Inspecting bag information:"
echo "=========================================================================="
ros2 bag info "$TARGET_PATH"

echo ""
echo "To replay this rosbag, run:"
echo "  ros2 bag play $TARGET_PATH"
echo "To replay at half speed:"
echo "  ros2 bag play $TARGET_PATH --rate 0.5"
