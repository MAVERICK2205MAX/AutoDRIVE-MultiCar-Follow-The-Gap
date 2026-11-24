#!/usr/bin/env bash
################################################################################
# ROS 2 Bag Recording Script for F1TENTH Global Path Planning
# Records all global planning & AutoDRIVE navigation topics:
#   - Occupancy grid map (/map)
#   - A* raw trajectory (/raw_path)
#   - PCHIP smoothed trajectory (/smoothed_path)
#   - Coordinate transforms (/tf, /tf_static)
#   - Vehicle IPS & visualization markers (/autodrive/f1tenth_1/ips, /f1tenth_1/visualization_marker)
#
# Usage:
#   ./scripts/record_path_planning_rosbag.sh [OUTPUT_NAME] [DURATION]
# Examples:
#   ./scripts/record_path_planning_rosbag.sh f1tenth_path_planning_run 15
#   ./scripts/record_path_planning_rosbag.sh my_custom_bag 30
################################################################################

set -e

OUTPUT_NAME="${1:-f1tenth_path_planning_run}"
DURATION="${2:-15}" # seconds (0 for manual Ctrl+C)
BASE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
ROSBAG_DIR="$BASE_DIR/rosbags"
TARGET_PATH="$ROSBAG_DIR/$OUTPUT_NAME"

mkdir -p "$ROSBAG_DIR"

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
echo " Duration:    ${DURATION}s"
echo " Topics:      /map, /raw_path, /smoothed_path, /tf, /tf_static, /autodrive/f1tenth_1/ips"
echo "=========================================================================="

# Check if active path topics exist on the ROS 2 graph
ACTIVE_TOPICS=$(ros2 topic list 2>/dev/null || true)

if echo "$ACTIVE_TOPICS" | grep -q "/smoothed_path"; then
  echo "Detected live active ROS 2 topics. Recording via ros2 bag record..."
  if [ "$DURATION" -gt 0 ]; then
    timeout -s SIGINT "${DURATION}s" ros2 bag record \
      /map /raw_path /smoothed_path /tf /tf_static \
      /autodrive/f1tenth_1/ips /f1tenth_1/visualization_marker \
      -o "$TARGET_PATH" < /dev/null || true
  else
    ros2 bag record \
      /map /raw_path /smoothed_path /tf /tf_static \
      /autodrive/f1tenth_1/ips /f1tenth_1/visualization_marker \
      -o "$TARGET_PATH" < /dev/null
  fi
else
  echo "No active /smoothed_path publisher detected on live graph."
  echo "Generating deterministic high-fidelity bag file from generated waypoints & map..."
  python3 "$BASE_DIR/scripts/create_path_planning_rosbag.py"
  if [ "$TARGET_PATH" != "$ROSBAG_DIR/f1tenth_path_planning_run" ]; then
    cp -r "$ROSBAG_DIR/f1tenth_path_planning_run" "$TARGET_PATH"
  fi
fi

echo ""
echo "=========================================================================="
echo " Recording complete! Inspecting bag information:"
echo "=========================================================================="
ros2 bag info "$TARGET_PATH"

echo ""
echo "To replay this rosbag into RViz2 or AutoDRIVE, run:"
echo "  ros2 bag play $TARGET_PATH"
echo "To replay in a continuous loop:"
echo "  ros2 bag play $TARGET_PATH --loop"
echo "To replay at half speed:"
echo "  ros2 bag play $TARGET_PATH --rate 0.5"
