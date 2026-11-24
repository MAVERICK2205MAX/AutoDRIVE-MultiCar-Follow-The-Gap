#!/usr/bin/env python3
"""
Create a comprehensive ROS 2 bag file for Tutorial 5: Global Path Planning.
Generates f1tenth_path_planning_run with:
  - /map: OccupancyGrid of the circuit (F1tenth_Map_walled)
  - /raw_path: nav_msgs/Path (A* raw trajectory, 269 waypoints)
  - /smoothed_path: nav_msgs/Path (PCHIP smoothed trajectory, 263 waypoints)
  - /tf: Dynamic transform (map -> f1tenth_1) traversing the path at 20 Hz
  - /tf_static: Static transform (map -> odom)
  - /autodrive/f1tenth_1/ips: Vehicle IPS position over time
  - /f1tenth_1/visualization_marker: 3D bounding box marker of the vehicle
  - /rosout: Node status log messages
"""

import os
import sys
import math
import csv
import shutil
import time
import yaml
import cv2
import numpy as np

# Ensure ROS 2 Humble packages are loaded
try:
    import rclpy
    from rclpy.serialization import serialize_message
    from rclpy.time import Time
    import rosbag2_py
    from geometry_msgs.msg import PoseStamped, Point, Quaternion, TransformStamped
    from nav_msgs.msg import Path, OccupancyGrid
    from tf2_msgs.msg import TFMessage
    from visualization_msgs.msg import Marker
    from rcl_interfaces.msg import Log
except ImportError as e:
    print(f"Error importing ROS 2 libraries: {e}")
    print("Please run with: source /opt/ros/humble/setup.bash")
    sys.exit(1)


def read_csv(path):
    rows = []
    with open(path, newline='') as f:
        for r in csv.DictReader(f):
            yaw = float(r['yaw']) if 'yaw' in r and r['yaw'] != '' else None
            rows.append((float(r['x']), float(r['y']), yaw))
    for i, (x, y, yaw) in enumerate(rows):
        if yaw is None:
            a = rows[i]
            b = rows[min(i + 1, len(rows) - 1)]
            if b == a and i > 0:
                a, b = rows[i - 1], rows[i]
            rows[i] = (x, y, math.atan2(b[1] - a[1], b[0] - a[0]))
    return rows


def build_occupancy_grid(yaml_path):
    with open(yaml_path, 'r') as f:
        meta = yaml.safe_load(f)

    img_dir = os.path.dirname(yaml_path)
    img_path = os.path.join(img_dir, meta['image'])
    img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    h, w = img.shape

    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.resolution = float(meta['resolution'])
    grid.info.width = w
    grid.info.height = h
    grid.info.origin.position.x = float(meta['origin'][0])
    grid.info.origin.position.y = float(meta['origin'][1])
    grid.info.origin.position.z = float(meta['origin'][2])
    grid.info.origin.orientation.w = 1.0

    # ROS OccupancyGrid origin is bottom-left, PGM origin is top-left
    flipped = cv2.flip(img, 0)
    occ = np.zeros_like(flipped, dtype=np.int8)
    occ_thresh = meta['occupied_thresh'] * 255.0
    free_thresh = meta['free_thresh'] * 255.0

    occ[flipped >= occ_thresh] = 0    # free
    occ[flipped <= free_thresh] = 100  # occupied
    occ[(flipped > free_thresh) & (flipped < occ_thresh)] = -1 # unknown

    grid.data = occ.flatten().tolist()
    return grid


def create_path_msg(rows, frame_id='map'):
    msg = Path()
    msg.header.frame_id = frame_id
    for x, y, yaw in rows:
        p = PoseStamped()
        p.header.frame_id = frame_id
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.position.z = 0.0
        p.pose.orientation.z = math.sin(yaw / 2.0)
        p.pose.orientation.w = math.cos(yaw / 2.0)
        msg.poses.append(p)
    return msg


def main():
    base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    output_dir = os.path.join(base_dir, 'rosbags', 'f1tenth_path_planning_run')
    map_yaml = os.path.expanduser('~/autodrive_ws/maps/F1tenth_Map_walled.yaml')
    if not os.path.exists(map_yaml):
        map_yaml = os.path.join(base_dir, 'maps', 'F1tenth_Map_walled.yaml')

    raw_csv = os.path.expanduser('~/autodrive_ws/waypoints/raw_path.csv')
    if not os.path.exists(raw_csv):
        raw_csv = os.path.join(base_dir, 'waypoints', 'raw_path.csv')

    smooth_csv = os.path.expanduser('~/autodrive_ws/waypoints/smoothed_path.csv')
    if not os.path.exists(smooth_csv):
        smooth_csv = os.path.join(base_dir, 'waypoints', 'smoothed_path.csv')

    print(f"Generating ROS 2 bag file: {output_dir}")
    print(f"  Map: {map_yaml}")
    print(f"  Raw waypoints: {raw_csv}")
    print(f"  Smoothed waypoints: {smooth_csv}")

    if os.path.exists(output_dir):
        print(f"Removing existing bag at {output_dir}...")
        shutil.rmtree(output_dir)

    # Load data
    grid_msg = build_occupancy_grid(map_yaml)
    raw_rows = read_csv(raw_csv)
    smooth_rows = read_csv(smooth_csv)
    raw_path_msg = create_path_msg(raw_rows)
    smooth_path_msg = create_path_msg(smooth_rows)

    # Initialize sequential writer
    writer = rosbag2_py.SequentialWriter()
    storage_options = rosbag2_py.StorageOptions(uri=output_dir, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    writer.open(storage_options, converter_options)

    # Define QoS profile strings
    transient_local_qos = (
        "- history: 3\n"
        "  depth: 1\n"
        "  reliability: 1\n"
        "  durability: 1\n"
        "  deadline:\n    sec: 9223372036\n    nsec: 854775807\n"
        "  lifespan:\n    sec: 9223372036\n    nsec: 854775807\n"
        "  liveliness: 1\n"
        "  liveliness_lease_duration:\n    sec: 9223372036\n    nsec: 854775807\n"
        "  avoid_ros_namespace_conventions: false"
    )
    volatile_qos = (
        "- history: 3\n"
        "  depth: 10\n"
        "  reliability: 1\n"
        "  durability: 2\n"
        "  deadline:\n    sec: 9223372036\n    nsec: 854775807\n"
        "  lifespan:\n    sec: 9223372036\n    nsec: 854775807\n"
        "  liveliness: 1\n"
        "  liveliness_lease_duration:\n    sec: 9223372036\n    nsec: 854775807\n"
        "  avoid_ros_namespace_conventions: false"
    )

    # Define topics with their QoS profile
    topics = [
        ('/map', 'nav_msgs/msg/OccupancyGrid', transient_local_qos),
        ('/raw_path', 'nav_msgs/msg/Path', transient_local_qos),
        ('/smoothed_path', 'nav_msgs/Path', transient_local_qos),
        ('/tf_static', 'tf2_msgs/msg/TFMessage', transient_local_qos),
        ('/tf', 'tf2_msgs/msg/TFMessage', volatile_qos),
        ('/autodrive/f1tenth_1/ips', 'geometry_msgs/msg/Point', volatile_qos),
        ('/f1tenth_1/visualization_marker', 'visualization_msgs/msg/Marker', volatile_qos),
        ('/rosout', 'rcl_interfaces/msg/Log', volatile_qos),
    ]

    for name, type_str, qos in topics:
        topic_meta = rosbag2_py.TopicMetadata(
            name=name,
            type=type_str,
            serialization_format='cdr',
            offered_qos_profiles=qos
        )
        writer.create_topic(topic_meta)

    # Base timestamp: current epoch time in nanoseconds
    start_time_sec = time.time()
    start_ns = int(start_time_sec * 1e9)
    duration_sec = 15.0
    hz = 20.0
    num_steps = int(duration_sec * hz)
    dt_ns = int((1.0 / hz) * 1e9)

    # Set stamps for map and paths
    grid_msg.header.stamp.sec = int(start_time_sec)
    grid_msg.header.stamp.nanosec = int((start_time_sec - int(start_time_sec)) * 1e9)
    raw_path_msg.header.stamp = grid_msg.header.stamp
    smooth_path_msg.header.stamp = grid_msg.header.stamp

    # Write initial static / latched messages
    writer.write('/map', serialize_message(grid_msg), start_ns)
    writer.write('/raw_path', serialize_message(raw_path_msg), start_ns + 10_000_000)
    writer.write('/smoothed_path', serialize_message(smooth_path_msg), start_ns + 20_000_000)

    # Static TF: map -> odom
    tf_static = TFMessage()
    t_static = TransformStamped()
    t_static.header.stamp = grid_msg.header.stamp
    t_static.header.frame_id = 'map'
    t_static.child_frame_id = 'odom'
    t_static.transform.rotation.w = 1.0
    tf_static.transforms.append(t_static)
    writer.write('/tf_static', serialize_message(tf_static), start_ns + 30_000_000)

    # Initial log message
    log_msg = Log()
    log_msg.stamp = grid_msg.header.stamp
    log_msg.level = 20 # INFO
    log_msg.name = 'global_planner'
    log_msg.msg = f'A* path computed: {len(raw_rows)} waypoints, PCHIP smoothed: {len(smooth_rows)} waypoints'
    writer.write('/rosout', serialize_message(log_msg), start_ns + 40_000_000)

    # Generate vehicle traversal along the smoothed trajectory
    smooth_pts = np.array([(x, y, yaw) for x, y, yaw in smooth_rows])
    num_waypoints = len(smooth_pts)

    print(f"Writing {num_steps} dynamic time steps ({duration_sec}s @ {hz} Hz)...")
    for step in range(num_steps):
        current_ns = start_ns + step * dt_ns
        cur_sec = int(current_ns // 1e9)
        cur_nsec = int(current_ns % 1e9)

        # Interpolate along smoothed path
        progress = step / float(num_steps - 1)
        idx_float = progress * (num_waypoints - 1)
        idx_low = int(math.floor(idx_float))
        idx_high = min(idx_low + 1, num_waypoints - 1)
        alpha = idx_float - idx_low

        p1 = smooth_pts[idx_low]
        p2 = smooth_pts[idx_high]

        cur_x = (1.0 - alpha) * p1[0] + alpha * p2[0]
        cur_y = (1.0 - alpha) * p1[1] + alpha * p2[1]

        # Angular interpolation (shortest angle)
        diff_yaw = (p2[2] - p1[2] + math.pi) % (2 * math.pi) - math.pi
        cur_yaw = p1[2] + alpha * diff_yaw

        # Dynamic TF: map -> f1tenth_1
        tf_dynamic = TFMessage()
        t_dyn = TransformStamped()
        t_dyn.header.stamp.sec = cur_sec
        t_dyn.header.stamp.nanosec = cur_nsec
        t_dyn.header.frame_id = 'map'
        t_dyn.child_frame_id = 'f1tenth_1'
        t_dyn.transform.translation.x = cur_x
        t_dyn.transform.translation.y = cur_y
        t_dyn.transform.translation.z = 0.05
        t_dyn.transform.rotation.z = math.sin(cur_yaw / 2.0)
        t_dyn.transform.rotation.w = math.cos(cur_yaw / 2.0)
        tf_dynamic.transforms.append(t_dyn)
        writer.write('/tf', serialize_message(tf_dynamic), current_ns)

        # IPS Point
        ips_pt = Point(x=cur_x, y=cur_y, z=0.0)
        writer.write('/autodrive/f1tenth_1/ips', serialize_message(ips_pt), current_ns)

        # 3D F1TENTH Vehicle Marker
        marker = Marker()
        marker.header.stamp.sec = cur_sec
        marker.header.stamp.nanosec = cur_nsec
        marker.header.frame_id = 'map'
        marker.ns = 'f1tenth_vehicle'
        marker.id = 1
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = cur_x
        marker.pose.position.y = cur_y
        marker.pose.position.z = 0.06
        marker.pose.orientation.z = math.sin(cur_yaw / 2.0)
        marker.pose.orientation.w = math.cos(cur_yaw / 2.0)
        marker.scale.x = 0.53  # F1TENTH vehicle length (~53 cm)
        marker.scale.y = 0.28  # F1TENTH vehicle width (~28 cm)
        marker.scale.z = 0.12  # F1TENTH vehicle height
        marker.color.r = 0.0
        marker.color.g = 0.85
        marker.color.b = 1.0   # Cyan / AutoDRIVE blue
        marker.color.a = 0.95
        writer.write('/f1tenth_1/visualization_marker', serialize_message(marker), current_ns)

        # Periodically publish map/path updates and log
        if step % int(hz) == 0:
            writer.write('/map', serialize_message(grid_msg), current_ns)
            writer.write('/raw_path', serialize_message(raw_path_msg), current_ns)
            writer.write('/smoothed_path', serialize_message(smooth_path_msg), current_ns)

    del writer
    print(" Successfully wrote ROS 2 bag file!")


if __name__ == '__main__':
    main()
