#!/usr/bin/env python3
"""
ROS 2 Launch file for Multi-Car Follow-The-Gap Navigation.
Launches two independent Follow-The-Gap reactive nodes running the exact same
algorithm, parameterized to control Car 1 (f1tenth_1) and Car 2 (f1tenth_2)
without topic or namespace collision.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Car 1 Follow The Gap Node (Lead Vehicle)
    car1_node = Node(
        package='gap_follow',
        executable='reactive_node_multicar',
        name='gap_follow_car1',
        output='screen',
        parameters=[{
            'car_id': 'f1tenth_1',
            'lidar_topic': '/autodrive/f1tenth_1/lidar',
            'throttle_topic': '/autodrive/f1tenth_1/throttle_command',
            'steering_topic': '/autodrive/f1tenth_1/steering_command',
            'processed_scan_topic': '/autodrive/f1tenth_1/processed_scan',
            'marker_topic': '/autodrive/f1tenth_1/visualization_marker',
            'view_angle_deg': 70.0,
            'max_lidar_range': 10.0,
            'min_obstacle_dist': 0.10,
            'bubble_radius': 18,
            'gap_threshold': 0.80,
            'straight_throttle': 0.25,
            'turning_throttle': 0.12,
            'straight_angle_threshold': 0.10
        }]
    )

    # Car 2 Follow The Gap Node (Trailing / Chasing Vehicle)
    # Uses the EXACT same algorithm, mapped to f1tenth_2 topics
    car2_node = Node(
        package='gap_follow',
        executable='reactive_node_multicar',
        name='gap_follow_car2',
        output='screen',
        parameters=[{
            'car_id': 'f1tenth_2',
            'lidar_topic': '/autodrive/f1tenth_2/lidar',
            'throttle_topic': '/autodrive/f1tenth_2/throttle_command',
            'steering_topic': '/autodrive/f1tenth_2/steering_command',
            'processed_scan_topic': '/autodrive/f1tenth_2/processed_scan',
            'marker_topic': '/autodrive/f1tenth_2/visualization_marker',
            'view_angle_deg': 70.0,
            'max_lidar_range': 10.0,
            'min_obstacle_dist': 0.10,
            'bubble_radius': 18,
            'gap_threshold': 0.80,
            'straight_throttle': 0.22,
            'turning_throttle': 0.11,
            'straight_angle_threshold': 0.10
        }]
    )

    return LaunchDescription([
        car1_node,
        car2_node
    ])
