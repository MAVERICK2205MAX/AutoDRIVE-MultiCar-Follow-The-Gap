# Copyright (c) 2023, Tinker Twins
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

################################################################################

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='autodrive_f1tenth',
            executable='autodrive_incoming_bridge',
            name='autodrive_incoming_bridge',
            emulate_tty=True,
            output='screen',
        ),
        Node(
            package='autodrive_f1tenth',
            executable='autodrive_outgoing_bridge',
            name='autodrive_outgoing_bridge',
            emulate_tty=True,
            output='screen',
        ),
        Node(
            package='autodrive_f1tenth',
            executable='autodrive_multicar_sim',
            name='autodrive_multicar_sim',
            emulate_tty=True,
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', [FindPackageShare("autodrive_f1tenth"), '/rviz', '/simulator.rviz',]]
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': '/home/admin1/autodrive_ws/maps/levineB.yaml'}]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{'use_sim_time': False},
                        {'autostart': True},
                        {'node_names': ['map_server']}]
        ),
        Node(
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
        ),
        Node(
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
        ),
    ])