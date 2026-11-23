#!/usr/bin/env python3

################################################################################

# Copyright (c) 2023, Tinker Twins
# All rights reserved.

################################################################################

# ROS 2 module imports
from std_msgs.msg import Int32, Float32 # Int32 and Float32 message classes
from geometry_msgs.msg import Point # Point message class
from sensor_msgs.msg import JointState, Imu, LaserScan, Image # JointState, Imu, LaserScan and Image message classes

# Python mudule imports
from attrdict import AttrDict # Mapping objects that allow their elements to be accessed both as keys and as attributes

################################################################################

# ROS 2 publishers and subscribers
pub_sub_dict = AttrDict({
    'subscribers': [
        # Vehicle 1 data subscribers
        {'topic':'/autodrive/f1tenth_1/throttle_command', 'type': Float32, 'name': 'sub_throttle_command'},
        {'topic':'/autodrive/f1tenth_1/steering_command', 'type': Float32, 'name': 'sub_steering_command'},
        # Vehicle 2 data subscribers
        {'topic':'/autodrive/f1tenth_2/throttle_command', 'type': Float32, 'name': 'sub_throttle_command_2'},
        {'topic':'/autodrive/f1tenth_2/steering_command', 'type': Float32, 'name': 'sub_steering_command_2'}
    ],
    'publishers': [
        # Vehicle 1 data publishers
        {'topic': '/autodrive/f1tenth_1/throttle', 'type': Float32, 'name': 'pub_throttle'},
        {'topic': '/autodrive/f1tenth_1/steering', 'type': Float32, 'name': 'pub_steering'},
        {'topic': '/autodrive/f1tenth_1/left_encoder', 'type': JointState, 'name': 'pub_left_encoder'},
        {'topic': '/autodrive/f1tenth_1/right_encoder', 'type': JointState, 'name': 'pub_right_encoder'},
        {'topic': '/autodrive/f1tenth_1/ips', 'type': Point, 'name': 'pub_ips'},
        {'topic': '/autodrive/f1tenth_1/imu', 'type': Imu, 'name': 'pub_imu'},
        {'topic': '/autodrive/f1tenth_1/lidar', 'type': LaserScan, 'name': 'pub_lidar'},
        {'topic': '/autodrive/f1tenth_1/front_camera', 'type': Image, 'name': 'pub_front_camera'},

        # Vehicle 2 data publishers
        {'topic': '/autodrive/f1tenth_2/throttle', 'type': Float32, 'name': 'pub_throttle_2'},
        {'topic': '/autodrive/f1tenth_2/steering', 'type': Float32, 'name': 'pub_steering_2'},
        {'topic': '/autodrive/f1tenth_2/left_encoder', 'type': JointState, 'name': 'pub_left_encoder_2'},
        {'topic': '/autodrive/f1tenth_2/right_encoder', 'type': JointState, 'name': 'pub_right_encoder_2'},
        {'topic': '/autodrive/f1tenth_2/ips', 'type': Point, 'name': 'pub_ips_2'},
        {'topic': '/autodrive/f1tenth_2/imu', 'type': Imu, 'name': 'pub_imu_2'},
        {'topic': '/autodrive/f1tenth_2/lidar', 'type': LaserScan, 'name': 'pub_lidar_2'},
        {'topic': '/autodrive/f1tenth_2/front_camera', 'type': Image, 'name': 'pub_front_camera_2'},
    ]
})

################################################################################

# Initialize vehicle control commands
throttle_command = 0.0 # [-1, 1]
steering_command = 0.0 # [-1, 1]
throttle_command_2 = 0.0 # [-1, 1]
steering_command_2 = 0.0 # [-1, 1]
