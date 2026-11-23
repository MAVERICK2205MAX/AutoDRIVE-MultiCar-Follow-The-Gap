#!/usr/bin/env python3

################################################################################

# Copyright (c) 2023, Tinker Twins
# All rights reserved.

################################################################################

# ROS 2 module imports
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from ament_index_python.packages import get_package_share_directory

# Python module imports
import numpy as np
import configparser
import autodrive_f1tenth.config as config

################################################################################

# Global declarations
global throttle_command_1, steering_command_1
global throttle_command_2, steering_command_2

throttle_command_1 = 0.0
steering_command_1 = 0.0
throttle_command_2 = 0.0
steering_command_2 = 0.0

#########################################################
# ROS 2 SUBSCRIBER CALLBACKS
#########################################################

def callback_throttle_command_1(msg):
    global throttle_command_1
    throttle_command_1 = float(np.round(msg.data, 3))

def callback_steering_command_1(msg):
    global steering_command_1
    steering_command_1 = float(np.round(msg.data, 3))

def callback_throttle_command_2(msg):
    global throttle_command_2
    throttle_command_2 = float(np.round(msg.data, 3))

def callback_steering_command_2(msg):
    global steering_command_2
    steering_command_2 = float(np.round(msg.data, 3))

#########################################################
# AUTODRIVE ROS 2 OUTGOING BRIDGE INFRASTRUCTURE
#########################################################

def main():
    global throttle_command_1, steering_command_1
    global throttle_command_2, steering_command_2

    rclpy.init()
    autodrive_outgoing_bridge = rclpy.create_node('autodrive_outgoing_bridge')
    qos_profile = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1
    )

    callbacks = {
        '/autodrive/f1tenth_1/throttle_command': callback_throttle_command_1,
        '/autodrive/f1tenth_1/steering_command': callback_steering_command_1,
        '/autodrive/f1tenth_2/throttle_command': callback_throttle_command_2,
        '/autodrive/f1tenth_2/steering_command': callback_steering_command_2,
    }

    subscribers = [
        autodrive_outgoing_bridge.create_subscription(e.type, e.topic, callbacks[e.topic], qos_profile)
        for e in config.pub_sub_dict.subscribers
    ]

    package_share_directory = get_package_share_directory('autodrive_f1tenth')

    while rclpy.ok():
        api_config = configparser.ConfigParser()
        api_config['f1tenth_1'] = {
            'throttle_command': str(throttle_command_1),
            'steering_command': str(steering_command_1)
        }
        api_config['f1tenth_2'] = {
            'throttle_command': str(throttle_command_2),
            'steering_command': str(steering_command_2)
        }
        try:
            with open(package_share_directory + '/api_config.ini', 'w') as configfile:
                api_config.write(configfile)
        except Exception:
            pass

        rclpy.spin_once(autodrive_outgoing_bridge, timeout_sec=0.01)

    autodrive_outgoing_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
