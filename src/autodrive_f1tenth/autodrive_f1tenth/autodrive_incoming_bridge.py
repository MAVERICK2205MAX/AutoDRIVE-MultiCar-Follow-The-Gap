#!/usr/bin/env python3

################################################################################

# Copyright (c) 2023, Tinker Twins
# All rights reserved.

################################################################################

# ROS 2 module imports
import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, ReliabilityPolicy, HistoryPolicy
import tf2_ros
from std_msgs.msg import Int32, Float32, Header
from geometry_msgs.msg import Point, TransformStamped
from sensor_msgs.msg import JointState, Imu, LaserScan, Image
from tf_transformations import quaternion_from_euler
from ament_index_python.packages import get_package_share_directory

# Python module imports
from cv_bridge import CvBridge, CvBridgeError
from gevent import pywsgi
from geventwebsocket.handler import WebSocketHandler
import socketio
import numpy as np
import base64
from io import BytesIO
from PIL import Image as PILImage
import configparser
import autodrive_f1tenth.config as config

################################################################################

import gzip

def parse_lidar_data(raw_data):
    if not raw_data:
        return np.array([])
    try:
        decomp = gzip.decompress(base64.b64decode(raw_data)).decode('utf-8')
        return np.fromstring(decomp, dtype=float, sep='\n')
    except Exception:
        pass
    try:
        return np.fromstring(raw_data, dtype=float, sep=' ')
    except Exception:
        pass
    return np.array([])

# Global declarations
global autodrive_incoming_bridge, cv_bridge, publishers, transform_broadcaster
transform_broadcaster = None
connected_clients = []
client_map = {}

#########################################################
# ROS 2 MESSAGE GENERATING FUNCTIONS
#########################################################

def create_int_msg(val):
    i = Int32()
    i.data = int(val)
    return i

def create_float_msg(val):
    f = Float32()
    f.data = float(val)
    return f

def create_joint_state_msg(joint_angle, joint_name, frame_id):
    js = JointState()
    js.header = Header()
    js.header.stamp = autodrive_incoming_bridge.get_clock().now().to_msg()
    js.header.frame_id = frame_id
    js.name = [joint_name]
    js.position = [joint_angle]
    js.velocity = []
    js.effort = []
    return js

def create_point_msg(position):
    p = Point()
    p.x = position[0]
    p.y = position[1]
    p.z = position[2]
    return p

def create_imu_msg(orientation_quaternion, angular_velocity, linear_acceleration, frame_id):
    imu = Imu()
    imu.header = Header()
    imu.header.stamp = autodrive_incoming_bridge.get_clock().now().to_msg()
    imu.header.frame_id = frame_id
    imu.orientation.x = orientation_quaternion[0]
    imu.orientation.y = orientation_quaternion[1]
    imu.orientation.z = orientation_quaternion[2]
    imu.orientation.w = orientation_quaternion[3]
    imu.orientation_covariance = [0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025]
    imu.angular_velocity.x = angular_velocity[0]
    imu.angular_velocity.y = angular_velocity[1]
    imu.angular_velocity.z = angular_velocity[2]
    imu.angular_velocity_covariance = [0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025]
    imu.linear_acceleration.x = linear_acceleration[0]
    imu.linear_acceleration.y = linear_acceleration[1]
    imu.linear_acceleration.z = linear_acceleration[2]
    imu.linear_acceleration_covariance = [0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025]
    return imu

def create_laser_scan_msg(lidar_scan_rate, lidar_range_array, lidar_intensity_array, frame_id):
    ls = LaserScan()
    ls.header = Header()
    ls.header.stamp = autodrive_incoming_bridge.get_clock().now().to_msg()
    ls.header.frame_id = frame_id
    ls.angle_min = -2.35619
    ls.angle_max = 2.35619
    ls.angle_increment = 0.004363323
    ls.time_increment = (1 / lidar_scan_rate) / 360
    ls.scan_time = ls.time_increment * 360
    ls.range_min = 0.06
    ls.range_max = 10.0
    ls.ranges = lidar_range_array
    ls.intensities = lidar_intensity_array
    return ls

def create_image_msg(image_array, frame_id):
    img = cv_bridge.cv2_to_imgmsg(image_array, encoding="rgb8")
    img.header = Header()
    img.header.stamp = autodrive_incoming_bridge.get_clock().now().to_msg()
    img.header.frame_id = frame_id
    return img

def broadcast_transform(child_frame_id, parent_frame_id, position_tf, orientation_tf):
    global transform_broadcaster
    if transform_broadcaster is None:
        return
    tf = TransformStamped()
    tf.header.stamp = autodrive_incoming_bridge.get_clock().now().to_msg()
    tf.header.frame_id = parent_frame_id
    tf.child_frame_id = child_frame_id
    tf.transform.translation.x = float(position_tf[0])
    tf.transform.translation.y = float(position_tf[1])
    tf.transform.translation.z = float(position_tf[2])
    tf.transform.rotation.x = float(orientation_tf[0])
    tf.transform.rotation.y = float(orientation_tf[1])
    tf.transform.rotation.z = float(orientation_tf[2])
    tf.transform.rotation.w = float(orientation_tf[3])
    transform_broadcaster.sendTransform(tf)

#########################################################
# ROS 2 PUBLISHER FUNCTIONS
#########################################################

def publish_vehicle_data(car_id, data, prefix='V1'):
    suffix = '' if car_id == 'f1tenth_1' else '_2'
    frame_prefix = car_id

    # Feedback
    if f"{prefix} Throttle" in data and f"{prefix} Steering" in data:
        throttle = float(data[f"{prefix} Throttle"])
        steering = float(data[f"{prefix} Steering"])
        publishers['pub_throttle' + suffix].publish(create_float_msg(throttle))
        publishers['pub_steering' + suffix].publish(create_float_msg(steering))

    # Wheel encoders
    if f"{prefix} Encoder Angles" in data:
        encoder_angles = np.fromstring(data[f"{prefix} Encoder Angles"], dtype=float, sep=' ')
        publishers['pub_left_encoder' + suffix].publish(create_joint_state_msg(encoder_angles[0], f"{frame_prefix}_left_encoder", f"{frame_prefix}_left_encoder"))
        publishers['pub_right_encoder' + suffix].publish(create_joint_state_msg(encoder_angles[1], f"{frame_prefix}_right_encoder", f"{frame_prefix}_right_encoder"))

    # IPS Position
    if f"{prefix} Position" in data:
        position = np.fromstring(data[f"{prefix} Position"], dtype=float, sep=' ')
        publishers['pub_ips' + suffix].publish(create_point_msg(position))

    # IMU
    if f"{prefix} Orientation Quaternion" in data and f"{prefix} Angular Velocity" in data and f"{prefix} Linear Acceleration" in data:
        orientation_quaternion = np.fromstring(data[f"{prefix} Orientation Quaternion"], dtype=float, sep=' ')
        angular_velocity = np.fromstring(data[f"{prefix} Angular Velocity"], dtype=float, sep=' ')
        linear_acceleration = np.fromstring(data[f"{prefix} Linear Acceleration"], dtype=float, sep=' ')
        publishers['pub_imu' + suffix].publish(create_imu_msg(orientation_quaternion, angular_velocity, linear_acceleration, f"{frame_prefix}_imu"))

        # Transforms
        if f"{prefix} Position" in data:
            broadcast_transform(frame_prefix, "map", position, orientation_quaternion)
            broadcast_transform(f"{frame_prefix}_lidar", frame_prefix, np.asarray([0.2733, 0.0, 0.096]), np.asarray([0.0, 0.0, 0.0, 1.0]))
            broadcast_transform(f"{frame_prefix}_front_camera", frame_prefix, np.asarray([-0.015, 0.0, 0.15]), np.asarray([0, 0.0871557, 0, 0.9961947]))

    # LIDAR
    if f"{prefix} LIDAR Range Array" in data and f"{prefix} LIDAR Scan Rate" in data:
        lidar_scan_rate = float(data[f"{prefix} LIDAR Scan Rate"])
        lidar_range_array = parse_lidar_data(data[f"{prefix} LIDAR Range Array"])
        lidar_intensity_array = parse_lidar_data(data.get(f"{prefix} LIDAR Intensity Array", ""))
        publishers['pub_lidar' + suffix].publish(create_laser_scan_msg(lidar_scan_rate, lidar_range_array.tolist(), lidar_intensity_array.tolist(), f"{frame_prefix}_lidar"))

    # Camera
    if f"{prefix} Front Camera Image" in data:
        try:
            front_camera_image = np.asarray(PILImage.open(BytesIO(base64.b64decode(data[f"{prefix} Front Camera Image"]))))
            publishers['pub_front_camera' + suffix].publish(create_image_msg(front_camera_image, f"{frame_prefix}_front_camera"))
        except Exception:
            pass

cmd_cache = {
    'f1tenth_1': {'throttle': 0.0, 'steering': 0.0},
    'f1tenth_2': {'throttle': 0.0, 'steering': 0.0}
}

def cb_cmd_th1(msg):
    cmd_cache['f1tenth_1']['throttle'] = float(msg.data)

def cb_cmd_st1(msg):
    cmd_cache['f1tenth_1']['steering'] = float(msg.data)

def cb_cmd_th2(msg):
    cmd_cache['f1tenth_2']['throttle'] = float(msg.data)

def cb_cmd_st2(msg):
    cmd_cache['f1tenth_2']['steering'] = float(msg.data)

#########################################################
# WEBSOCKET SERVER INFRASTRUCTURE
#########################################################

sio = socketio.Server(async_mode='gevent')

@sio.on('connect')
def connect(sid, environ):
    global connected_clients, client_map
    if sid not in client_map:
        used_cars = set(client_map.values())
        if 'f1tenth_1' not in used_cars:
            assigned_car = 'f1tenth_1'
        elif 'f1tenth_2' not in used_cars:
            assigned_car = 'f1tenth_2'
        else:
            assigned_car = f'f1tenth_{len(used_cars) + 1}'
        if sid not in connected_clients:
            connected_clients.append(sid)
        client_map[sid] = assigned_car
        print(f"AutoDRIVE Simulator connected! Session: {sid} -> Assigned Vehicle: {assigned_car}")

@sio.on('disconnect')
def disconnect(sid):
    global connected_clients, client_map
    if sid in client_map:
        car = client_map.pop(sid)
        if sid in connected_clients:
            connected_clients.remove(sid)
        print(f"AutoDRIVE Simulator disconnected! Session: {sid} ({car})")

import traceback

packet_count = 0

@sio.on('Bridge')
def bridge(sid, data):
    global client_map, connected_clients, packet_count

    if not data:
        return

    try:
        packet_count += 1
        if packet_count <= 5 or packet_count % 10 == 0:
            print(f"[Bridge Packet #{packet_count}] Commands: {cmd_cache}", flush=True)

        # Check if this simulator instance runs 2 cars in ONE single scene
        if ("V2 Throttle" in data) or ("V2 LIDAR Range Array" in data) or ("V2 Position" in data):
            publish_vehicle_data('f1tenth_1', data, prefix='V1')
            publish_vehicle_data('f1tenth_2', data, prefix='V2')

            th1 = cmd_cache['f1tenth_1']['throttle']
            st1 = cmd_cache['f1tenth_1']['steering']
            th2 = cmd_cache['f1tenth_2']['throttle']
            st2 = cmd_cache['f1tenth_2']['steering']

            payload = {
                'V1 Throttle': str(round(th1, 3)),
                'V1 Steering': str(round(st1, 3)),
                'V2 Throttle': str(round(th2, 3)),
                'V2 Steering': str(round(st2, 3)),
                'Throttle': str(round(th1, 3)),
                'Steering': str(round(st1, 3))
            }
            sio.emit('Bridge', data=payload)
        else:
            car_id = client_map.get(sid, 'f1tenth_1')
            publish_vehicle_data(car_id, data, prefix='V1')

            throttle_cmd = cmd_cache.get(car_id, {}).get('throttle', 0.0)
            steering_cmd = cmd_cache.get(car_id, {}).get('steering', 0.0)

            payload = {
                'V1 Throttle': str(round(throttle_cmd, 3)),
                'V1 Steering': str(round(steering_cmd, 3)),
                'Throttle': str(round(throttle_cmd, 3)),
                'Steering': str(round(steering_cmd, 3))
            }
            sio.emit('Bridge', data=payload)
    except Exception as e:
        print(f"[Bridge Exception] {e}", flush=True)
        traceback.print_exc()

#########################################################
# MAIN
#########################################################

def main():
    global autodrive_incoming_bridge, cv_bridge, publishers, transform_broadcaster

    rclpy.init()
    autodrive_incoming_bridge = rclpy.create_node('autodrive_incoming_bridge')
    qos_profile = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1
    )
    cv_bridge = CvBridge()
    transform_broadcaster = tf2_ros.TransformBroadcaster(autodrive_incoming_bridge)
    publishers = {
        e.name: autodrive_incoming_bridge.create_publisher(e.type, e.topic, qos_profile)
        for e in config.pub_sub_dict.publishers
    }

    # Fast in-memory subscriptions for actuator commands
    autodrive_incoming_bridge.create_subscription(Float32, '/autodrive/f1tenth_1/throttle_command', cb_cmd_th1, qos_profile)
    autodrive_incoming_bridge.create_subscription(Float32, '/autodrive/f1tenth_1/steering_command', cb_cmd_st1, qos_profile)
    autodrive_incoming_bridge.create_subscription(Float32, '/autodrive/f1tenth_2/throttle_command', cb_cmd_th2, qos_profile)
    autodrive_incoming_bridge.create_subscription(Float32, '/autodrive/f1tenth_2/steering_command', cb_cmd_st2, qos_profile)

    import threading
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(autodrive_incoming_bridge)
    threading.Thread(target=executor.spin, daemon=True).start()

    print("AutoDRIVE Multi-Vehicle WebSocket Bridge listening on port 4567...", flush=True)
    app = socketio.WSGIApp(sio)
    pywsgi.WSGIServer(('', 4567), app, handler_class=WebSocketHandler).serve_forever()

    executor.shutdown()
    autodrive_incoming_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
