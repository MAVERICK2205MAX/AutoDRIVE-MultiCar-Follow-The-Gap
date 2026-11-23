#!/usr/bin/env python3
"""
AutoDRIVE Ecosystem Multi-Vehicle Simulation & Bridge Engine
Provides high-fidelity digital-twin simulation of dual F1TENTH vehicles
operating concurrently on the Levine track loop within the AutoDRIVE ecosystem.
"""

import os
import math
import yaml
import numpy as np
from PIL import Image

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float32, Header
from geometry_msgs.msg import Point, TransformStamped
from sensor_msgs.msg import LaserScan, Imu, JointState
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
from tf_transformations import quaternion_from_euler


class AutoDriveMultiCarSim(Node):
    def __init__(self):
        super().__init__('autodrive_multicar_sim')

        # Load Map
        map_yaml_path = '/home/admin1/autodrive_ws/maps/levineB.yaml'
        with open(map_yaml_path, 'r') as f:
            cfg = yaml.safe_load(f)

        self.origin = np.array(cfg['origin'][:2], dtype=float)
        self.res = float(cfg['resolution'])
        map_img_path = os.path.join(os.path.dirname(map_yaml_path), cfg['image'])
        img = Image.open(map_img_path)
        grid = np.array(img)
        if len(grid.shape) == 3:
            grid = grid[:, :, 0]
        self.H, self.W = grid.shape
        # Occupied if black lines (<100) or unknown exterior (216)
        self.occ = (grid < 100) | (grid == 216)

        # Vehicle Physical Parameters
        self.L = 0.33  # Wheelbase (m)
        self.dt = 0.025  # 40 Hz simulation step

        # Vehicle 1 State [x, y, yaw, v]
        self.v1_x = 0.0
        self.v1_y = 0.0
        self.v1_yaw = 0.0
        self.v1_steer = 0.0
        self.v1_throttle = 0.0

        # Vehicle 2 State [x, y, yaw, v] (Trailing Car 1 by 2.2 meters)
        self.v2_x = -2.2
        self.v2_y = 0.0
        self.v2_yaw = 0.0
        self.v2_steer = 0.0
        self.v2_throttle = 0.0

        # LaserScan Configuration matching AutoDRIVE
        self.num_beams = 360
        self.angle_min = -2.35619
        self.angle_max = 2.35619
        self.angles = np.linspace(self.angle_min, self.angle_max, self.num_beams)
        self.dist_steps = np.arange(0.05, 10.0, 0.05)

        # QoS Profile
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=5
        )

        # Vehicle 1 Publishers
        self.pub_lidar1 = self.create_publisher(LaserScan, '/autodrive/f1tenth_1/lidar', qos)
        self.pub_ips1 = self.create_publisher(Point, '/autodrive/f1tenth_1/ips', qos)
        self.pub_imu1 = self.create_publisher(Imu, '/autodrive/f1tenth_1/imu', qos)
        self.pub_throttle1 = self.create_publisher(Float32, '/autodrive/f1tenth_1/throttle', qos)
        self.pub_steering1 = self.create_publisher(Float32, '/autodrive/f1tenth_1/steering', qos)

        # Vehicle 2 Publishers
        self.pub_lidar2 = self.create_publisher(LaserScan, '/autodrive/f1tenth_2/lidar', qos)
        self.pub_ips2 = self.create_publisher(Point, '/autodrive/f1tenth_2/ips', qos)
        self.pub_imu2 = self.create_publisher(Imu, '/autodrive/f1tenth_2/imu', qos)
        self.pub_throttle2 = self.create_publisher(Float32, '/autodrive/f1tenth_2/throttle', qos)
        self.pub_steering2 = self.create_publisher(Float32, '/autodrive/f1tenth_2/steering', qos)

        # Vehicle 3D Visual Markers Publisher
        self.pub_markers = self.create_publisher(MarkerArray, '/autodrive/vehicle_markers', qos)

        # Subscribers
        self.create_subscription(Float32, '/autodrive/f1tenth_1/throttle_command', self.cb_throttle1, qos)
        self.create_subscription(Float32, '/autodrive/f1tenth_1/steering_command', self.cb_steering1, qos)
        self.create_subscription(Float32, '/autodrive/f1tenth_2/throttle_command', self.cb_throttle2, qos)
        self.create_subscription(Float32, '/autodrive/f1tenth_2/steering_command', self.cb_steering2, qos)

        # TF Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Main Simulation Timer (40 Hz)
        self.timer = self.create_timer(self.dt, self.simulation_loop)
        self.step_count = 0

        self.get_logger().info('AutoDRIVE Dual-Car Simulation Engine initialized on Levine Track.')
        self.get_logger().info('  Vehicle 1 (Leader):   x=0.0m, y=0.0m, yaw=0.0 rad')
        self.get_logger().info('  Vehicle 2 (Follower): x=-2.2m, y=0.0m, yaw=0.0 rad')

    def cb_throttle1(self, msg):
        self.v1_throttle = float(msg.data)

    def cb_steering1(self, msg):
        self.v1_steer = float(np.clip(msg.data, -0.34, 0.34))

    def cb_throttle2(self, msg):
        self.v2_throttle = float(msg.data)

    def cb_steering2(self, msg):
        self.v2_steer = float(np.clip(msg.data, -0.34, 0.34))

    def _raycast(self, x, y, yaw, other_x, other_y):
        r_list = np.full(self.num_beams, 10.0, dtype=np.float32)
        sin_a = np.sin(yaw + self.angles)
        cos_a = np.cos(yaw + self.angles)

        # Other vehicle relative location
        dx_other = other_x - x
        dy_other = other_y - y
        dist_other = float(np.hypot(dx_other, dy_other))
        angle_other = float(np.arctan2(dy_other, dx_other) - yaw)
        angle_other = (angle_other + np.pi) % (2.0 * np.pi) - np.pi

        for i in range(self.num_beams):
            xs = x + self.dist_steps * cos_a[i]
            ys = y + self.dist_steps * sin_a[i]
            cols = ((xs - self.origin[0]) / self.res).astype(int)
            rows = (self.H - 1 - (ys - self.origin[1]) / self.res).astype(int)
            valid = (cols >= 0) & (cols < self.W) & (rows >= 0) & (rows < self.H)
            hit_idx = np.where(~valid | self.occ[np.clip(rows, 0, self.H - 1), np.clip(cols, 0, self.W - 1)])[0]
            r_map = self.dist_steps[hit_idx[0]] if len(hit_idx) > 0 else 10.0

            # Obstacle intersection with other car
            if abs(self.angles[i] - angle_other) < 0.16 and dist_other < r_map:
                r_list[i] = dist_other
            else:
                r_list[i] = r_map

        return r_list

    def simulation_loop(self):
        now = self.get_clock().now()

        # Update Kinematics for Vehicle 1
        speed1 = self.v1_throttle * 6.0  # Normalized throttle to speed (m/s)
        self.v1_x += speed1 * math.cos(self.v1_yaw) * self.dt
        self.v1_y += speed1 * math.sin(self.v1_yaw) * self.dt
        self.v1_yaw += (speed1 / self.L) * math.tan(self.v1_steer) * self.dt

        # Update Kinematics for Vehicle 2
        speed2 = self.v2_throttle * 6.0
        self.v2_x += speed2 * math.cos(self.v2_yaw) * self.dt
        self.v2_y += speed2 * math.sin(self.v2_yaw) * self.dt
        self.v2_yaw += (speed2 / self.L) * math.tan(self.v2_steer) * self.dt

        # Raycast LiDAR scans
        r1 = self._raycast(self.v1_x, self.v1_y, self.v1_yaw, self.v2_x, self.v2_y)
        r2 = self._raycast(self.v2_x, self.v2_y, self.v2_yaw, self.v1_x, self.v1_y)

        # Publish Telemetry for Vehicle 1
        scan_msg1 = LaserScan()
        scan_msg1.header.stamp = now.to_msg()
        scan_msg1.header.frame_id = 'f1tenth_1_lidar'
        scan_msg1.angle_min = self.angle_min
        scan_msg1.angle_max = self.angle_max
        scan_msg1.angle_increment = float(self.angles[1] - self.angles[0])
        scan_msg1.time_increment = 0.0001
        scan_msg1.scan_time = self.dt
        scan_msg1.range_min = 0.05
        scan_msg1.range_max = 10.0
        scan_msg1.ranges = r1.tolist()
        self.pub_lidar1.publish(scan_msg1)

        p1 = Point()
        p1.x, p1.y, p1.z = self.v1_x, self.v1_y, 0.0
        self.pub_ips1.publish(p1)

        f_th1 = Float32()
        f_th1.data = self.v1_throttle
        self.pub_throttle1.publish(f_th1)

        f_st1 = Float32()
        f_st1.data = self.v1_steer
        self.pub_steering1.publish(f_st1)

        # Publish Telemetry for Vehicle 2
        scan_msg2 = LaserScan()
        scan_msg2.header.stamp = now.to_msg()
        scan_msg2.header.frame_id = 'f1tenth_2_lidar'
        scan_msg2.angle_min = self.angle_min
        scan_msg2.angle_max = self.angle_max
        scan_msg2.angle_increment = float(self.angles[1] - self.angles[0])
        scan_msg2.time_increment = 0.0001
        scan_msg2.scan_time = self.dt
        scan_msg2.range_min = 0.05
        scan_msg2.range_max = 10.0
        scan_msg2.ranges = r2.tolist()
        self.pub_lidar2.publish(scan_msg2)

        p2 = Point()
        p2.x, p2.y, p2.z = self.v2_x, self.v2_y, 0.0
        self.pub_ips2.publish(p2)

        f_th2 = Float32()
        f_th2.data = self.v2_throttle
        self.pub_throttle2.publish(f_th2)

        f_st2 = Float32()
        f_st2.data = self.v2_steer
        self.pub_steering2.publish(f_st2)

        # Broadcast TF transforms
        q1 = quaternion_from_euler(0, 0, self.v1_yaw)
        self._broadcast_tf('f1tenth_1', 'map', [self.v1_x, self.v1_y, 0.0], q1, now)
        self._broadcast_tf('f1tenth_1_lidar', 'f1tenth_1', [0.27, 0.0, 0.10], [0, 0, 0, 1], now)

        q2 = quaternion_from_euler(0, 0, self.v2_yaw)
        self._broadcast_tf('f1tenth_2', 'map', [self.v2_x, self.v2_y, 0.0], q2, now)
        self._broadcast_tf('f1tenth_2_lidar', 'f1tenth_2', [0.27, 0.0, 0.10], [0, 0, 0, 1], now)

        # Publish Visual Vehicle Markers in RViz
        self._publish_markers(now, q1, q2)

        self.step_count += 1
        if self.step_count % 160 == 0:
            self.get_logger().info(
                f'[Car 1] Pose: ({self.v1_x:+5.2f}, {self.v1_y:+5.2f}) | Steer: {math.degrees(self.v1_steer):+4.1f}° | Speed: {speed1:.2f} m/s'
            )
            self.get_logger().info(
                f'[Car 2] Pose: ({self.v2_x:+5.2f}, {self.v2_y:+5.2f}) | Steer: {math.degrees(self.v2_steer):+4.1f}° | Speed: {speed2:.2f} m/s | Separation: {math.hypot(self.v1_x-self.v2_x, self.v1_y-self.v2_y):.2f}m'
            )

    def _broadcast_tf(self, child, parent, translation, rotation, now):
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = float(translation[0])
        t.transform.translation.y = float(translation[1])
        t.transform.translation.z = float(translation[2])
        t.transform.rotation.x = float(rotation[0])
        t.transform.rotation.y = float(rotation[1])
        t.transform.rotation.z = float(rotation[2])
        t.transform.rotation.w = float(rotation[3])
        self.tf_broadcaster.sendTransform(t)

    def _publish_markers(self, now, q1, q2):
        ma = MarkerArray()

        # Car 1 Chassis (Cyan Box)
        m1 = Marker()
        m1.header.stamp = now.to_msg()
        m1.header.frame_id = 'map'
        m1.ns = 'chassis'
        m1.id = 1
        m1.type = Marker.CUBE
        m1.action = Marker.ADD
        m1.pose.position.x = self.v1_x
        m1.pose.position.y = self.v1_y
        m1.pose.position.z = 0.08
        m1.pose.orientation.x = float(q1[0])
        m1.pose.orientation.y = float(q1[1])
        m1.pose.orientation.z = float(q1[2])
        m1.pose.orientation.w = float(q1[3])
        m1.scale.x = 0.50
        m1.scale.y = 0.28
        m1.scale.z = 0.12
        m1.color.r, m1.color.g, m1.color.b, m1.color.a = 0.0, 0.9, 1.0, 0.85
        ma.markers.append(m1)

        # Car 1 Text Label
        t1 = Marker()
        t1.header.stamp = now.to_msg()
        t1.header.frame_id = 'map'
        t1.ns = 'labels'
        t1.id = 10
        t1.type = Marker.TEXT_VIEW_FACING
        t1.action = Marker.ADD
        t1.pose.position.x = self.v1_x
        t1.pose.position.y = self.v1_y
        t1.pose.position.z = 0.45
        t1.scale.z = 0.25
        t1.color.r, t1.color.g, t1.color.b, t1.color.a = 0.0, 1.0, 1.0, 1.0
        t1.text = "Car 1: FTG Leader"
        ma.markers.append(t1)

        # Car 2 Chassis (Orange Box)
        m2 = Marker()
        m2.header.stamp = now.to_msg()
        m2.header.frame_id = 'map'
        m2.ns = 'chassis'
        m2.id = 2
        m2.type = Marker.CUBE
        m2.action = Marker.ADD
        m2.pose.position.x = self.v2_x
        m2.pose.position.y = self.v2_y
        m2.pose.position.z = 0.08
        m2.pose.orientation.x = float(q2[0])
        m2.pose.orientation.y = float(q2[1])
        m2.pose.orientation.z = float(q2[2])
        m2.pose.orientation.w = float(q2[3])
        m2.scale.x = 0.50
        m2.scale.y = 0.28
        m2.scale.z = 0.12
        m2.color.r, m2.color.g, m2.color.b, m2.color.a = 1.0, 0.5, 0.0, 0.85
        ma.markers.append(m2)

        # Car 2 Text Label
        t2 = Marker()
        t2.header.stamp = now.to_msg()
        t2.header.frame_id = 'map'
        t2.ns = 'labels'
        t2.id = 20
        t2.type = Marker.TEXT_VIEW_FACING
        t2.action = Marker.ADD
        t2.pose.position.x = self.v2_x
        t2.pose.position.y = self.v2_y
        t2.pose.position.z = 0.45
        t2.scale.z = 0.25
        t2.color.r, t2.color.g, t2.color.b, t2.color.a = 1.0, 0.6, 0.0, 1.0
        t2.text = "Car 2: FTG Follower"
        ma.markers.append(t2)

        self.pub_markers.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = AutoDriveMultiCarSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
