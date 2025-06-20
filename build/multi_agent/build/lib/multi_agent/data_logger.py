#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
import csv
import os
from datetime import datetime

class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')
        
        # Subscribers
        self.leader_odom_sub = self.create_subscription(
            Odometry, '/leader/odom', self.leader_odom_callback, 10)
        self.follower_odom_sub = self.create_subscription(
            Odometry, '/follower/odom', self.follower_odom_callback, 10)
        self.leader_path_sub = self.create_subscription(
            Point, '/leader_path', self.leader_path_callback, 10)
        
        # Data storage
        self.leader_data = []
        self.follower_data = []
        self.path_data = []
        
        # Create log directory
        self.log_dir = os.path.expanduser('~/cobak/logs')
        os.makedirs(self.log_dir, exist_ok=True)
        
        # Initialize CSV files
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.leader_file = os.path.join(self.log_dir, f'leader_data_{timestamp}.csv')
        self.follower_file = os.path.join(self.log_dir, f'follower_data_{timestamp}.csv')
        self.path_file = os.path.join(self.log_dir, f'leader_path_{timestamp}.csv')
        
        self.init_csv_files()
        
        self.get_logger().info('Data Logger initialized')
        self.get_logger().info(f'Logging to: {self.log_dir}')

    def init_csv_files(self):
        # Leader data CSV
        with open(self.leader_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'x', 'y', 'yaw', 'linear_vel', 'angular_vel'])
        
        # Follower data CSV
        with open(self.follower_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'x', 'y', 'yaw', 'linear_vel', 'angular_vel'])
        
        # Path data CSV
        with open(self.path_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'x', 'y'])

    def leader_odom_callback(self, msg):
        import time
        timestamp = time.time()
        pos = msg.pose.pose.position
        
        # Calculate yaw from quaternion
        orientation = msg.pose.pose.orientation
        import math
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Get velocities
        linear_vel = msg.twist.twist.linear.x
        angular_vel = msg.twist.twist.angular.z
        
        # Save to CSV
        with open(self.leader_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([timestamp, pos.x, pos.y, yaw, linear_vel, angular_vel])

    def follower_odom_callback(self, msg):
        import time
        timestamp = time.time()
        pos = msg.pose.pose.position
        
        # Calculate yaw from quaternion
        orientation = msg.pose.pose.orientation
        import math
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Get velocities
        linear_vel = msg.twist.twist.linear.x
        angular_vel = msg.twist.twist.angular.z
        
        # Save to CSV
        with open(self.follower_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([timestamp, pos.x, pos.y, yaw, linear_vel, angular_vel])

    def leader_path_callback(self, msg):
        timestamp = msg.z  # z contains timestamp
        
        # Save to CSV
        with open(self.path_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([timestamp, msg.x, msg.y])

def main(args=None):
    rclpy.init(args=args)
    
    logger_node = DataLogger()
    
    try:
        rclpy.spin(logger_node)
    except KeyboardInterrupt:
        logger_node.get_logger().info('Data Logger stopped')
    finally:
        logger_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()