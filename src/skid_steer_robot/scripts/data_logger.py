# // filepath: /home/rojebay/cobak/src/skid_steer_robot/scripts/data_logger.py
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import csv
import os
from datetime import datetime
import threading

class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')
        
        # Create data directory
        self.data_dir = os.path.expanduser('~/robot_data')
        if not os.path.exists(self.data_dir):
            os.makedirs(self.data_dir)
        
        # Create CSV files with timestamp
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.odom_file = os.path.join(self.data_dir, f'odometry_{timestamp}.csv')
        self.imu_file = os.path.join(self.data_dir, f'imu_{timestamp}.csv')
        
        # Initialize CSV files
        self.init_csv_files()
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )
        
        # Thread locks for file writing
        self.odom_lock = threading.Lock()
        self.imu_lock = threading.Lock()
        
        self.get_logger().info(f'Data logger started. Files: {self.odom_file}, {self.imu_file}')
    
    def init_csv_files(self):
        # Initialize odometry CSV
        with open(self.odom_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp_sec', 'timestamp_nanosec',
                'pos_x', 'pos_y', 'pos_z',
                'orient_x', 'orient_y', 'orient_z', 'orient_w',
                'linear_vel_x', 'linear_vel_y', 'linear_vel_z',
                'angular_vel_x', 'angular_vel_y', 'angular_vel_z',
                'pos_cov_xx', 'pos_cov_yy', 'pos_cov_zz',
                'orient_cov_xx', 'orient_cov_yy', 'orient_cov_zz'
            ])
        
        # Initialize IMU CSV
        with open(self.imu_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp_sec', 'timestamp_nanosec',
                'orient_x', 'orient_y', 'orient_z', 'orient_w',
                'angular_vel_x', 'angular_vel_y', 'angular_vel_z',
                'linear_acc_x', 'linear_acc_y', 'linear_acc_z',
                'orient_cov_xx', 'orient_cov_yy', 'orient_cov_zz',
                'angular_vel_cov_xx', 'angular_vel_cov_yy', 'angular_vel_cov_zz',
                'linear_acc_cov_xx', 'linear_acc_cov_yy', 'linear_acc_cov_zz'
            ])
    
    def odom_callback(self, msg):
        with self.odom_lock:
            try:
                with open(self.odom_file, 'a', newline='') as f:
                    writer = csv.writer(f)
                    
                    # Extract covariance diagonal elements
                    pos_cov = msg.pose.covariance
                    twist_cov = msg.twist.covariance
                    
                    writer.writerow([
                        msg.header.stamp.sec,
                        msg.header.stamp.nanosec,
                        msg.pose.pose.position.x,
                        msg.pose.pose.position.y,
                        msg.pose.pose.position.z,
                        msg.pose.pose.orientation.x,
                        msg.pose.pose.orientation.y,
                        msg.pose.pose.orientation.z,
                        msg.pose.pose.orientation.w,
                        msg.twist.twist.linear.x,
                        msg.twist.twist.linear.y,
                        msg.twist.twist.linear.z,
                        msg.twist.twist.angular.x,
                        msg.twist.twist.angular.y,
                        msg.twist.twist.angular.z,
                        pos_cov[0],   # x position variance
                        pos_cov[7],   # y position variance
                        pos_cov[14],  # z position variance
                        pos_cov[21],  # roll variance
                        pos_cov[28],  # pitch variance
                        pos_cov[35]   # yaw variance
                    ])
            except Exception as e:
                self.get_logger().error(f'Error writing odometry data: {e}')
    
    def imu_callback(self, msg):
        with self.imu_lock:
            try:
                with open(self.imu_file, 'a', newline='') as f:
                    writer = csv.writer(f)
                    
                    # Extract covariance diagonal elements
                    orient_cov = msg.orientation_covariance
                    angular_vel_cov = msg.angular_velocity_covariance
                    linear_acc_cov = msg.linear_acceleration_covariance
                    
                    writer.writerow([
                        msg.header.stamp.sec,
                        msg.header.stamp.nanosec,
                        msg.orientation.x,
                        msg.orientation.y,
                        msg.orientation.z,
                        msg.orientation.w,
                        msg.angular_velocity.x,
                        msg.angular_velocity.y,
                        msg.angular_velocity.z,
                        msg.linear_acceleration.x,
                        msg.linear_acceleration.y,
                        msg.linear_acceleration.z,
                        orient_cov[0],      # roll variance
                        orient_cov[4],      # pitch variance
                        orient_cov[8],      # yaw variance
                        angular_vel_cov[0], # angular vel x variance
                        angular_vel_cov[4], # angular vel y variance
                        angular_vel_cov[8], # angular vel z variance
                        linear_acc_cov[0],  # linear acc x variance
                        linear_acc_cov[4],  # linear acc y variance
                        linear_acc_cov[8]   # linear acc z variance
                    ])
            except Exception as e:
                self.get_logger().error(f'Error writing IMU data: {e}')

def main(args=None):
    rclpy.init(args=args)
    data_logger = DataLogger()
    
    try:
        rclpy.spin(data_logger)
    except KeyboardInterrupt:
        pass
    finally:
        data_logger.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()