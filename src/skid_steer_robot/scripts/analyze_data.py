#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
import glob

def analyze_robot_data(data_dir='~/robot_data'):
    """Analyze logged robot data and create plots"""
    
    data_dir = os.path.expanduser(data_dir)
    
    # Find latest files
    odom_files = glob.glob(os.path.join(data_dir, 'odometry_*.csv'))
    imu_files = glob.glob(os.path.join(data_dir, 'imu_*.csv'))
    
    if not odom_files or not imu_files:
        print("No data files found!")
        return
    
    # Load latest files
    odom_file = max(odom_files, key=os.path.getctime)
    imu_file = max(imu_files, key=os.path.getctime)
    
    print(f"Analyzing: {odom_file}")
    print(f"Analyzing: {imu_file}")
    
    # Load data
    odom_df = pd.read_csv(odom_file)
    imu_df = pd.read_csv(imu_file)
    
    # Convert timestamps
    odom_df['time'] = odom_df['timestamp_sec'] + odom_df['timestamp_nanosec'] * 1e-9
    imu_df['time'] = imu_df['timestamp_sec'] + imu_df['timestamp_nanosec'] * 1e-9
    
    # Normalize time to start from 0
    odom_df['time'] = odom_df['time'] - odom_df['time'].iloc[0]
    imu_df['time'] = imu_df['time'] - imu_df['time'].iloc[0]
    
    # Create plots
    fig, axes = plt.subplots(3, 2, figsize=(15, 12))
    
    # Odometry position
    axes[0, 0].plot(odom_df['pos_x'], odom_df['pos_y'])
    axes[0, 0].set_xlabel('X Position (m)')
    axes[0, 0].set_ylabel('Y Position (m)')
    axes[0, 0].set_title('Robot Trajectory')
    axes[0, 0].grid(True)
    axes[0, 0].axis('equal')
    
    # Velocity over time
    axes[0, 1].plot(odom_df['time'], odom_df['linear_vel_x'], label='Linear X')
    axes[0, 1].plot(odom_df['time'], odom_df['angular_vel_z'], label='Angular Z')
    axes[0, 1].set_xlabel('Time (s)')
    axes[0, 1].set_ylabel('Velocity')
    axes[0, 1].set_title('Robot Velocity')
    axes[0, 1].legend()
    axes[0, 1].grid(True)
    
    # IMU angular velocity
    axes[1, 0].plot(imu_df['time'], imu_df['angular_vel_x'], label='X')
    axes[1, 0].plot(imu_df['time'], imu_df['angular_vel_y'], label='Y')
    axes[1, 0].plot(imu_df['time'], imu_df['angular_vel_z'], label='Z')
    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 0].set_ylabel('Angular Velocity (rad/s)')
    axes[1, 0].set_title('IMU Angular Velocity')
    axes[1, 0].legend()
    axes[1, 0].grid(True)
    
    # IMU linear acceleration
    axes[1, 1].plot(imu_df['time'], imu_df['linear_acc_x'], label='X')
    axes[1, 1].plot(imu_df['time'], imu_df['linear_acc_y'], label='Y')
    axes[1, 1].plot(imu_df['time'], imu_df['linear_acc_z'], label='Z')
    axes[1, 1].set_xlabel('Time (s)')
    axes[1, 1].set_ylabel('Linear Acceleration (m/s²)')
    axes[1, 1].set_title('IMU Linear Acceleration')
    axes[1, 1].legend()
    axes[1, 1].grid(True)
    
    # Position vs time
    axes[2, 0].plot(odom_df['time'], odom_df['pos_x'], label='X')
    axes[2, 0].plot(odom_df['time'], odom_df['pos_y'], label='Y')
    axes[2, 0].set_xlabel('Time (s)')
    axes[2, 0].set_ylabel('Position (m)')
    axes[2, 0].set_title('Position vs Time')
    axes[2, 0].legend()
    axes[2, 0].grid(True)
    
    # Calculate orientation from quaternion
    odom_df['yaw'] = np.arctan2(
        2 * (odom_df['orient_w'] * odom_df['orient_z'] + odom_df['orient_x'] * odom_df['orient_y']),
        1 - 2 * (odom_df['orient_y']**2 + odom_df['orient_z']**2)
    )
    
    axes[2, 1].plot(odom_df['time'], np.degrees(odom_df['yaw']))
    axes[2, 1].set_xlabel('Time (s)')
    axes[2, 1].set_ylabel('Yaw Angle (degrees)')
    axes[2, 1].set_title('Robot Orientation')
    axes[2, 1].grid(True)
    
    plt.tight_layout()
    
    # Save plot
    plot_file = os.path.join(data_dir, f'analysis_{pd.Timestamp.now().strftime("%Y%m%d_%H%M%S")}.png')
    plt.savefig(plot_file, dpi=300, bbox_inches='tight')
    print(f"Analysis plot saved: {plot_file}")
    
    plt.show()
    
    # Print statistics
    print("\n=== STATISTICS ===")
    print(f"Total distance traveled: {np.sum(np.sqrt(np.diff(odom_df['pos_x'])**2 + np.diff(odom_df['pos_y'])**2)):.2f} m")
    print(f"Max linear velocity: {odom_df['linear_vel_x'].max():.2f} m/s")
    print(f"Max angular velocity: {odom_df['angular_vel_z'].max():.2f} rad/s")
    print(f"Data collection time: {odom_df['time'].iloc[-1]:.2f} s")
    print(f"Number of odometry samples: {len(odom_df)}")
    print(f"Number of IMU samples: {len(imu_df)}")

if __name__ == '__main__':
    analyze_robot_data()