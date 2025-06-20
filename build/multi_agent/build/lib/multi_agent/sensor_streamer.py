#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool
import csv
import os
import math
import time
from datetime import datetime
import threading
import statistics

class SensorStreamer(Node):
    def __init__(self):
        super().__init__('sensor_streamer')
        
        # Goal parameters
        self.target_x = 20.0
        self.target_y = 20.0
        self.goal_threshold = 2.0
        self.goal_reached = False
        self.goal_reached_time = None
        self.shutdown_delay = 3.0  # Wait 3 seconds after goal reached before shutdown
        
        # Create data directory
        self.data_dir = os.path.expanduser('~/robot_streaming_data')
        if not os.path.exists(self.data_dir):
            os.makedirs(self.data_dir)
        
        # Create timestamped files
        self.session_id = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.create_data_files()
        
        # Subscribers
        self.setup_subscribers()
        
        # Publishers
        self.goal_status_pub = self.create_publisher(Bool, '/goal_reached', 10)
        
        # Data storage for real-time analysis
        self.latest_odom = None
        self.latest_imu = None
        self.latest_lidar = None
        
        # Statistics tracking
        self.start_time = time.time()
        self.odom_count = 0
        self.imu_count = 0
        self.lidar_count = 0
        
        # Thread locks
        self.odom_lock = threading.Lock()
        self.imu_lock = threading.Lock()
        self.lidar_lock = threading.Lock()
        
        # Real-time display timer
        self.display_timer = self.create_timer(1.0, self.display_real_time_data)
        
        # Statistics timer
        self.stats_timer = self.create_timer(5.0, self.update_statistics)
        
        # Goal check timer
        self.goal_timer = self.create_timer(0.5, self.check_goal_status)
        
        # Shutdown timer (will be activated when goal is reached)
        self.shutdown_timer = None
        
        self.get_logger().info('🚀 Sensor Streamer and Logger Started!')
        self.get_logger().info(f'📂 Data directory: {self.data_dir}')
        self.get_logger().info(f'🔖 Session ID: {self.session_id}')
        self.get_logger().info(f'🎯 Target: ({self.target_x}, {self.target_y}) with threshold {self.goal_threshold}m')
        self.get_logger().info('📊 Real-time data streaming and CSV logging active...')
        self.get_logger().info('⏹️  Will automatically stop when goal is reached!')
    
    def create_data_files(self):
        """Create CSV files for each sensor type"""
        # Odometry CSV
        self.odom_file = os.path.join(self.data_dir, f'odom_stream_{self.session_id}.csv')
        with open(self.odom_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp', 'time_elapsed', 'sequence',
                'pos_x', 'pos_y', 'pos_z', 
                'orient_x', 'orient_y', 'orient_z', 'orient_w',
                'roll_deg', 'pitch_deg', 'yaw_deg',
                'linear_vel_x', 'linear_vel_y', 'linear_vel_z',
                'angular_vel_x', 'angular_vel_y', 'angular_vel_z',
                'linear_speed', 'angular_speed', 'distance_to_goal'
            ])
        
        # IMU CSV
        self.imu_file = os.path.join(self.data_dir, f'imu_stream_{self.session_id}.csv')
        with open(self.imu_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp', 'time_elapsed', 'sequence',
                'orient_x', 'orient_y', 'orient_z', 'orient_w',
                'roll_deg', 'pitch_deg', 'yaw_deg',
                'angular_vel_x', 'angular_vel_y', 'angular_vel_z',
                'linear_acc_x', 'linear_acc_y', 'linear_acc_z',
                'acc_magnitude', 'angular_vel_magnitude'
            ])
        
        # LiDAR CSV
        self.lidar_file = os.path.join(self.data_dir, f'lidar_stream_{self.session_id}.csv')
        with open(self.lidar_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp', 'time_elapsed', 'sequence',
                'angle_min', 'angle_max', 'angle_increment',
                'range_min', 'range_max', 'num_ranges',
                'min_range', 'max_range', 'mean_range', 'median_range',
                'ranges_under_1m', 'ranges_under_2m', 'closest_obstacle_angle',
                'front_distance', 'left_distance', 'right_distance', 'back_distance'
            ])
        
        # Summary stats file
        self.stats_file = os.path.join(self.data_dir, f'streaming_stats_{self.session_id}.txt')
        
        # Mission summary file
        self.mission_file = os.path.join(self.data_dir, f'mission_summary_{self.session_id}.txt')
        
        self.get_logger().info(f'📄 Created data files:')
        self.get_logger().info(f'   Odometry: {os.path.basename(self.odom_file)}')
        self.get_logger().info(f'   IMU: {os.path.basename(self.imu_file)}')
        self.get_logger().info(f'   LiDAR: {os.path.basename(self.lidar_file)}')
    
    def setup_subscribers(self):
        """Setup subscribers for all sensor topics"""
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self.imu_callback, 10)
        
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)
    
    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to euler angles in degrees"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)
    
    def calculate_distance_to_goal(self, x, y):
        """Calculate distance to target goal"""
        return math.sqrt((self.target_x - x)**2 + (self.target_y - y)**2)
    
    def check_goal_status(self):
        """Check if robot has reached the goal"""
        if self.goal_reached:
            return
            
        if self.latest_odom is not None:
            pos = self.latest_odom.pose.pose.position
            distance = self.calculate_distance_to_goal(pos.x, pos.y)
            
            if distance <= self.goal_threshold:
                self.goal_reached = True
                self.goal_reached_time = time.time()
                
                self.get_logger().info('🎯 GOAL REACHED! 🎉')
                self.get_logger().info(f'   Final position: ({pos.x:.2f}, {pos.y:.2f})')
                self.get_logger().info(f'   Distance to target: {distance:.2f}m')
                self.get_logger().info(f'⏱️  Stopping data collection in {self.shutdown_delay} seconds...')
                
                # Publish goal status
                goal_msg = Bool()
                goal_msg.data = True
                self.goal_status_pub.publish(goal_msg)
                
                # Write mission completion info
                self.write_mission_summary()
                
                # Start shutdown timer
                self.shutdown_timer = self.create_timer(
                    self.shutdown_delay, 
                    self.shutdown_data_collection
                )
    
    def write_mission_summary(self):
        """Write mission completion summary"""
        try:
            elapsed = time.time() - self.start_time
            pos = self.latest_odom.pose.pose.position
            distance = self.calculate_distance_to_goal(pos.x, pos.y)
            
            with open(self.mission_file, 'w') as f:
                f.write("=== MISSION COMPLETION SUMMARY ===\n")
                f.write(f"Session ID: {self.session_id}\n")
                f.write(f"Start time: {datetime.fromtimestamp(self.start_time).strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"Goal reached time: {datetime.fromtimestamp(self.goal_reached_time).strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"Mission duration: {elapsed:.1f} seconds\n")
                f.write(f"\n=== TARGET INFO ===\n")
                f.write(f"Target position: ({self.target_x}, {self.target_y})\n")
                f.write(f"Goal threshold: {self.goal_threshold} m\n")
                f.write(f"Final position: ({pos.x:.3f}, {pos.y:.3f})\n")
                f.write(f"Final distance to target: {distance:.3f} m\n")
                f.write(f"Goal achieved: {'YES' if distance <= self.goal_threshold else 'NO'}\n")
                f.write(f"\n=== DATA COLLECTION STATS ===\n")
                f.write(f"Odometry samples: {self.odom_count}\n")
                f.write(f"IMU samples: {self.imu_count}\n")
                f.write(f"LiDAR samples: {self.lidar_count}\n")
                f.write(f"Total samples: {self.odom_count + self.imu_count + self.lidar_count}\n")
                
                if elapsed > 0:
                    f.write(f"\n=== AVERAGE SAMPLING RATES ===\n")
                    f.write(f"Odometry: {self.odom_count/elapsed:.1f} Hz\n")
                    f.write(f"IMU: {self.imu_count/elapsed:.1f} Hz\n")
                    f.write(f"LiDAR: {self.lidar_count/elapsed:.1f} Hz\n")
                
                f.write(f"\n=== DATA FILES GENERATED ===\n")
                f.write(f"Odometry: {os.path.basename(self.odom_file)}\n")
                f.write(f"IMU: {os.path.basename(self.imu_file)}\n")
                f.write(f"LiDAR: {os.path.basename(self.lidar_file)}\n")
                f.write(f"Statistics: {os.path.basename(self.stats_file)}\n")
                f.write(f"Mission summary: {os.path.basename(self.mission_file)}\n")
                
            self.get_logger().info(f'📝 Mission summary saved: {os.path.basename(self.mission_file)}')
                
        except Exception as e:
            self.get_logger().error(f'Error writing mission summary: {e}')
    
    def shutdown_data_collection(self):
        """Shutdown data collection and node"""
        self.get_logger().info('⏹️  Data collection completed!')
        self.get_logger().info('📁 Final data files saved in: {}'.format(self.data_dir))
        
        # Cancel all timers
        if self.display_timer:
            self.display_timer.cancel()
        if self.stats_timer:
            self.stats_timer.cancel()
        if self.goal_timer:
            self.goal_timer.cancel()
        if self.shutdown_timer:
            self.shutdown_timer.cancel()
        
        # Final statistics update
        self.update_statistics(final=True)
        
        # Shutdown node
        rclpy.shutdown()
    
    def odom_callback(self, msg):
        """Process and log odometry data"""
        if self.goal_reached:
            return
            
        with self.odom_lock:
            try:
                current_time = time.time()
                elapsed = current_time - self.start_time
                
                # Store latest data
                self.latest_odom = msg
                self.odom_count += 1
                
                # Calculate euler angles
                roll, pitch, yaw = self.quaternion_to_euler(
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w
                )
                
                # Calculate speeds
                linear_speed = math.sqrt(
                    msg.twist.twist.linear.x**2 + 
                    msg.twist.twist.linear.y**2 + 
                    msg.twist.twist.linear.z**2
                )
                angular_speed = math.sqrt(
                    msg.twist.twist.angular.x**2 + 
                    msg.twist.twist.angular.y**2 + 
                    msg.twist.twist.angular.z**2
                )
                
                # Calculate distance to goal
                distance_to_goal = self.calculate_distance_to_goal(
                    msg.pose.pose.position.x, 
                    msg.pose.pose.position.y
                )
                
                # Write to CSV
                with open(self.odom_file, 'a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([
                        current_time, elapsed, self.odom_count,
                        round(msg.pose.pose.position.x, 4),
                        round(msg.pose.pose.position.y, 4),
                        round(msg.pose.pose.position.z, 4),
                        round(msg.pose.pose.orientation.x, 4),
                        round(msg.pose.pose.orientation.y, 4),
                        round(msg.pose.pose.orientation.z, 4),
                        round(msg.pose.pose.orientation.w, 4),
                        round(roll, 2), round(pitch, 2), round(yaw, 2),
                        round(msg.twist.twist.linear.x, 4),
                        round(msg.twist.twist.linear.y, 4),
                        round(msg.twist.twist.linear.z, 4),
                        round(msg.twist.twist.angular.x, 4),
                        round(msg.twist.twist.angular.y, 4),
                        round(msg.twist.twist.angular.z, 4),
                        round(linear_speed, 4),
                        round(angular_speed, 4),
                        round(distance_to_goal, 4)
                    ])
                
            except Exception as e:
                self.get_logger().error(f'Error processing odometry: {e}')
    
    def imu_callback(self, msg):
        """Process and log IMU data"""
        if self.goal_reached:
            return
            
        with self.imu_lock:
            try:
                current_time = time.time()
                elapsed = current_time - self.start_time
                
                # Store latest data
                self.latest_imu = msg
                self.imu_count += 1
                
                # Calculate euler angles
                roll, pitch, yaw = self.quaternion_to_euler(
                    msg.orientation.x,
                    msg.orientation.y,
                    msg.orientation.z,
                    msg.orientation.w
                )
                
                # Calculate magnitudes
                acc_magnitude = math.sqrt(
                    msg.linear_acceleration.x**2 + 
                    msg.linear_acceleration.y**2 + 
                    msg.linear_acceleration.z**2
                )
                
                angular_vel_magnitude = math.sqrt(
                    msg.angular_velocity.x**2 + 
                    msg.angular_velocity.y**2 + 
                    msg.angular_velocity.z**2
                )
                
                # Write to CSV
                with open(self.imu_file, 'a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([
                        current_time, elapsed, self.imu_count,
                        round(msg.orientation.x, 4),
                        round(msg.orientation.y, 4),
                        round(msg.orientation.z, 4),
                        round(msg.orientation.w, 4),
                        round(roll, 2), round(pitch, 2), round(yaw, 2),
                        round(msg.angular_velocity.x, 4),
                        round(msg.angular_velocity.y, 4),
                        round(msg.angular_velocity.z, 4),
                        round(msg.linear_acceleration.x, 4),
                        round(msg.linear_acceleration.y, 4),
                        round(msg.linear_acceleration.z, 4),
                        round(acc_magnitude, 4),
                        round(angular_vel_magnitude, 4)
                    ])
                
            except Exception as e:
                self.get_logger().error(f'Error processing IMU: {e}')
    
    def lidar_callback(self, msg):
        """Process and log LiDAR data"""
        if self.goal_reached:
            return
            
        with self.lidar_lock:
            try:
                current_time = time.time()
                elapsed = current_time - self.start_time
                
                # Store latest data
                self.latest_lidar = msg
                self.lidar_count += 1
                
                # Process ranges data
                valid_ranges = [r for r in msg.ranges if not (math.isinf(r) or math.isnan(r))]
                
                if valid_ranges:
                    min_range = min(valid_ranges)
                    max_range = max(valid_ranges)
                    mean_range = statistics.mean(valid_ranges)
                    median_range = statistics.median(valid_ranges)
                    
                    # Count close obstacles
                    ranges_under_1m = sum(1 for r in valid_ranges if r < 1.0)
                    ranges_under_2m = sum(1 for r in valid_ranges if r < 2.0)
                    
                    # Find closest obstacle angle
                    min_idx = msg.ranges.index(min_range)
                    closest_obstacle_angle = msg.angle_min + min_idx * msg.angle_increment
                    closest_obstacle_angle_deg = math.degrees(closest_obstacle_angle)
                    
                    # Sector analysis (front, left, right, back)
                    num_ranges = len(msg.ranges)
                    sector_size = num_ranges // 4
                    
                    front_ranges = msg.ranges[num_ranges//2 - sector_size//4 : num_ranges//2 + sector_size//4]
                    left_ranges = msg.ranges[sector_size//2 : sector_size + sector_size//2]
                    right_ranges = msg.ranges[-sector_size : -sector_size//2]
                    back_ranges = msg.ranges[:sector_size//4] + msg.ranges[-sector_size//4:]
                    
                    def get_sector_distance(ranges):
                        valid = [r for r in ranges if not (math.isinf(r) or math.isnan(r))]
                        return min(valid) if valid else float('inf')
                    
                    front_dist = get_sector_distance(front_ranges)
                    left_dist = get_sector_distance(left_ranges)
                    right_dist = get_sector_distance(right_ranges)
                    back_dist = get_sector_distance(back_ranges)
                    
                else:
                    min_range = max_range = mean_range = median_range = 0.0
                    ranges_under_1m = ranges_under_2m = 0
                    closest_obstacle_angle_deg = 0.0
                    front_dist = left_dist = right_dist = back_dist = float('inf')
                
                # Write to CSV
                with open(self.lidar_file, 'a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([
                        current_time, elapsed, self.lidar_count,
                        round(msg.angle_min, 4), round(msg.angle_max, 4), round(msg.angle_increment, 6),
                        round(msg.range_min, 2), round(msg.range_max, 2), len(msg.ranges),
                        round(min_range, 4), round(max_range, 4), round(mean_range, 4), round(median_range, 4),
                        ranges_under_1m, ranges_under_2m, round(closest_obstacle_angle_deg, 2),
                        round(front_dist, 4), round(left_dist, 4), round(right_dist, 4), round(back_dist, 4)
                    ])
                
            except Exception as e:
                self.get_logger().error(f'Error processing LiDAR: {e}')
    
    def display_real_time_data(self):
        """Display real-time sensor data"""
        if self.goal_reached:
            return
            
        current_time = time.time()
        elapsed = current_time - self.start_time
        
        print(f"\n{'='*60}")
        print(f"🤖 REAL-TIME SENSOR DATA STREAM - {elapsed:.1f}s")
        print(f"{'='*60}")
        
        # Odometry data
        if self.latest_odom:
            pos = self.latest_odom.pose.pose.position
            vel = self.latest_odom.twist.twist
            _, _, yaw = self.quaternion_to_euler(
                self.latest_odom.pose.pose.orientation.x,
                self.latest_odom.pose.pose.orientation.y,
                self.latest_odom.pose.pose.orientation.z,
                self.latest_odom.pose.pose.orientation.w
            )
            
            distance_to_goal = self.calculate_distance_to_goal(pos.x, pos.y)
            
            print(f"📍 ODOMETRY (samples: {self.odom_count})")
            print(f"   Position: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}) m")
            print(f"   🎯 Distance to goal: {distance_to_goal:.2f} m")
            print(f"   Orientation: {yaw:.1f}°")
            print(f"   Velocity: {vel.linear.x:.2f} m/s, {vel.angular.z:.2f} rad/s")
        else:
            print(f"📍 ODOMETRY: ❌ No data")
        
        # IMU data
        if self.latest_imu:
            acc = self.latest_imu.linear_acceleration
            ang_vel = self.latest_imu.angular_velocity
            acc_mag = math.sqrt(acc.x**2 + acc.y**2 + acc.z**2)
            
            print(f"🧭 IMU (samples: {self.imu_count})")
            print(f"   Acceleration: {acc_mag:.2f} m/s² ({acc.x:.2f}, {acc.y:.2f}, {acc.z:.2f})")
            print(f"   Angular velocity: ({ang_vel.x:.2f}, {ang_vel.y:.2f}, {ang_vel.z:.2f}) rad/s")
        else:
            print(f"🧭 IMU: ❌ No data")
        
        # LiDAR data
        if self.latest_lidar:
            valid_ranges = [r for r in self.latest_lidar.ranges if not (math.isinf(r) or math.isnan(r))]
            if valid_ranges:
                min_dist = min(valid_ranges)
                mean_dist = statistics.mean(valid_ranges)
                close_obstacles = sum(1 for r in valid_ranges if r < 1.0)
                
                print(f"📡 LIDAR (samples: {self.lidar_count})")
                print(f"   Range: {len(self.latest_lidar.ranges)} points")
                print(f"   Closest obstacle: {min_dist:.2f} m")
                print(f"   Average distance: {mean_dist:.2f} m")
                print(f"   Obstacles <1m: {close_obstacles}")
            else:
                print(f"📡 LIDAR: No valid ranges")
        else:
            print(f"📡 LIDAR: ❌ No data")
        
        # Data rates
        if elapsed > 0:
            odom_rate = self.odom_count / elapsed
            imu_rate = self.imu_count / elapsed
            lidar_rate = self.lidar_count / elapsed
            
            print(f"📊 DATA RATES:")
            print(f"   Odometry: {odom_rate:.1f} Hz")
            print(f"   IMU: {imu_rate:.1f} Hz")
            print(f"   LiDAR: {lidar_rate:.1f} Hz")
    
    def update_statistics(self, final=False):
        """Update statistics file"""
        try:
            current_time = time.time()
            elapsed = current_time - self.start_time
            
            status = "FINAL" if final else "RUNNING"
            
            with open(self.stats_file, 'w') as f:
                f.write(f"=== STREAMING SESSION STATISTICS ({status}) ===\n")
                f.write(f"Session ID: {self.session_id}\n")
                f.write(f"Start time: {datetime.fromtimestamp(self.start_time).strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"Elapsed time: {elapsed:.1f} seconds\n")
                f.write(f"Current time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"Goal reached: {'YES' if self.goal_reached else 'NO'}\n")
                
                if self.goal_reached and self.goal_reached_time:
                    mission_duration = self.goal_reached_time - self.start_time
                    f.write(f"Mission completion time: {mission_duration:.1f} seconds\n")
                
                f.write(f"\n=== GOAL STATUS ===\n")
                f.write(f"Target position: ({self.target_x}, {self.target_y})\n")
                f.write(f"Goal threshold: {self.goal_threshold} m\n")
                
                if self.latest_odom:
                    pos = self.latest_odom.pose.pose.position
                    distance = self.calculate_distance_to_goal(pos.x, pos.y)
                    f.write(f"Current position: ({pos.x:.3f}, {pos.y:.3f})\n")
                    f.write(f"Current distance to goal: {distance:.3f} m\n")
                
                f.write(f"\n=== DATA COUNTS ===\n")
                f.write(f"Odometry samples: {self.odom_count}\n")
                f.write(f"IMU samples: {self.imu_count}\n")
                f.write(f"LiDAR samples: {self.lidar_count}\n")
                
                if elapsed > 0:
                    f.write(f"\n=== SAMPLING RATES ===\n")
                    f.write(f"Odometry: {self.odom_count/elapsed:.1f} Hz\n")
                    f.write(f"IMU: {self.imu_count/elapsed:.1f} Hz\n")
                    f.write(f"LiDAR: {self.lidar_count/elapsed:.1f} Hz\n")
                
                f.write(f"\n=== DATA FILES ===\n")
                f.write(f"Odometry: {os.path.basename(self.odom_file)}\n")
                f.write(f"IMU: {os.path.basename(self.imu_file)}\n")
                f.write(f"LiDAR: {os.path.basename(self.lidar_file)}\n")
                f.write(f"Statistics: {os.path.basename(self.stats_file)}\n")
                if self.goal_reached:
                    f.write(f"Mission summary: {os.path.basename(self.mission_file)}\n")
        
        except Exception as e:
            self.get_logger().error(f'Error updating statistics: {e}')

def main(args=None):
    rclpy.init(args=args)
    streamer = SensorStreamer()
    
    try:
        print("\n🚀 Starting Real-time Sensor Streaming and Logging...")
        print("📊 Data will be displayed every second")
        print("💾 CSV files are being written continuously")
        print("🎯 Will automatically stop when robot reaches target area")
        print("⏹️  Press Ctrl+C to stop manually\n")
        
        rclpy.spin(streamer)
        
    except KeyboardInterrupt:
        print("\n⏹️  Stopping sensor streaming...")
        
        # Final statistics
        elapsed = time.time() - streamer.start_time
        print(f"\n📊 SESSION SUMMARY:")
        print(f"   Duration: {elapsed:.1f} seconds")
        print(f"   Goal reached: {'YES' if streamer.goal_reached else 'NO'}")
        print(f"   Odometry samples: {streamer.odom_count}")
        print(f"   IMU samples: {streamer.imu_count}")
        print(f"   LiDAR samples: {streamer.lidar_count}")
        print(f"   Data saved in: {streamer.data_dir}")
        
        # Update final statistics
        streamer.update_statistics(final=True)
        
    finally:
        streamer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()