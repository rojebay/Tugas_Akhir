#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import math
import time

class APFNavigation(Node):
    def __init__(self):
        super().__init__('apf_navigation')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_status_pub = self.create_publisher(Bool, '/goal_reached', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10)
        
        # Robot state
        self.current_pos = Point()
        self.current_yaw = 0.0
        self.laser_data = None
        
        # Target position (green platform coordinates)
        self.target_pos = Point()
        self.target_pos.x = 20.0  # Target area X coordinate
        self.target_pos.y = 20.0  # Target area Y coordinate
        
        # APF Parameters
        self.k_att = 5.0        # Attractive force gain
        self.k_rep = 50.0       # Repulsive force gain
        self.d0 = 2.5           # Obstacle influence distance
        self.goal_threshold = 2.0  # Distance to consider goal reached
        
        # Control parameters
        self.max_linear_vel = 0.8
        self.max_angular_vel = 1.0
        self.min_linear_vel = 0.1
        
        # Safety parameters
        self.min_obstacle_dist = 0.5
        self.emergency_stop_dist = 0.3
        
        # Wait parameters
        self.start_time = time.time()
        self.wait_duration = 5.0  # Wait 5 seconds before starting
        self.started = False
        
        # Goal tracking
        self.goal_reached = False
        self.goal_reached_time = None
        self.celebration_duration = 5.0  # Celebrate for 5 seconds after goal
        
        # Navigation timer
        self.timer = self.create_timer(0.1, self.navigation_callback)
        
        self.get_logger().info('APF Navigation initialized')
        self.get_logger().info(f'Target: ({self.target_pos.x}, {self.target_pos.y})')
        self.get_logger().info('Waiting 5 seconds before starting navigation...')
        self.get_logger().info('Will stop automatically when goal is reached!')
        
    def odom_callback(self, msg):
        """Update robot position and orientation from odometry"""
        self.current_pos = msg.pose.pose.position
        
        # Convert quaternion to yaw angle
        orientation = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
    
    def laser_callback(self, msg):
        """Update laser scan data"""
        self.laser_data = msg
    
    def calculate_attractive_force(self):
        """Calculate attractive force towards target"""
        if self.current_pos is None:
            return 0.0, 0.0
        
        # Distance to target
        dx = self.target_pos.x - self.current_pos.x
        dy = self.target_pos.y - self.current_pos.y
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < self.goal_threshold:
            return 0.0, 0.0  # Goal reached
        
        # Attractive force (proportional to distance)
        f_att_x = self.k_att * dx
        f_att_y = self.k_att * dy
        
        return f_att_x, f_att_y
    
    def calculate_repulsive_force(self):
        """Calculate repulsive force from obstacles using laser data"""
        if self.laser_data is None:
            return 0.0, 0.0
        
        f_rep_x = 0.0
        f_rep_y = 0.0
        
        ranges = self.laser_data.ranges
        angle_min = self.laser_data.angle_min
        angle_increment = self.laser_data.angle_increment
        
        for i, r in enumerate(ranges):
            if math.isinf(r) or math.isnan(r) or r > self.d0:
                continue
            
            # Calculate obstacle angle relative to robot
            obstacle_angle = angle_min + i * angle_increment
            
            # Convert to global coordinates
            global_angle = self.current_yaw + obstacle_angle
            
            # Obstacle position relative to robot
            obs_x = r * math.cos(global_angle)
            obs_y = r * math.sin(global_angle)
            
            # Distance to obstacle
            distance = r
            
            if distance > 0 and distance < self.d0:
                # Repulsive force magnitude (inversely proportional to distance)
                force_magnitude = self.k_rep * (1.0/distance - 1.0/self.d0) * (1.0/distance**2)
                
                # Direction away from obstacle
                force_direction_x = -obs_x / distance
                force_direction_y = -obs_y / distance
                
                # Add to total repulsive force
                f_rep_x += force_magnitude * force_direction_x
                f_rep_y += force_magnitude * force_direction_y
        
        return f_rep_x, f_rep_y
    
    def check_emergency_stop(self):
        """Check if emergency stop is needed due to close obstacles"""
        if self.laser_data is None:
            return False
        
        # Check front sector for emergency stop
        ranges = self.laser_data.ranges
        front_sector = len(ranges) // 6  # 60 degrees front sector
        start_idx = len(ranges) // 2 - front_sector // 2
        end_idx = len(ranges) // 2 + front_sector // 2
        
        for i in range(start_idx, end_idx):
            if i < len(ranges) and ranges[i] < self.emergency_stop_dist:
                return True
        
        return False
    
    def calculate_control_commands(self, f_total_x, f_total_y):
        """Convert total force to control commands"""
        # Calculate desired angle
        desired_angle = math.atan2(f_total_y, f_total_x)
        
        # Angular error
        angle_error = desired_angle - self.current_yaw
        
        # Normalize angle error to [-pi, pi]
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi
        
        # Force magnitude
        force_magnitude = math.sqrt(f_total_x**2 + f_total_y**2)
        
        # Control commands
        angular_vel = max(-self.max_angular_vel, 
                         min(self.max_angular_vel, 2.0 * angle_error))
        
        # Reduce linear velocity when turning
        linear_vel = min(self.max_linear_vel, 
                        max(self.min_linear_vel, 
                            force_magnitude * 0.1 * (1.0 - abs(angle_error)/math.pi)))
        
        # Stop if angular error is too large
        if abs(angle_error) > math.pi/3:
            linear_vel = 0.0
        
        return linear_vel, angular_vel
    
    def check_goal_reached(self):
        """Check if robot has reached the target"""
        if self.current_pos is None:
            return False
        
        distance = math.sqrt(
            (self.target_pos.x - self.current_pos.x)**2 + 
            (self.target_pos.y - self.current_pos.y)**2
        )
        
        return distance < self.goal_threshold
    
    def publish_goal_status(self, reached):
        """Publish goal status to other nodes"""
        goal_msg = Bool()
        goal_msg.data = reached
        self.goal_status_pub.publish(goal_msg)
    
    def navigation_callback(self):
        """Main navigation control loop"""
        # Wait 5 seconds before starting
        if not self.started:
            elapsed_time = time.time() - self.start_time
            if elapsed_time < self.wait_duration:
                # Publish zero velocity (stay still)
                twist = Twist()
                self.cmd_vel_pub.publish(twist)
                
                # Log countdown every second
                remaining = self.wait_duration - elapsed_time
                if int(remaining) != getattr(self, 'last_countdown', -1):
                    self.last_countdown = int(remaining)
                    self.get_logger().info(f'Starting navigation in {int(remaining)+1} seconds...')
                return
            else:
                self.started = True
                self.get_logger().info('🚀 Starting APF navigation!')
        
        # Check if goal is reached
        if self.check_goal_reached() and not self.goal_reached:
            self.goal_reached = True
            self.goal_reached_time = time.time()
            
            # Stop robot immediately
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            
            # Publish goal status
            self.publish_goal_status(True)
            
            distance = math.sqrt(
                (self.target_pos.x - self.current_pos.x)**2 + 
                (self.target_pos.y - self.current_pos.y)**2
            )
            
            self.get_logger().info('🎯 GOAL REACHED! 🎉')
            self.get_logger().info(f'   Final position: ({self.current_pos.x:.2f}, {self.current_pos.y:.2f})')
            self.get_logger().info(f'   Distance to target: {distance:.2f}m')
            self.get_logger().info(f'   Mission duration: {time.time() - self.start_time:.1f}s')
            self.get_logger().info('🎉 Celebrating for 5 seconds before shutdown...')
        
        # Handle goal reached state
        if self.goal_reached:
            elapsed_since_goal = time.time() - self.goal_reached_time
            
            if elapsed_since_goal < self.celebration_duration:
                # Stay stopped during celebration
                twist = Twist()
                self.cmd_vel_pub.publish(twist)
                
                # Log celebration countdown
                remaining = self.celebration_duration - elapsed_since_goal
                if int(remaining) != getattr(self, 'last_celebration_countdown', -1):
                    self.last_celebration_countdown = int(remaining)
                    self.get_logger().info(f'🎉 Mission completed! Shutting down in {int(remaining)+1} seconds...')
                
                return
            else:
                # Celebration over, shutdown
                self.get_logger().info('✅ APF Navigation completed successfully!')
                self.get_logger().info('⏹️  Shutting down navigation node...')
                
                # Final stop command
                twist = Twist()
                self.cmd_vel_pub.publish(twist)
                
                # Shutdown the node
                rclpy.shutdown()
                return
        
        # Emergency stop check
        if self.check_emergency_stop():
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            self.get_logger().warn('⚠️ Emergency stop: Obstacle too close!')
            return
        
        # Calculate forces
        f_att_x, f_att_y = self.calculate_attractive_force()
        f_rep_x, f_rep_y = self.calculate_repulsive_force()
        
        # Total force
        f_total_x = f_att_x + f_rep_x
        f_total_y = f_att_y + f_rep_y
        
        # Calculate control commands
        linear_vel, angular_vel = self.calculate_control_commands(f_total_x, f_total_y)
        
        # Create and publish twist message
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        
        self.cmd_vel_pub.publish(twist)
        
        # Log status every 2 seconds
        current_time = time.time()
        if not hasattr(self, 'last_log_time'):
            self.last_log_time = current_time
        
        if current_time - self.last_log_time > 2.0:
            distance_to_goal = math.sqrt(
                (self.target_pos.x - self.current_pos.x)**2 + 
                (self.target_pos.y - self.current_pos.y)**2
            )
            
            self.get_logger().info(
                f'Pos: ({self.current_pos.x:.1f}, {self.current_pos.y:.1f}), '
                f'Goal dist: {distance_to_goal:.1f}m, '
                f'Vel: {linear_vel:.2f}m/s, {angular_vel:.2f}rad/s'
            )
            self.last_log_time = current_time

def main(args=None):
    rclpy.init(args=args)
    apf_nav = APFNavigation()
    
    try:
        rclpy.spin(apf_nav)
    except KeyboardInterrupt:
        apf_nav.get_logger().info('APF Navigation stopped by user')
    finally:
        # Stop robot before shutdown
        if rclpy.ok():
            twist = Twist()
            apf_nav.cmd_vel_pub.publish(twist)
        apf_nav.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()