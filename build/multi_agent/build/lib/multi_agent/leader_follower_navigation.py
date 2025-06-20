#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import math
import time
import sys
from collections import deque

class LeaderFollowerNavigation(Node):
    def __init__(self, robot_type='leader', robot_id=0):
        super().__init__(f'{robot_type}_navigation_{robot_id}')
        
        self.robot_type = robot_type
        self.robot_id = robot_id
        
        try:
            # Publishers
            if robot_type == 'leader':
                self.cmd_vel_pub = self.create_publisher(Twist, '/leader/cmd_vel', 10)
                self.goal_status_pub = self.create_publisher(Bool, '/goal_reached', 10)
                self.path_pub = self.create_publisher(Point, '/leader_path', 10)
            else:
                self.cmd_vel_pub = self.create_publisher(Twist, '/follower/cmd_vel', 10)
                
            # Subscribers
            if robot_type == 'leader':
                self.odom_sub = self.create_subscription(
                    Odometry, '/leader/odom', self.odom_callback, 10)
                self.laser_sub = self.create_subscription(
                    LaserScan, '/leader/scan', self.laser_callback, 10)
            else:
                self.odom_sub = self.create_subscription(
                    Odometry, '/follower/odom', self.odom_callback, 10)
                self.leader_path_sub = self.create_subscription(
                    Point, '/leader_path', self.leader_path_callback, 10)
                    
            # Robot state
            self.current_pos = Point()
            self.current_pos.x = 0.0
            self.current_pos.y = 0.0
            self.current_pos.z = 0.0
            self.current_yaw = 0.0
            self.laser_data = None
            
            # Leader-specific parameters
            if robot_type == 'leader':
                # Target position (green platform coordinates)
                self.target_pos = Point()
                self.target_pos.x = 20.0
                self.target_pos.y = 20.0
                
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
                
                # Goal tracking
                self.goal_reached = False
                self.goal_reached_time = None
                
            else:  # Follower-specific parameters
                self.leader_path = deque(maxlen=50)  # Store last 50 leader positions
                self.target_point = None
                self.following_distance = 2.0  # Desired distance behind leader
                
                # Control parameters for follower
                self.max_linear_vel = 0.6   # Slightly slower than leader
                self.max_angular_vel = 1.2
                self.min_linear_vel = 0.05
                
            # Wait parameters
            self.start_time = time.time()
            self.wait_duration = 5.0  # Wait 5 seconds before starting
            self.started = False
            
            # Navigation timer
            self.timer = self.create_timer(0.1, self.navigation_callback)
            
            self.get_logger().info(f'{robot_type.capitalize()} Navigation initialized successfully')
            if robot_type == 'leader':
                self.get_logger().info(f'Target: ({self.target_pos.x}, {self.target_pos.y})')
            self.get_logger().info('Waiting 5 seconds before starting navigation...')
            
        except Exception as e:
            self.get_logger().error(f'Error in initialization: {str(e)}')
            raise

    def odom_callback(self, msg):
        """Update robot position and orientation from odometry"""
        try:
            self.current_pos = msg.pose.pose.position
            
            # Convert quaternion to yaw angle
            orientation = msg.pose.pose.orientation
            siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
            cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
            self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
            
            # Publish leader path for followers
            if self.robot_type == 'leader' and self.started:
                path_point = Point()
                path_point.x = self.current_pos.x
                path_point.y = self.current_pos.y
                path_point.z = time.time()  # Use z for timestamp
                self.path_pub.publish(path_point)
        except Exception as e:
            self.get_logger().error(f'Error in odom_callback: {str(e)}')

    def laser_callback(self, msg):
        """Update laser scan data (leader only)"""
        try:
            self.laser_data = msg
        except Exception as e:
            self.get_logger().error(f'Error in laser_callback: {str(e)}')

    def leader_path_callback(self, msg):
        """Update leader path for follower"""
        try:
            if self.robot_type == 'follower':
                self.leader_path.append({
                    'x': msg.x,
                    'y': msg.y,
                    'timestamp': msg.z
                })
        except Exception as e:
            self.get_logger().error(f'Error in leader_path_callback: {str(e)}')

    def navigation_callback(self):
        """Main navigation control loop"""
        try:
            # Wait before starting
            if not self.started:
                if time.time() - self.start_time > self.wait_duration:
                    self.started = True
                    self.get_logger().info(f'{self.robot_type.capitalize()} starting navigation!')
                else:
                    remaining = self.wait_duration - (time.time() - self.start_time)
                    if int(remaining) != getattr(self, 'last_countdown', -1):
                        self.last_countdown = int(remaining)
                        self.get_logger().info(f'Starting in {int(remaining)}...')
                return
                
            # Simple navigation for now - just move forward slowly
            twist = Twist()
            
            if self.robot_type == 'leader':
                # Check if goal is reached
                distance_to_goal = math.sqrt(
                    (self.target_pos.x - self.current_pos.x)**2 + 
                    (self.target_pos.y - self.current_pos.y)**2
                )
                
                if distance_to_goal < self.goal_threshold:
                    if not self.goal_reached:
                        self.goal_reached = True
                        self.get_logger().info('🎯 LEADER REACHED GOAL! 🎉')
                        
                        # Publish goal status
                        goal_msg = Bool()
                        goal_msg.data = True
                        self.goal_status_pub.publish(goal_msg)
                    
                    # Stop at goal
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                else:
                    # Simple movement towards goal
                    angle_to_goal = math.atan2(
                        self.target_pos.y - self.current_pos.y,
                        self.target_pos.x - self.current_pos.x
                    )
                    
                    angle_error = angle_to_goal - self.current_yaw
                    while angle_error > math.pi:
                        angle_error -= 2 * math.pi
                    while angle_error < -math.pi:
                        angle_error += 2 * math.pi
                    
                    # Simple proportional control
                    twist.linear.x = 0.3 if abs(angle_error) < math.pi/4 else 0.0
                    twist.angular.z = 0.5 * angle_error
                    
                    # Limit velocities
                    twist.angular.z = max(-self.max_angular_vel, 
                                         min(self.max_angular_vel, twist.angular.z))
                    
            else:  # Follower
                # Simple following: stay behind leader
                if len(self.leader_path) > 0:
                    leader_pos = self.leader_path[-1]
                    
                    # Calculate distance to leader
                    dist_to_leader = math.sqrt(
                        (leader_pos['x'] - self.current_pos.x)**2 + 
                        (leader_pos['y'] - self.current_pos.y)**2
                    )
                    
                    if dist_to_leader > self.following_distance + 0.5:
                        # Move towards leader
                        angle_to_leader = math.atan2(
                            leader_pos['y'] - self.current_pos.y,
                            leader_pos['x'] - self.current_pos.x
                        )
                        
                        angle_error = angle_to_leader - self.current_yaw
                        while angle_error > math.pi:
                            angle_error -= 2 * math.pi
                        while angle_error < -math.pi:
                            angle_error += 2 * math.pi
                        
                        twist.linear.x = 0.2 if abs(angle_error) < math.pi/4 else 0.0
                        twist.angular.z = 0.5 * angle_error
                        
                        # Limit velocities
                        twist.angular.z = max(-self.max_angular_vel, 
                                             min(self.max_angular_vel, twist.angular.z))
                else:
                    # No leader data, stop
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
            
            # Publish command
            self.cmd_vel_pub.publish(twist)
            
        except Exception as e:
            self.get_logger().error(f'Error in navigation_callback: {str(e)}')
            # Publish stop command on error
            twist = Twist()
            self.cmd_vel_pub.publish(twist)

def main(args=None):
    try:
        rclpy.init(args=args)
        
        # Parse command line arguments
        robot_type = 'leader'
        robot_id = 0
        
        if len(sys.argv) > 1:
            robot_type = sys.argv[1]
        if len(sys.argv) > 2:
            robot_id = int(sys.argv[2])
        
        print(f"Starting {robot_type} navigation with ID {robot_id}")
        
        nav_node = LeaderFollowerNavigation(robot_type, robot_id)
        
        try:
            rclpy.spin(nav_node)
        except KeyboardInterrupt:
            nav_node.get_logger().info(f'{robot_type.capitalize()} Navigation stopped by user')
        finally:
            # Stop robot before shutdown
            if rclpy.ok():
                twist = Twist()
                nav_node.cmd_vel_pub.publish(twist)
            nav_node.destroy_node()
            
    except Exception as e:
        print(f"Error in main: {str(e)}")
        import traceback
        traceback.print_exc()
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()