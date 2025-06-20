#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math

class TestAPF(Node):
    def __init__(self):
        super().__init__('test_apf')
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.test_movement)
        
        self.start_time = time.time()
        self.phase = 0
        
        self.get_logger().info('Test APF started - Testing basic movements')
    
    def test_movement(self):
        elapsed = time.time() - self.start_time
        twist = Twist()
        
        if elapsed < 5.0:
            # Wait 5 seconds
            self.get_logger().info(f'Waiting... {5.0-elapsed:.1f} seconds remaining')
            
        elif elapsed < 10.0:
            # Move forward
            twist.linear.x = 0.3
            self.get_logger().info('Phase 1: Moving forward')
            
        elif elapsed < 15.0:
            # Turn left
            twist.angular.z = 0.5
            self.get_logger().info('Phase 2: Turning left')
            
        elif elapsed < 20.0:
            # Move forward again
            twist.linear.x = 0.3
            self.get_logger().info('Phase 3: Moving forward again')
            
        else:
            # Stop
            self.get_logger().info('Test completed - Stopping')
            
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    test_node = TestAPF()
    
    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()