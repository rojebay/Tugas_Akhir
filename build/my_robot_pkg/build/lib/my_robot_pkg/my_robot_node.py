#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.move_robot)
        self.get_logger().info("Controller node started!")
    
    def move_robot(self):
        msg = Twist()
        msg.linear.x = 0.5  # Maju 0.5 m/s
        msg.angular.z = 0.2 # Belok 0.2 rad/s
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()