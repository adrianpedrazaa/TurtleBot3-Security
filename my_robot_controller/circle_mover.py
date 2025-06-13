#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CircleMover(Node):
    def __init__(self):
        super().__init__('circle_mover')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('CircleMover node has started.')

    def timer_callback(self):
        twist = Twist()
        twist.linear.x = 0.2  # Move forward
        twist.angular.z = 0.2  # Turn
        self.publisher_.publish(twist)
        self.get_logger().info(f'Moving in a circle: linear.x={twist.linear.x}, angular.z={twist.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    node = CircleMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
