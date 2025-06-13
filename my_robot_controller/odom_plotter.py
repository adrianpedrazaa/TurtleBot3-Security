#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import threading

class OdomPlotter(Node):
    def __init__(self):
        super().__init__('odom_plotter')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.x_data = []
        self.y_data = []

        # Start the matplotlib plot in a separate thread
        self.plot_thread = threading.Thread(target=self.plot_trajectory)
        self.plot_thread.start()

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.x_data.append(x)
        self.y_data.append(y)
        self.get_logger().info(f"Position: x={x:.2f}, y={y:.2f}")

    def plot_trajectory(self):
        plt.ion()
        fig, ax = plt.subplots()
        while rclpy.ok():
            ax.clear()
            ax.plot(self.x_data, self.y_data, 'b.-')
            ax.set_title("TurtleBot3 Trajectory")
            ax.set_xlabel("X Position (m)")
            ax.set_ylabel("Y Position (m)")
            ax.grid(True)
            plt.pause(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = OdomPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
