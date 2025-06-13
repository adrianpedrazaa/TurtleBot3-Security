#!/usr/bin/env python3 
import rclpy # import the rclpy library for ROS 2 Python client library
from rclpy.node import Node # import the Node class from rclpy.node module
from std_msgs.msg import String # import the String message type from std_msgs package

class MyNode(Node):
    def __init__(self):
        super().__init__("first_node")#Initialize the Node with a name
        self.counter_ = 0
        self.publisher_ = self.create_publisher(String, "chatter", 10) # This is the publisher
        self.create_timer(1.0, self.timer_callback) # Call the timer every 1 second

    def timer_callback(self):
        msg = String()
        msg.data = f"Hello, Turtlebot! {self.counter_}"
        self.publisher_.publish(msg) # Publish the message
        self.get_logger().info(f"Publishing: {msg.data}")
        self.counter_ += 1


def main(args=None): # main function to initialize the ROS 2 node
    rclpy.init(args=args) # initialize the rclpy library with command line arguments
    node = MyNode() # create an instance of MyNode
    rclpy.spin(node) # This means that this node will keep running until it is shutdown
    rclpy.shutdown() # Until we kill the node with Ctrl+C 

if __name__ == '__main__': # check if the script is being run directly
    main()
