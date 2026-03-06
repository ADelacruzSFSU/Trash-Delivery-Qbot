"""
Name: Anthony Delacruz
Filename: num_talker.py
Description:
    ROS 2 node that generates random integers and publishes them to the /numbers topic
    using std_msgs/msg/Int32. Also prints the published values to the console.
"""

import random  # Used to generate random integers

import rclpy  # Core ROS 2 Python client library
from rclpy.node import Node  # Base class for ROS 2 nodes
from std_msgs.msg import Int32  # Message type for publishing integers (NOT a String)


class NumTalker(Node):
    """
    A ROS 2 publisher node that periodically publishes random integers to /numbers.
    """

    def __init__(self) -> None:
        """
        Initialize the NumTalker node.

        Parameters:
            self (NumTalker): The current instance of the NumTalker node.

        Returns:
            None
        """
        # Initialize the parent Node class with a node name
        super().__init__('num_talker')

        # Create a publisher that sends Int32 messages on the /numbers topic
        # The last argument (10) is the QoS queue size (buffer depth)
        self.publisher_ = self.create_publisher(Int32, '/numbers', 10)

        # Create a timer to call timer_callback periodically (0.5s = 2 Hz)
        self.timer_ = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self) -> None:
        """
        Timer callback that generates and publishes a random integer.

        Parameters:
            self (NumTalker): The current instance of the NumTalker node.

        Returns:
            None
        """
        # Construct an Int32 message object
        msg = Int32()

        # Generate a random integer in a reasonable range for demo purposes
        msg.data = random.randint(0, 100)

        # Publish the message to the /numbers topic
        self.publisher_.publish(msg)

        # Print to console via ROS logger so output shows cleanly in ROS
        self.get_logger().info(f'Publishing number: {msg.data}')


def main(args=None) -> None:
    """
    Entry point for the num_talker node.

    Parameters:
        args (list[str] | None): Optional command-line arguments passed by ROS 2.

    Returns:
        None
    """
    # Initialize the ROS 2 communication layer
    rclpy.init(args=args)

    # Create the node instance
    node = NumTalker()

    # Keep the node running, processing callbacks (timers, subscriptions, etc.)
    rclpy.spin(node)

    # Cleanup: explicitly destroy the node before shutting down
    node.destroy_node()

    # Shutdown ROS 2
    rclpy.shutdown()

