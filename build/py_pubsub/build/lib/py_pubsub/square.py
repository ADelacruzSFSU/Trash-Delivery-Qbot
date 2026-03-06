"""
Name: Anthony Delacruz
Filename: square.py
Description:
    ROS 2 node that subscribes to the /numbers topic (std_msgs/msg/Int32),
    squares the received integer, and prints the squared result to the console.
"""

import rclpy  # Core ROS 2 Python client library
from rclpy.node import Node  # Base class for ROS 2 nodes
from std_msgs.msg import Int32  # Message type being received


class Square(Node):
    """
    A ROS 2 subscriber node that listens on /numbers and prints squared values.
    """

    def __init__(self) -> None:
        """
        Initialize the Square node and create the subscription.

        Parameters:
            self (Square): The current instance of the Square node.

        Returns:
            None
        """
        # Initialize the parent Node class with a node name
        super().__init__('square')

        # Create a subscription to the /numbers topic with Int32 messages
        # The last argument (10) is the QoS queue size (buffer depth)
        self.subscription_ = self.create_subscription(
            Int32,
            '/numbers',
            self.listener_callback,
            10
        )

        # Store the subscription to prevent it from being garbage collected
        # (This is a common ROS 2 Python pattern.)
        self.subscription_

    def listener_callback(self, msg: Int32) -> None:
        """
        Callback that runs whenever a message is received on /numbers.

        Parameters:
            self (Square): The current instance of the Square node.
            msg (std_msgs.msg.Int32): The received integer message.

        Returns:
            None
        """
        # Extract the integer from the message
        value = msg.data

        # Compute the squared value
        squared_value = value * value

        # Print the squared result to the console using ROS logger
        self.get_logger().info(f'Received: {value} | Squared: {squared_value}')


def main(args=None) -> None:
    """
    Entry point for the square node.

    Parameters:
        args (list[str] | None): Optional command-line arguments passed by ROS 2.

    Returns:
        None
    """
    # Initialize the ROS 2 communication layer
    rclpy.init(args=args)

    # Create the node instance
    node = Square()

    # Keep the node running to process subscription callbacks
    rclpy.spin(node)

    # Cleanup: explicitly destroy the node before shutting down
    node.destroy_node()

    # Shutdown ROS 2
    rclpy.shutdown()

