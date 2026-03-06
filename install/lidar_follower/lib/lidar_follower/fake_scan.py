#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import time

class FakeScan(Node):
    def __init__(self):
        super().__init__('fake_scan')
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.timer = self.create_timer(0.2, self.publish_scan)
        self.phase = 0
        self.counter = 0

    def publish_scan(self):
        msg = LaserScan()
        msg.header.frame_id = "base_scan"
        msg.angle_min = -0.5
        msg.angle_max = 0.5
        msg.angle_increment = 0.05
        msg.range_min = 0.0
        msg.range_max = 10.0

        num_points = int((msg.angle_max - msg.angle_min) / msg.angle_increment) + 1
        ranges = [3.0] * num_points  # default: no object

        # Phase behavior
        if self.phase == 0:
            # Object centered at 1.5m
            for i in range(8, 13):
                ranges[i] = 1.5
        elif self.phase == 1:
            # Object shifted left
            for i in range(5, 9):
                ranges[i] = 1.0
        elif self.phase == 2:
            # Object shifted right
            for i in range(12, 16):
                ranges[i] = 1.0
        elif self.phase == 3:
            # Object too close
            for i in range(8, 13):
                ranges[i] = 0.6

        msg.ranges = ranges
        self.pub.publish(msg)

        self.counter += 1
        if self.counter > 20:
            self.phase = (self.phase + 1) % 4
            self.counter = 0

def main():
    rclpy.init()
    node = FakeScan()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
