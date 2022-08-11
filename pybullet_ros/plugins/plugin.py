#!/usr/bin/env python3

"""
TODO: briefly describe your plugin here
"""

import rclpy
from rclpy.node import Node


class Plugin:
    def __init__(self, node, pybullet, robot, **kargs):
        self.node = node
        # get "import pybullet as pb" and store in self.pb
        self.pb = pybullet
        # get robot from parent class
        self.robot = robot
        # TODO: implement here...

    def execute(self):
        """this function gets called from pybullet ros main update loop"""
        self.node.get_logger().info('my plugin is running!')
