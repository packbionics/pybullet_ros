#!/usr/bin/env python3

"""
TODO: briefly describe your plugin here
"""

from rclpy.node import Node

class ROSPlugin(Node):
    def __init__(self, wrapper, name, pybullet, robot, **kargs):
        # reference to pybullet wrapper
        self.wrapper = wrapper
        # node name
        self.name = name
        # get "import pybullet as pb" and store in self.pb
        self.pb = pybullet
        # get robot from parent class
        self.robot = robot

        # construct ROS 2 node
        super().__init__(self.name)

        self.declare_parameter('loop_rate', 40.0)

        # define plugin loop
        self.rate = self.get_parameter('loop_rate').value
        self.timer = self.create_timer(1.0/self.rate, self.execute)

    def execute(self):
        """this function gets called from pybullet ros main update loop"""
        self.node.get_logger().info('my plugin is running!')
