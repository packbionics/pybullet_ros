#!/usr/bin/env python3

"""
TODO: briefly describe your plugin here
"""

from rclpy.node import Node

class RosPlugin(Node):
    def __init__(self, name, pybullet, robot, **kargs):
        self.name = name
        # get "import pybullet as pb" and store in self.pb
        self.pb = pybullet
        # get robot from parent class
        self.robot = robot

        # construct ROS 2 node
        if 'automatically_declare_parameters_from_overrides' in kargs:
            super().__init__(self.name, automatically_declare_parameters_from_overrides=kargs['automatically_declare_parameters_from_overrides'])
        else:
            super().__init__(self.name)
        # TODO: implement here...

        # define plugin loop
        self.rate = self.get_parameter('loop_rate').value
        self.timer = self.create_timer(1.0/self.rate, self.execute)

    def execute(self):
        """this function gets called from pybullet ros main update loop"""
        self.node.get_logger().info('my plugin is running!')
