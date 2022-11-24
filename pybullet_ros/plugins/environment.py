#!/usr/bin/env python3

"""
plugin that is loaded one time only at the beginning
It is meant to be for you to upload your environment
"""

import rclpy


class Environment:
    def __init__(self, pybullet_ros_node, **kargs):
        # get "import pybullet as pb" and store in self.pb
        self.node = pybullet_ros_node
        self.pb = self.node.pb
        # enable soft body simulation if needed
        if self.node.get_parameter('use_deformable_world').value:
            self.node.get_logger().info('Using deformable world (soft body simulation)')
            self.pb.resetSimulation(self.pb.RESET_USE_DEFORMABLE_WORLD)

    def load_environment(self):
        """
        set gravity, ground plane and load URDF or SDF models as required
        """
        # set gravity
        gravity = self.node.get_parameter('gravity').value # get gravity from param server
        self.pb.setGravity(0, 0, gravity)
        # set floor
        self.pb.loadURDF('plane.urdf')
        