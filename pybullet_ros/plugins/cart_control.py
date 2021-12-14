#!/usr/bin/env python3

"""
position, velocity and effort control for the cart on cartpole
"""

from rclpy.node import Node
from pybullet_ros.plugins.control import Control, PveControl


class CartControl(Control):
    def __init__(self, pybullet, robot, **kargs):
        super().__init__(pybullet, robot, **kargs)
        
        # revolute joints - joint position, velocity and effort control command individual subscribers
        for joint_index in self.joint_index_name_dic:
            self.get_logger().info("{} {}".format(joint_index, self.joint_index_name_dic[joint_index]))
            # the pendulum should swing freely
            if (self.joint_index_name_dic[joint_index] == 'revolute_pole'):
                self.pb.setJointMotorControl2(bodyUniqueId=self.robot, jointIndex=joint_index,
                            controlMode=self.pb.POSITION_CONTROL, force=0.0)
                continue
            
