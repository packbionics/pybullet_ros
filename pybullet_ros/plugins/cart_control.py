#!/usr/bin/env python3

"""
position, velocity and effort control for the cart on cartpole
"""

from rclpy.node import Node
from std_msgs.msg import Float64
from pybullet_ros.plugins.control import pveControl


class CartControl(Node):
    def __init__(self, pybullet, robot, **kargs):
        super().__init__('pybullet_ros_control')
        self.rate = self.declare_parameter('loop_rate', 80.0).value
        self.timer = self.create_timer(1.0/self.rate, self.execute)

        # get "import pybullet as pb" and store in self.pb
        self.pb = pybullet
        # get robot from parent class
        self.robot = robot
        # lists to recall last received command (useful when controlling multiple joints)
        self.position_joint_commands = []
        self.velocity_joint_commands = []
        self.effort_joint_commands = []
        # this parameter will be set for all robot joints
        if self.has_parameter('max_effort_vel_mode'):
            self.get_logger().warn('max_effort_vel_mode parameter is deprecated, please use max_effort instead')
            # kept for backwards compatibility, delete after some time
            max_effort = self.declare_parameter('max_effort_vel_mode', 1000.0).value
        else:
            max_effort = self.declare_parameter('max_effort', 1000.0).value
            self.get_logger().info("Control: Max effort={}".format(max_effort))
        # the max force to apply to the joint, used in velocity control
        self.force_commands = []
        # get joints names and store them in dictionary, combine both revolute and prismatic dic
        self.joint_index_name_dic = {**kargs['rev_joints'], **kargs['prism_joints']}
        # setup subscribers
        self.pc_subscribers = []
        self.vc_subscribers = []
        self.ec_subscribers = []
        self.joint_indices = []
        
        self.position_ctrl_joint_indices = []
        self.velocity_ctrl_joint_indices = []
        self.effort_ctrl_joint_indices = []
        
        # revolute joints - joint position, velocity and effort control command individual subscribers
        for joint_index in self.joint_index_name_dic:
            self.get_logger().info("{} {}".format(joint_index, self.joint_index_name_dic[joint_index]))
            # the pendulum should swing freely
            if (self.joint_index_name_dic[joint_index] == 'revolute_pole'):
                self.pb.setJointMotorControl2(bodyUniqueId=self.robot, jointIndex=joint_index,
                            controlMode=self.pb.POSITION_CONTROL, force=0.0)
                continue

            self.position_joint_commands.append(0.0)
            self.velocity_joint_commands.append(0.0)
            self.effort_joint_commands.append(0.0)
            # used only in velocity and position control mode
            self.force_commands.append(max_effort)
            # get joint name from dictionary, is used for naming the subscriber
            joint_name = self.joint_index_name_dic[joint_index]
            # create list of joints for later use in pve_ctrl_cmd(...)
            self.joint_indices.append(joint_index)
            # create position control object
            self.pc_subscribers.append(pveControl(self, joint_index, joint_name, 'position'))
            # create position control object
            self.vc_subscribers.append(pveControl(self, joint_index, joint_name, 'velocity'))
            # create position control object
            self.ec_subscribers.append(pveControl(self, joint_index, joint_name, 'effort'))

    def execute(self):
        """this function gets called from pybullet ros main update loop"""
        """check if user has commanded a joint and forward the request to pybullet"""
        
        self.position_ctrl_joint_indices = []
        self.velocity_ctrl_joint_indices = []
        self.effort_ctrl_joint_indices = []
    
        # create arrays for motor control, position is prioritized first, velocity second, effort third
        for index, subscriber in enumerate(self.pc_subscribers):
            if subscriber.get_is_data_available():
                self.position_joint_commands.append(subscriber.get_last_cmd())
                self.position_ctrl_joint_indices.append(self.joint_indices[index])
 
        for index, subscriber in enumerate(self.vc_subscribers):
            if subscriber.get_is_data_available() and index not in self.position_ctrl_joint_indices:
                self.velocity_joint_commands.append(subscriber.get_last_cmd())
                self.velocity_ctrl_joint_indices.append(self.joint_indices[index])
                
        for index, subscriber in enumerate(self.ec_subscribers):
            if (subscriber.get_is_data_available() 
                and index not in self.position_ctrl_joint_indices 
                and index not in self.velocity_ctrl_joint_indices):
                
                self.effort_joint_commands.append(subscriber.get_last_cmd())
                self.effort_ctrl_joint_indices.append(self.joint_indices[index])
                
        # forward commands to pybullet
        
        # position control
        self.pb.setJointMotorControlArray(bodyUniqueId=self.robot, jointIndices=self.position_ctrl_joint_indices,
                                    controlMode=self.pb.POSITION_CONTROL, targetPositions=self.position_joint_commands, forces=self.force_commands)
        # velocity control
        self.pb.setJointMotorControlArray(bodyUniqueId=self.robot, jointIndices=self.velocity_ctrl_joint_indices,
                                    controlMode=self.pb.VELOCITY_CONTROL, targetVelocities=self.velocity_joint_commands, forces=self.force_commands)
        # effort control
        self.pb.setJointMotorControlArray(bodyUniqueId=self.robot, jointIndices=self.effort_ctrl_joint_indices,
                                    controlMode=self.pb.POSITION_CONTROL, forces=[0.0] * len(self.effort_joint_commands))
        self.pb.setJointMotorControlArray(bodyUniqueId=self.robot, jointIndices=self.effort_ctrl_joint_indices,
                                    controlMode=self.pb.TORQUE_CONTROL, forces=self.effort_joint_commands)
    