#!/usr/bin/env python3

"""
query robot state and publish position, velocity and effort values to /joint_states
"""
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStatePub(Node):
    def __init__(self, pybullet, robot, **kargs):
        super().__init__('pybullet_ros_joint_state_pub', 
            automatically_declare_parameters_from_overrides=True)
        self.rate = self.get_parameter('loop_rate').value
        self.timer = self.create_timer(1.0/self.rate, self.execute)
        # get "import pybullet as pb" and store in self.pb
        self.pb = pybullet
        # get robot from parent class
        self.robot = robot
        # get revolute joints names and store them in dictionary
        self.rev_joint_index_name_dic = kargs['rev_joints']
        # get prismatic joints names and store them in dictionary
        self.prism_joint_index_name_dic = kargs['prism_joints']
        # combine dictionaries of revolute and prismatic joints
        self.joint_index_name_dic = {**self.prism_joint_index_name_dic, **self.rev_joint_index_name_dic}
        # register this node in the network as a publisher in /joint_states topic
        self.pub_joint_states = self.create_publisher(JointState, 'joint_states', 1)

    def execute(self):
        """this function gets called from pybullet ros main update loop"""
        # setup msg placeholder
        joint_msg = JointState()
        # get joint states
        for joint_index in self.joint_index_name_dic:
            # get joint state from pybullet
            joint_state = self.pb.getJointState(self.robot, joint_index)
            # fill msg
            joint_msg.name.append(self.joint_index_name_dic[joint_index])
            joint_msg.position.append(joint_state[0])
            joint_msg.velocity.append(joint_state[1])
            joint_msg.effort.append(joint_state[3]) # applied effort in last sim step
        # update msg time using ROS time api
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        # publish joint states to ROS
        self.pub_joint_states.publish(joint_msg)
