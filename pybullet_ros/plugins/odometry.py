#!/usr/bin/env python3

"""
Query robot base pose and speed from pybullet and publish to /odom topic
This component does not add any noise to it
"""

from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, TransformStamped


class SimpleOdometry(Node):
    def __init__(self, pybullet, robot, **kargs):
        super().__init__('pybullet_ros_odometry')
        self.rate = self.declare_parameter('loop_rate', 80.0).value
        self.timer = self.create_timer(1.0/self.rate, self.execute)
        # get "import pybullet as pb" and store in self.pb
        self.pb = pybullet
        # get robot from parent class
        self.robot = robot
        # register this node as a /odom publisher
        self.pub_odometry = self.create_publisher(Odometry, 'odom', 1)
        # save some overhead by setting some information only once
        self.odom_msg = Odometry()
        self.odom_trans = TransformStamped()
        odom_frame_param = self.declare_parameter('odom_frame', 'odom').value
        robot_base_frame_param = self.declare_parameter('robot_base_frame', 'base_link').value
        self.odom_msg.header.frame_id = odom_frame_param
        self.odom_msg.child_frame_id = robot_base_frame_param
        self.odom_trans.header.frame_id = odom_frame_param
        self.odom_trans.child_frame_id = robot_base_frame_param
        self.br = TransformBroadcaster(self)

    def execute(self):
        """this function gets called from pybullet ros main update loop"""
        # set msg timestamp based on current time
        now = self.get_clock().now().to_msg()
        self.odom_msg.header.stamp = now
        self.odom_trans.header.stamp = now
        # query base pose from pybullet and store in odom msg
        position, orientation = self.pb.getBasePositionAndOrientation(self.robot)
        [self.odom_trans.transform.translation.x,\
         self.odom_trans.transform.translation.y,\
         self.odom_trans.transform.translation.z] = position
        [self.odom_msg.pose.pose.position.x,\
         self.odom_msg.pose.pose.position.y,\
         self.odom_msg.pose.pose.position.z] = position
        [self.odom_trans.transform.rotation.x,\
         self.odom_trans.transform.rotation.y,\
         self.odom_trans.transform.rotation.z,\
         self.odom_trans.transform.rotation.w] = orientation
        [self.odom_msg.pose.pose.orientation.x,\
         self.odom_msg.pose.pose.orientation.y,\
         self.odom_msg.pose.pose.orientation.z,\
        self.odom_msg.pose.pose.orientation.w] = orientation
        # query base velocity from pybullet and store it in msg
        [self.odom_msg.twist.twist.linear.x,\
         self.odom_msg.twist.twist.linear.y,\
         self.odom_msg.twist.twist.linear.z],\
        [self.odom_msg.twist.twist.angular.x,\
         self.odom_msg.twist.twist.angular.y,\
         self.odom_msg.twist.twist.angular.z] = self.pb.getBaseVelocity(self.robot)
        self.pub_odometry.publish(self.odom_msg)
        # tf broadcast (odom to base_link)
        self.br.sendTransform(self.odom_trans)
