#!/usr/bin/env python3

"""
Query robot base pose and speed from pybullet and publish to /odom topic
This component does not add any noise to it
"""

from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, TransformStamped

from pybullet_ros.plugins.ros_plugin import ROSPlugin

class SimpleOdometry(ROSPlugin):
    """Handles odometry for main robot

    Query robot base pose and speed from pybullet and publish to /odom topic.
    This component does not add any noise to it
    """

    def __init__(self, wrapper, pybullet, robot, **kargs):
        """Handles odometry for main robot

        Args:
            pybullet (ModuleType): used to access Pybullet API
            robot (int): first robot loaded
        """

        super().__init__(wrapper, 'pybullet_ros_odometry', pybullet, robot, automatically_declare_parameters_from_overrides=True)

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

    def pub_odom_transform(self):
                    # set msg timestamp based on current time
            now = self.get_clock().now().to_msg()
            self.odom_msg.header.stamp = now
            self.odom_trans.header.stamp = now
            # query base pose from pybullet and store in odom msg
            try:
                position, orientation = self.pb.getBasePositionAndOrientation(self.robot)
            except Exception as ex:
                raise ConnectionError('An error occurred while trying to access position and orientation of robot. Please ensure the robot is fully loaded in the environment.')

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

    def execute(self):
        """this function gets called from pybullet ros main update loop"""
        try:
            if not self.wrapper.pause_simulation:
                self.pub_odom_transform()
        except ConnectionError as connex:
            self.get_logger().info(str(connex))