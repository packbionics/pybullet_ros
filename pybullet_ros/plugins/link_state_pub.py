#!/usr/bin/env python3

"""
query robot state and publish position, velocity and effort values to /link_states
"""
from geometry_msgs.msg import PoseStamped

from pybullet_ros.plugins.ros_plugin import RosPlugin

import rclpy.qos

class LinkStatePub(RosPlugin):
    """query robot state and publish position, velocity and effort values to /joint_states

    Attributes:
        rate (float): determines the rate of execute()
        timer (Timer): handles the main loop of the plugin
        pb (ModuleType): used to access Pybullet API
        robot (int): id for the first loaded robot
        link_names_to_ids_dic (dict): dictionary mapping link names to link ids
        pub_link_states (Publisher): publisher for broadcasting joint states
    """

    def __init__(self, pybullet, robot, **kargs):
        """publishes the joint states of the robot

        Args:
            pybullet (ModuleType): used to access Pybullet API
            robot (int): first robot loaded
        """

        super().__init__('pybullet_ros_link_state_pub', pybullet, robot, automatically_declare_parameters_from_overrides=True)

        # retrieve dictionary of link names to link ids
        self.link_names_to_ids_dic = kargs['link_ids']
        # register this node in the network as a publisher in /link_states topic
        self.pub_link_states = self.create_publisher(PoseStamped, 'link_states', rclpy.qos.qos_profile_system_default)

    def execute(self):
        """this function gets called from pybullet ros main update loop"""
        # get link states
        for link_name in self.link_names_to_ids_dic:
            # setup msg placeholder
            link_msg = PoseStamped()

            # get joint state from pybullet
            link_state = self.pb.getLinkState(self.robot, self.link_names_to_ids_dic[link_name])

            # fill msg header
            link_msg.header.frame_id = link_name
            link_msg.header.stamp = self.get_clock().now().to_msg()
            # fill msg pose
            link_msg.pose.position.x = link_state[0][0]
            link_msg.pose.position.y = link_state[0][1]
            link_msg.pose.position.z = link_state[0][2]

            link_msg.pose.orientation.x = link_state[1][0]
            link_msg.pose.orientation.y = link_state[1][1]
            link_msg.pose.orientation.z = link_state[1][2]
            link_msg.pose.orientation.w = link_state[1][3]
    
            # publish joint states to ROS
            self.pub_link_states.publish(link_msg)
