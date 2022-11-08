#!/usr/bin/env python3

"""
query robot state and publish position, velocity and effort values to /link_states
"""
from jetleg_interfaces.msg import LinkState

from geometry_msgs.msg import Point, Quaternion

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

    def __init__(self, wrapper, pybullet, robot, **kargs):
        """publishes the joint states of the robot

        Args:
            pybullet (ModuleType): used to access Pybullet API
            robot (int): first robot loaded
        """

        super().__init__(wrapper, 'pybullet_ros_link_state_pub', pybullet, robot, automatically_declare_parameters_from_overrides=True)

        # retrieve dictionary of link names to link ids
        self.link_names_to_ids_dic = kargs['link_ids']
        # register this node in the network as a publisher in /link_states topic
        self.pub_link_states = self.create_publisher(LinkState, 'link_states', rclpy.qos.qos_profile_system_default)

    def get_link_state(self):
        link_msg = LinkState()

        # get link states
        for link_name in self.link_names_to_ids_dic:
            # setup msg placeholder
            point = Point()
            orientation = Quaternion()

            # get joint state from pybullet
            try:
                link_state = self.pb.getLinkState(self.robot, self.link_names_to_ids_dic[link_name])
            except Exception as ex:
                raise ConnectionError('An error occurred while trying to access link state. Please ensure robot is fully loaded in the environment.')

            # fill msg pose
            point.x = link_state[0][0]
            point.y = link_state[0][1]
            point.z = link_state[0][2]

            orientation.x = link_state[1][0]
            orientation.y = link_state[1][1]
            orientation.z = link_state[1][2]
            orientation.w = link_state[1][3]

            link_msg.name.append(link_name)
            link_msg.position.append(point)
            link_msg.orientation.append(orientation)

        # fill msg header
        link_msg.header.stamp = self.get_clock().now().to_msg()
        return link_msg

    def pub_link_state(self):
        # Access link state from PyBullet
        link_msg = self.get_link_state()

        # publish joint states to ROS
        self.pub_link_states.publish(link_msg)
    
    def execute(self):
        """this function gets called from pybullet ros main update loop"""

        try:
            if not self.wrapper.pause_simulation:
                self.pub_link_state()
        except ConnectionError as connex:
            self.get_logger().info(str(connex))
