#!/usr/bin/env python3

"""
Laser scanner simulation based on pybullet rayTestBatch function
This code does not add noise to the laser scanner readings
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class laserScanner:
    def __init__(self, node, pybullet, robot, **kargs):
        # get "import pybullet as pb" and store in self.pb
        self.node = node
        self.pb = pybullet
        # get robot from parent class
        self.robot = robot
        # laser params
        laser_frame_id = self.node.declare_parameter('laser/frame_id', None).value # laser reference frame, has to be an existing link
        if not laser_frame_id:
            self.node.get_logger().error('required parameter laser_frame_id not set, will exit now')
            rclpy.shutdown()
            return
        # get pybullet laser link id from its name
        link_names_to_ids_dic = kargs['link_ids']
        if not laser_frame_id in link_names_to_ids_dic:
            self.node.get_logger().error('laser reference frame "{}" not found in URDF model, cannot continue'.format(laser_frame_id))
            self.node.get_logger().error('Available frames are: {}'.format(link_names_to_ids_dic))
            rclpy.shutdown()
            return
        self.pb_laser_link_id = link_names_to_ids_dic[laser_frame_id]
        # create laser msg placeholder for publication
        self.laser_msg = LaserScan()
        # laser field of view
        angle_min = self.node.declare_parameter('laser/angle_min', -1.5707963).value
        angle_max = self.node.declare_parameter('laser/angle_max', 1.5707963).value
        assert(angle_max > angle_min)
        self.numRays = self.node.declare_parameter('laser/num_beams', 50).value # should be 512 beams but simulation becomes slow
        self.laser_msg.range_min = self.node.declare_parameter('laser/range_min', 0.03).value
        self.laser_msg.range_max = self.node.declare_parameter('laser/range_max', 5.6).value
        self.beam_visualisation = self.node.declare_parameter('laser/beam_visualisation', False).value
        self.laser_msg.angle_min = angle_min
        self.laser_msg.angle_max = angle_max
        self.laser_msg.angle_increment = (angle_max - angle_min) / self.numRays
        # register this node in the network as a publisher in /scan topic
        self.pub_laser_scanner = self.node.create_publisher(LaserScan, 'scan', 1)
        self.laser_msg.header.frame_id = laser_frame_id
        self.laser_msg.time_increment = 0.01 # ?
        self.laser_msg.scan_time = 0.1 # 10 hz
        # fixed_joint_index_name_dic = kargs['fixed_joints']
        self.rayHitColor = [1, 0, 0] # red color
        self.rayMissColor = [0, 1, 0] # green color
        # compute rays end beam position
        self.rayFrom, self.rayTo = self.prepare_rays()
        # variable used to run this plugin at a lower frequency, HACK
        self.count = 0

    def prepare_rays(self):
        """assume laser is in the origin and compute its x, y beam end position"""
        # prepare raycast origin and end values
        rayFrom = []
        rayTo = []
        for n in range(0, self.numRays):
            alpha = self.laser_msg.angle_min + n * self.laser_msg.angle_increment
            rayFrom.append([self.laser_msg.range_min * math.cos(alpha),
                          self.laser_msg.range_min * math.sin(alpha), 0.0])
            rayTo.append([self.laser_msg.range_max * math.cos(alpha),
                          self.laser_msg.range_max * math.sin(alpha), 0.0])
        return rayFrom, rayTo

    def transform_rays(self, laser_position, laser_orientation):
        """transform rays from reference frame using pybullet functions (not tf)"""
        laser_position = [laser_position[0], laser_position[1], laser_position[2]]
        TFrayFrom = []
        TFrayTo = []
        rm = self.pb.getMatrixFromQuaternion(laser_orientation)
        rotation_matrix = [[rm[0], rm[1], rm[2]],[rm[3], rm[4], rm[5]],[rm[6], rm[7], rm[8]]]
        for ray in self.rayFrom:
            position = np.dot(rotation_matrix, [ray[0], ray[1], ray[2]]) + laser_position
            TFrayFrom.append([position[0], position[1], position[2]])
        for ray in self.rayTo:
            position = np.dot(rotation_matrix, [ray[0], ray[1], ray[2]]) + laser_position
            TFrayTo.append([position[0], position[1], position[2]])
        return TFrayFrom, TFrayTo

    def execute(self):
        """this function gets called from pybullet ros main update loop"""
        # run at lower frequency, laser computations are expensive
        self.count += 1
        if self.count < 100:
            return
        self.count = 0 # reset count
        # remove any previous laser data if any
        self.laser_msg.ranges = []
        # remove previous beam lines from screen
        if self.beam_visualisation:
            self.pb.removeAllUserDebugItems()
        # get laser link position
        laser_state = self.pb.getLinkState(self.robot, self.pb_laser_link_id)
        # transform start and end position of the rays which were generated considering laser at the origin
        rayFrom, rayTo  = self.transform_rays(laser_state[0], laser_state[1]) # position + orientation
        # raycast using 4 threads
        results = self.pb.rayTestBatch(rayFrom, rayTo, 4)
        for i in range(self.numRays):
            if self.beam_visualisation:
                hitObjectUid = results[i][0]
                if hitObjectUid < 0:
                    # draw a line on pybullet gui for debug purposes in green because it did not hit any obstacle
                    self.pb.addUserDebugLine(rayFrom[i], rayTo[i], self.rayMissColor)
                else:
                    # draw a line on pybullet gui for debug purposes in red because it hited obstacle, results[i][3] -> hitPosition
                    self.pb.addUserDebugLine(rayFrom[i], results[i][3], self.rayHitColor)
            # compute laser ranges from hitFraction -> results[i][2]
            self.laser_msg.ranges.append(results[i][2] * self.laser_msg.range_max)
        # update laser time stamp with current time
        self.laser_msg.header.stamp = self.node.get_clock().now().to_msg()
        # publish scan
        self.pub_laser_scanner.publish(self.laser_msg)
