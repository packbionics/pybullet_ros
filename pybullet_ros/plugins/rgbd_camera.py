#!/usr/bin/env python3

"""
RGBD camera sensor simulation for pybullet_ros base on pybullet.getCameraImage()
"""

from os.path import exists
import math
from re import S

import numpy as np
import rclpy
import cv2
from cv_bridge import CvBridge
from rclpy.node import Node

from sensor_msgs.msg import Image

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

from pointcloud_interfaces.srv import CameraParams


class RGBDCamera(Node):
    """Plugin used to process camera frames from simulated camera in Pybullet

    Attributes:
        rate (float): _description_
        timer (Timer): _description_
        pb (ModuleType): _description_
        robot (int): _description_
        camera_pose_msg (PoseStamped): _description_
        camera_position (Point): _description_
        camera_orientation (Quaternion): _description_
        image_msg (Image): _description_
        image_mode (str): _description_
        pb_camera_link_id (int): _description_
        pub_camera_state (_type_): _description_
        pub_image (Image): _description_
        hfov (float): _description_
        vfov (float): _description_
        near_plane (float): _description_
        far_plane (float): _description_
        projection_matrix (numpy.array): _description_
        camera_params_serv (CameraParams): _description_
        image_bridge (CvBridge): _description_
    """

    def __init__(self, pybullet, robot, **kargs):
        """sets up parameters, topics, and services

        Args:
            pybullet (ModuleType): used to access Pybullet API
            robot (int): first robot loaded
        """

        super().__init__('pybullet_ros_rgbd_camera')
        self.rate = self.declare_parameter('loop_rate', 80.0).value
        self.timer = self.create_timer(1.0/self.rate, self.execute)
        # get "import pybullet as pb" and store in self.pb
        self.pb = pybullet
        # get robot from parent class
        self.robot = robot
        # create msg placeholders for publication
        self.camera_pose_msg = PoseStamped()
        self.camera_position = Point()
        self.camera_orientation = Quaternion()
        self.image_msg = Image()
        self.image_mode = self.declare_parameter('image_mode', 'rgb').value
        assert self.image_mode in ['rgb', 'rgbd', 'segmentation']
        # get RGBD camera parameters from ROS param server
        self.image_msg.width = self.declare_parameter('rgbd_camera/resolution/width', 640).value
        self.image_msg.height = self.declare_parameter('rgbd_camera/resolution/height', 480).value
        assert(self.image_msg.width > 5)
        assert(self.image_msg.height > 5)
        cam_frame_id = self.declare_parameter('rgbd_camera/frame_id', None).value
        if not cam_frame_id:
            self.get_logger().error('Required parameter rgbd_camera/frame_id not set, will exit now...')
            rclpy.shutdown()
            return
        # get pybullet camera link id from its name
        link_names_to_ids_dic = kargs['link_ids']
        if not cam_frame_id in link_names_to_ids_dic:
            self.get_logger().error('Camera reference frame "{}" not found in URDF model'.format(cam_frame_id))
            self.get_logger().warn('Available frames are: {}'.format(link_names_to_ids_dic))
            rclpy.shutdown()
            return
        self.pb_camera_link_id = link_names_to_ids_dic[cam_frame_id]
        self.image_msg.header.frame_id = cam_frame_id
        # create publishers
        self.pub_camera_state = self.create_publisher(PoseStamped, 'camera/state', 2)
        self.pub_image = self.create_publisher(Image, 'camera/depth/image_raw', 2)
        self.image_msg.encoding = self.declare_parameter('rgbd_camera/resolution/encoding', 'rgb8').value
        self.image_msg.is_bigendian = self.declare_parameter('rgbd_camera/resolution/is_bigendian', 0).value
        self.image_msg.step = self.declare_parameter('rgbd_camera/resolution/step', 1920).value
        # projection matrix
        self.hfov = self.declare_parameter('rgbd_camera/hfov', 56.3).value
        self.vfov = self.declare_parameter('rgbd_camera/vfov', 43.7).value
        self.near_plane = self.declare_parameter('rgbd_camera/near_plane', 0.4).value
        self.far_plane = self.declare_parameter('rgbd_camera/far_plane', 8.0).value
        self.projection_matrix = self.compute_projection_matrix()
        # create service
        self.camera_params_serv = self.create_service(CameraParams, 'camera/params', self.camera_params_callback)

        # use cv_bridge ros to convert cv matrix to ros format
        self.image_bridge = CvBridge()

        self.get_logger().info('RGBD camera plugin initialized, camera parameters:')
        self.get_logger().info('  hfov: {}'.format(self.hfov))
        self.get_logger().info('  vfov: {}'.format(self.vfov))
        self.get_logger().info('  near_plane: {}'.format(self.near_plane))
        self.get_logger().info('  far_plane: {}'.format(self.far_plane))
        self.get_logger().info('  resolution: {}x{}'.format(self.image_msg.width, self.image_msg.height))
        self.get_logger().info('  frame_id: {}'.format(self.image_msg.header.frame_id))

    def camera_params_callback(self, request, response):
        """handles request for service

        Args:
            request (CameraParams.Request): holds information related to service request from client
            response (CameraParams): used to send details of the physical (or simulated) camera

        Returns:
            CameraParams: details of the physical (or simulated) camera
        """

        response.hfov = self.hfov
        response.vfov = self.vfov
        
        response.near = self.near_plane
        response.far = self.far_plane

        self.get_logger().info('sending service response...')
        return response

    def compute_projection_matrix(self):
        """calculated a 4x4 project matrix based on intrinsic camera parameters

        Returns:
            numpy.array: 4x4 project matrix
        """

        return self.pb.computeProjectionMatrix(
                    left=-math.tan(math.pi * self.hfov / 360.0) * self.near_plane,
                    right=math.tan(math.pi * self.hfov / 360.0) * self.near_plane,
                    bottom=-math.tan(math.pi * self.vfov / 360.0) * self.near_plane,
                    top=math.tan(math.pi * self.vfov / 360.0) * self.near_plane,
                    nearVal=self.near_plane,
                    farVal=self.far_plane)

    def extract_frame(self, camera_image):
        """retrieves the RGB information from a frame of the camera

        Args:
            camera_image (numpy.array): numpy array containing frame in RGBA format

        Returns:
            numpy.array: numpy array containing frame in RGB format
        """

        camera_image = np.reshape(camera_image[2], (camera_image[1], camera_image[0], 4))

        rgb_image = camera_image[:,:,:3]

        # return frame
        return rgb_image.astype(np.uint8)

    def extract_depth(self, camera_image):
        """retrieves depth information from a frame of the camera

        Args:
            camera_image (numpy.array): numpy array containing frame in RGBA format

        Returns:
            numpy.array: numpy array containing depth information in range 0.0 - 1.0
        """

        # Extract depth buffer
        depth_buffer = np.reshape(camera_image[3], (camera_image[1], camera_image[0]))

        if self.image_msg.encoding == '8UC1':
            # Convert from floating-point to 8-bit format and return result
            return (depth_buffer*255).astype(np.uint8)
        elif self.image_msg.encoding == 'mono16':
            # Convert from floating-point to 16-bit format and return result
            return (depth_buffer*(255**2)).astype(np.uint16)
        else:
            # no conversion
            return depth_buffer

    def compute_camera_target(self, camera_position, camera_orientation):
        """calculates a position 5m in front of the camera

        camera target is a point 5m in front of the robot camera
        This method is used to tranform it to the world reference frame
        NOTE: this method uses pybullet functions and not tf

        Args:
            camera_position (numpy.array): XYZ position of the camera
            camera_orientation (numpy.array): Quaternion representation of the camera orientation

        Returns:
            np.array: a XYZ position 5m in front of the camera
        """

        target_point = [5.0, 0, 0] # expressed w.r.t camera reference frame
        camera_position = [camera_position[0], camera_position[1], camera_position[2]]
        rm = self.pb.getMatrixFromQuaternion(camera_orientation)
        rotation_matrix = [[rm[0], rm[1], rm[2]],[rm[3], rm[4], rm[5]],[rm[6], rm[7], rm[8]]]
        return np.dot(rotation_matrix, target_point) + camera_position

    def execute(self):
        """main loop of the plugin"""

        # get camera pose
        cam_state = self.pb.getLinkState(self.robot, self.pb_camera_link_id)
        # target is a point 5m ahead of the robot camera expressed w.r.t world reference frame
        target = self.compute_camera_target(cam_state[0], cam_state[1])
        view_matrix = self.pb.computeViewMatrix(cam_state[0], target, [0, 0, 1])

        self.camera_position.x = cam_state[0][0]
        self.camera_position.y = cam_state[0][1]
        self.camera_position.z = cam_state[0][2]

        self.camera_orientation.x = cam_state[1][0]
        self.camera_orientation.y = cam_state[1][1]
        self.camera_orientation.z = cam_state[1][2]
        self.camera_orientation.w = cam_state[1][3]

        self.camera_pose_msg.header.stamp = self.get_clock().now().to_msg()
        self.camera_pose_msg.pose.position = self.camera_position
        self.camera_pose_msg.pose.orientation = self.camera_orientation

        # get camera image from pybullet
        pybullet_cam_resp = self.pb.getCameraImage(self.image_msg.width,
                                                    self.image_msg.height,
                                                    view_matrix,
                                                    self.projection_matrix,
                                                    renderer=self.pb.ER_BULLET_HARDWARE_OPENGL
                                                )
        # frame extraction function from qibullet
        if self.image_mode == 'rgb':
            frame = self.extract_frame(pybullet_cam_resp)
        else:
            frame = self.extract_depth(pybullet_cam_resp)
            #print(frame)

        # fill pixel data array
        self.image_msg.data = self.image_bridge.cv2_to_imgmsg(frame, self.image_msg.encoding).data
        # update msg time stamp
        self.image_msg.header.stamp = self.get_clock().now().to_msg()
        # publish camera image to ROS network
        self.pub_camera_state.publish(self.camera_pose_msg)
        self.pub_image.publish(self.image_msg)