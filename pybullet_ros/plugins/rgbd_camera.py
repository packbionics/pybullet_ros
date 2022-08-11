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
from sensor_msgs.msg import CameraInfo

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

from pointcloud_interfaces.srv import CameraParams


class RGBDCamera(Node):
    """Plugin used to process camera frames from simulated camera in Pybullet

    Attributes:
        rate (float): determines the rate of execute()
        timer (Timer): handles the main loop of the plugin
        pb (ModuleType): used to access Pybullet API
        robot (int): id for the first loaded robot
        camera_pose_msg (PoseStamped): contains the XYZ position and orientation of the camera
        camera_position (Point): used to update XYZ position of camera
        camera_orientation (Quaternion): used to update orientation of camera
        image_msg (Image): contains image information of the most current camera frame
        image_mode (str): specifies the image mode to access (e.g. RGB, depth, etc)
        pb_camera_link_id (int): id of the camera link used for computer vision
        pub_camera_state (Publisher): ROS 2 publisher for broadcasting current camera pose
        pub_image (Publisher): ROS 2 publisher for broadcasting current simulated frame
        hfov (float): horizontal FOV of camera
        vfov (float): vertical FOV of the camera
        near_plane (float): nearest depth to the camera that is captured
        far_plane (float): farthest depth from the camera that is captured
        projection_matrix (numpy.array): 4x4 matrix that maps 3D points to image plane
        camera_params_serv (CameraParams): service that handles requests for camera parameters
        image_bridge (CvBridge): used for converting OpenCV images to ROS 2 Image msg type
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
        self.image_mode = self.declare_parameter('image_mode', 'rgb8').value
        assert self.image_mode in ['rgb', 'rgbd', 'depth', 'segmentation']
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
        self.pub_image = self.create_publisher(Image, 'camera/depth/image_raw', 10)
        self.pub_camera_info = self.create_publisher(CameraInfo, 'camera/info', 10)
        self.image_msg.encoding = self.declare_parameter('rgbd_camera/resolution/encoding', 'rgb8').value
        self.image_msg.is_bigendian = self.declare_parameter('rgbd_camera/resolution/is_bigendian', 0).value
        self.image_msg.step = self.declare_parameter('rgbd_camera/resolution/step', 1920).value
        assert self.image_msg.encoding in ['32FC1'] + ['mono8', 'mono16', 'bgr8', 'rgb8', 'bgra8', 'rgba8', 'passthrough']
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

        # information about simulated camera
        self.camera_info = CameraInfo()

        # used by ROS to handle ROS msgs
        self.camera_info.header = self.image_msg.header

        # resolution of camera
        self.camera_info.height = self.image_msg.width
        self.camera_info.width = self.image_msg.height

        # distortion model
        self.camera_info.distortion_model = 'plumb_bob'

        # distortion parameters (no distortion in simulation)
        self.camera_info.d = [0.0] * 5

        # intrinsic camera matrix
        self.camera_info.k = [  450.0, 0.0,    640.0,
                                0.0,   515.0,  320.0,
                                0.0,   0.0,    1.0  ]

        # rectification matrix (identify matrix for monocular camera)
        self.camera_info.r = [  1.0, 0.0, 0.0,
                                0.0, 1.0, 0.0,
                                0.0, 0.0, 1.0]

        # projection matrix
        self.camera_info.p = [  450.0, 0.0,    640.0,  0.0,
                                0.0,   515.0,  320.0,  0.,
                                0.0,   0.0,    0.0,    1.0]

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

        if self.image_msg.encoding == 'mono8':
            # Convert from floating-point to 8-bit format and return result
            return (depth_buffer*255).astype(np.uint8)
        elif self.image_msg.encoding == 'mono16':
            # Convert from floating-point to 16-bit format and return result
            return (depth_buffer*(255**2)).astype(np.uint16)
        else:
            # no conversion
            return depth_buffer

    def calc_true_depth(self, depth_img, near, far):
        depth_range = far - near

        depth_img = depth_img * depth_range
        depth_img = far - depth_img
        depth_img = far / depth_img

        return depth_img


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
        # frame extraction function from pybullet
        if self.image_mode == 'rgb8':
            frame = self.extract_frame(pybullet_cam_resp)
        elif self.image_mode == 'depth':
            frame = self.extract_depth(pybullet_cam_resp)
            frame = self.calc_true_depth(frame, self.near_plane, self.far_plane)

        # fill pixel data array
        self.image_msg.data = self.image_bridge.cv2_to_imgmsg(frame, self.image_msg.encoding).data
        # update msg time stamp
        self.image_msg.header.stamp = self.get_clock().now().to_msg()
        self.camera_info.header.stamp = self.image_msg.header.stamp
        # publish camera image to ROS network
        self.pub_camera_state.publish(self.camera_pose_msg)
        self.pub_image.publish(self.image_msg)
        self.pub_camera_info.publish(self.camera_info)