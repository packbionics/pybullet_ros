#!/usr/bin/env python3

"""
RGBD camera sensor simulation for pybullet_ros based on pybullet.getCameraImage()
"""

import math

import numpy as np
import rclpy
import rclpy.qos

import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

from pybullet_ros.plugins.ros_plugin import ROSPlugin


class RGBDCamera(ROSPlugin):
    """Plugin used to process camera frames from simulated camera in Pybullet"""

    def __init__(self, wrapper, pybullet, robot, **kargs):
        """sets up parameters, topics, and services

        Args:
            pybullet (ModuleType): used to access Pybullet API
            robot (int): first robot loaded
        """

        super().__init__(wrapper, 'pybullet_ros_rgbd_camera', pybullet, robot)
        
        # create msg placeholders for publication
        self.camera_pose_msg = PoseStamped()
        self.camera_position = Point()
        self.camera_orientation = Quaternion()
        self.rgb_img_msg = Image()
        self.depth_img_msg = Image()

        self.declare_parameter('image_mode', 'rgbd')
        self.declare_parameter('rgbd_camera/resolution/width', 1280)
        self.declare_parameter('rgbd_camera/resolution/height', 720)
        self.declare_parameter('rgbd_camera/frame_id', 'camera_link')
        self.declare_parameter('rgbd_camera/resolution/is_bigendian', False)
        self.declare_parameter('rgbd_camera/resolution/encoding', 'rgba8')
        self.declare_parameter('rgbd_camera/resolution/step', 5120)
        self.declare_parameter('rgbd_camera/hfov', 110.0)
        self.declare_parameter('rgbd_camera/vfov', 70.0)
        self.declare_parameter('rgbd_camera/near_plane', 1.0)
        self.declare_parameter('rgbd_camera/far_plane', 20.0)
        
        self.image_mode = self.get_parameter('image_mode').value
        assert self.image_mode in ['rgb', 'rgbd', 'depth', 'segmentation']
        # get RGBD camera parameters from ROS param server
        self.rgb_img_msg.width = self.get_parameter('rgbd_camera/resolution/width').value
        self.rgb_img_msg.height = self.get_parameter('rgbd_camera/resolution/height').value

        self.depth_img_msg.width = self.get_parameter('rgbd_camera/resolution/width').value
        self.depth_img_msg.height = self.get_parameter('rgbd_camera/resolution/height').value
        assert(self.depth_img_msg.width > 5)
        assert(self.depth_img_msg.height > 5)
        cam_frame_id = self.get_parameter('rgbd_camera/frame_id').value
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
        self.rgb_img_msg.header.frame_id = self.get_parameter('rgbd_camera/frame_id').value
        self.depth_img_msg.header.frame_id = self.get_parameter('rgbd_camera/frame_id').value
        # create publishers
        self.pub_camera_state = self.create_publisher(PoseStamped, 'camera/state', 2)
        self.pub_rgb_img = self.create_publisher(Image, 'camera/image_raw', rclpy.qos.qos_profile_system_default)
        self.pub_depth_img = self.create_publisher(Image, 'camera/depth/image_raw', rclpy.qos.qos_profile_system_default)
        self.pub_camera_info = self.create_publisher(CameraInfo, 'camera/info', rclpy.qos.qos_profile_system_default)

        self.rgb_img_msg.encoding = 'rgb8'
        self.rgb_img_msg.is_bigendian = self.get_parameter('rgbd_camera/resolution/is_bigendian').value
        self.rgb_img_msg.step = self.rgb_img_msg.width * 3

        self.depth_img_msg.encoding = self.get_parameter('rgbd_camera/resolution/encoding').value
        self.depth_img_msg.is_bigendian = self.get_parameter('rgbd_camera/resolution/is_bigendian').value
        self.depth_img_msg.step = self.get_parameter('rgbd_camera/resolution/step').value
        assert self.depth_img_msg.encoding in ['32FC1'] + ['mono8', 'mono16', 'bgr8', 'rgb8', 'bgra8', 'rgba8', 'passthrough']
        
        # projection matrix
        self.hfov = self.get_parameter('rgbd_camera/hfov').value
        self.vfov = self.get_parameter('rgbd_camera/vfov').value
        self.near_plane = self.get_parameter('rgbd_camera/near_plane').value
        self.far_plane = self.get_parameter('rgbd_camera/far_plane').value
        self.projection_matrix = self.compute_projection_matrix()

        # use cv_bridge ros to convert cv matrix to ros format
        self.image_bridge = CvBridge()

        # information about simulated camera
        self.camera_info = CameraInfo()

        # used by ROS to handle ROS msgs
        self.camera_info.header = self.depth_img_msg.header

        # resolution of camera
        self.camera_info.height = self.depth_img_msg.width
        self.camera_info.width = self.depth_img_msg.height

        # distortion model
        self.camera_info.distortion_model = 'plumb_bob'

        # distortion parameters (no distortion in simulation)
        self.camera_info.d = [0.0] * 5

        self.fov = (math.radians(self.hfov), math.radians(self.vfov))

        self.image_center = (self.depth_img_msg.width / 2, self.depth_img_msg.height / 2)
        self.focal_lengths = (self.image_center[0] / math.tan(self.fov[0] / 2), self.image_center[1] / math.tan(self.fov[1] / 2))

        # intrinsic camera matrix
        self.camera_info.k = [  self.focal_lengths[0],  0.0,                    self.image_center[0],
                                0.0,                    self.focal_lengths[1],  self.image_center[1],
                                0.0,                    0.0,                    1.0                  ]

        # rectification matrix (identify matrix for monocular camera)
        self.camera_info.r = [  1.0, 0.0, 0.0,
                                0.0, 1.0, 0.0,
                                0.0, 0.0, 1.0]

        # projection matrix
        # first row
        self.camera_info.p[0:3] = self.camera_info.k[0:3]
        self.camera_info.p[3]   = 0.0
        # second row
        self.camera_info.p[4:7] = self.camera_info.k[3:6]
        self.camera_info.p[7]   = 0.0
        # third row
        self.camera_info.p[8:11]= self.camera_info.k[6:9]
        self.camera_info.p[11]  = 0.0

        self.get_logger().info('RGBD camera plugin initialized, camera parameters:')
        self.get_logger().info('  hfov: {}'.format(self.hfov))
        self.get_logger().info('  vfov: {}'.format(self.vfov))
        self.get_logger().info('  fx: {}'.format(self.focal_lengths[0]))
        self.get_logger().info('  fy: {}'.format(self.focal_lengths[1]))
        self.get_logger().info('  near_plane: {}'.format(self.near_plane))
        self.get_logger().info('  far_plane: {}'.format(self.far_plane))
        self.get_logger().info('  resolution: {}x{}'.format(self.depth_img_msg.width, self.depth_img_msg.height))
        self.get_logger().info('  frame_id: {}'.format(self.depth_img_msg.header.frame_id))

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

        if self.depth_img_msg.encoding == 'mono8':
            # Convert from floating-point to 8-bit format and return result
            return (depth_buffer*255).astype(np.uint8)
        elif self.depth_img_msg.encoding == 'mono16':
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

    def flip_image(self, depth_img, img_width, img_height):
        for i in range(int(img_height / 2)):
            upper_row = depth_img[i][0: img_width]
            tmp = upper_row
            upper_row = depth_img[img_height - i - 1][0: img_width]
            depth_img[img_height - i - 1] = tmp

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

        target_point = [0.0, 0.0, 5.0] # expressed w.r.t camera reference frame
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
        pybullet_cam_resp = self.pb.getCameraImage(self.depth_img_msg.width,
                                                    self.depth_img_msg.height,
                                                    view_matrix,
                                                    self.projection_matrix,
                                                    renderer=self.pb.ER_BULLET_HARDWARE_OPENGL
                                                )

        # update msg time stamp
        self.rgb_img_msg.header.stamp = self.get_clock().now().to_msg()
        self.depth_img_msg.header.stamp = self.rgb_img_msg.header.stamp
        self.camera_info.header.stamp = self.depth_img_msg.header.stamp

        # frame extraction function from pybullet
        rgb_frame = self.extract_frame(pybullet_cam_resp)
        rgb_frame = np.flip(rgb_frame, axis=0)
            
        depth_frame = self.extract_depth(pybullet_cam_resp)
        depth_frame = self.calc_true_depth(depth_frame, self.near_plane, self.far_plane)
        depth_frame = np.flip(depth_frame, axis=0)

        # fill pixel data array
        self.rgb_img_msg.data = self.image_bridge.cv2_to_imgmsg(rgb_frame).data
        self.depth_img_msg.data = self.image_bridge.cv2_to_imgmsg(depth_frame, self.depth_img_msg.encoding).data
        # publish camera image to ROS network
        self.pub_camera_state.publish(self.camera_pose_msg)
        self.pub_rgb_img.publish(self.rgb_img_msg)
        self.pub_depth_img.publish(self.depth_img_msg)
        self.pub_camera_info.publish(self.camera_info)