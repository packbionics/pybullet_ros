#!/usr/bin/env python3

"""
RGBD camera sensor simulation for pybullet_ros base on pybullet.getCameraImage()
"""

from os.path import exists
import math
from re import S

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from pointcloud_interfaces.msg import ImageCamInfo


class RGBDCamera(Node):
    def __init__(self, pybullet, robot, **kargs):
        super().__init__('pybullet_ros_rgbd_camera')
        self.rate = self.declare_parameter('loop_rate', 80.0).value
        self.timer = self.create_timer(1.0/self.rate, self.execute)
        # get "import pybullet as pb" and store in self.pb
        self.pb = pybullet
        # get robot from parent class
        self.robot = robot
        # create camera info msg placeholder for publication
        self.image_cam_info_msg = ImageCamInfo()
        # create image msg placeholder for publication
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
        # create publisher
        self.pub_image = self.create_publisher(Image, 'camera/depth/image_raw', 10)
        self.image_msg.encoding = self.declare_parameter('rgbd_camera/resolution/encoding', 'rgb8').value
        self.image_msg.is_bigendian = self.declare_parameter('rgbd_camera/resolution/is_bigendian', 0).value
        self.image_msg.step = self.declare_parameter('rgbd_camera/resolution/step', 1920).value
        # projection matrix
        self.hfov = self.declare_parameter('rgbd_camera/hfov', 56.3).value
        self.vfov = self.declare_parameter('rgbd_camera/vfov', 43.7).value
        self.near_plane = self.declare_parameter('rgbd_camera/near_plane', 0.4).value
        self.far_plane = self.declare_parameter('rgbd_camera/far_plane', 8).value
        self.projection_matrix = self.compute_projection_matrix()
        # create publisher
        self.pub_image_camera_info = self.create_publisher(ImageCamInfo, 'camera/image_cam_info', 10)
        self.image_cam_info_msg.hfov = self.hfov
        self.image_cam_info_msg.vfov = self.vfov

        # use cv_bridge ros to convert cv matrix to ros format
        self.image_bridge = CvBridge()

        self.get_logger().info('RGBD camera plugin initialized, camera parameters:')
        self.get_logger().info('  hfov: {}'.format(self.hfov))
        self.get_logger().info('  vfov: {}'.format(self.vfov))
        self.get_logger().info('  near_plane: {}'.format(self.near_plane))
        self.get_logger().info('  far_plane: {}'.format(self.far_plane))
        self.get_logger().info('  resolution: {}x{}'.format(self.image_msg.width, self.image_msg.height))
        self.get_logger().info('  frame_id: {}'.format(self.image_msg.header.frame_id))

    def compute_projection_matrix(self):
        return self.pb.computeProjectionMatrix(
                    left=-math.tan(math.pi * self.hfov / 360.0) * self.near_plane,
                    right=math.tan(math.pi * self.hfov / 360.0) * self.near_plane,
                    bottom=-math.tan(math.pi * self.vfov / 360.0) * self.near_plane,
                    top=math.tan(math.pi * self.vfov / 360.0) * self.near_plane,
                    nearVal=self.near_plane,
                    farVal=self.far_plane)

    def extract_frame(self, camera_image):

        bgr_image = np.zeros((self.image_msg.height, self.image_msg.width, 3))

        camera_image = np.reshape(camera_image[2], (camera_image[1], camera_image[0], 4))

        bgr_image[:, :, 2] =\
            (1 - camera_image[:, :, 3]) * camera_image[:, :, 2] +\
            camera_image[:, :, 3] * camera_image[:, :, 2]

        bgr_image[:, :, 1] =\
            (1 - camera_image[:, :, 3]) * camera_image[:, :, 1] +\
            camera_image[:, :, 3] * camera_image[:, :, 1]

        bgr_image[:, :, 0] =\
            (1 - camera_image[:, :, 3]) * camera_image[:, :, 0] +\
            camera_image[:, :, 3] * camera_image[:, :, 0]

        # return frame
        return bgr_image.astype(np.uint8)

    def extract_depth(self, camera_image):
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
        """
        camera target is a point 5m in front of the robot camera
        This method is used to tranform it to the world reference frame
        NOTE: this method uses pybullet functions and not tf
        """
        target_point = [5.0, 0, 0] # expressed w.r.t camera reference frame
        camera_position = [camera_position[0], camera_position[1], camera_position[2]]
        rm = self.pb.getMatrixFromQuaternion(camera_orientation)
        rotation_matrix = [[rm[0], rm[1], rm[2]],[rm[3], rm[4], rm[5]],[rm[6], rm[7], rm[8]]]
        return np.dot(rotation_matrix, target_point) + camera_position

    def execute(self):
        # get camera pose
        cam_state = self.pb.getLinkState(self.robot, self.pb_camera_link_id)
        # target is a point 5m ahead of the robot camera expressed w.r.t world reference frame
        target = self.compute_camera_target(cam_state[0], cam_state[1])
        view_matrix = self.pb.computeViewMatrix(cam_state[0], target, [0, 0, 1])
        #TODO: compute camera matrix from projection and view matrix
        camera_matrix = np.reshape(self.projection_matrix, (1,16))

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
        self.image_cam_info_msg.image = self.image_msg
        # publish camera image to ROS network
        self.pub_image.publish(self.image_msg)
        self.pub_image_camera_info.publish(self.image_cam_info_msg)
        self.get_logger().info('Published image')
