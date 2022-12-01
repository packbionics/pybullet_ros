#!/usr/bin/env python3

from pybullet_ros.pybullet_ros_wrapper import pyBulletRosWrapper

from ament_index_python import get_package_share_path

import yaml
import os

class Environment:
    """
    plugin that is loaded one time only at the beginning
    It is meant to be for you to upload your environment.
    """

    def __init__(self, pybullet_ros_node: pyBulletRosWrapper, **kargs):
        """constructs an environment for the pybullet simulation

        :param pybullet_ros_node: reference to the pybullet ros wrapper
        :type pybullet_ros_node: pyBulletRosWrapper
        """

        # get "import pybullet as pb" and store in self.pb
        self.node = pybullet_ros_node
        self.pb = self.node.pb
        # enable soft body simulation if needed
        if self.node.get_parameter('use_deformable_world').value:
            self.node.get_logger().info('Using deformable world (soft body simulation)')
            self.pb.resetSimulation(self.pb.RESET_USE_DEFORMABLE_WORLD)

    def load_environment(self):
        """
        set gravity, ground plane and load URDF or SDF models as required
        """

        # set gravity
        gravity = self.node.get_parameter('gravity').value # get gravity from param server
        self.pb.setGravity(0, 0, gravity)
        # set floor
        self.pb.loadURDF('plane.urdf')

        # load custom environment
        path = self.node.get_parameter('models_to_load').value
        if os.path.exists(path=path):
            self.load_environment_via_code(path=path)
        else:
            self.node.get_logger().info('File not found: ' + path + '! Skipping environment models')
        
    def load_environment_via_code(self, path):
        """
        This method provides the possibility for the user to define an environment via python code
        """
        self.node.get_logger().warn('loading custom environment via code!')

        # set URDF flags and config file
        fixed_base = self.node.get_parameter('fixed_base').value
        
        # keys to access values from yaml contents dictionary
        PACKAGE_KEY = "Package"         # name of package containing URDF or XACRO
        MODEL_PATH_KEY = "RelPath"      # path of URDF or XACRO relative to package directory
        POSE_KEY = "Pose"               # xyz pose of model
        YAW_KEY = "Yaw"                 # yaw orientation of model

        # open file for reading
        file = open(path, "r")

        model_config = {}

        try:
            # load contents into python dictionary
            model_config = yaml.safe_load(file)
            if model_config == None:
                raise RuntimeError('Model config file is empty or invalid')
        except yaml.YAMLError as exc:
            print(exc)

        urdf_flags = self.pb.URDF_USE_SELF_COLLISION
        if self.node.get_parameter('use_inertia_from_file').value:
            # combining several boolean flags using "or" according to pybullet documentation
            urdf_flags |= self.pb.URDF_USE_INERTIA_FROM_FILE

        for yaml_key in model_config:
            URDF_SUFFIX = '.urdf'
            current_model = model_config[yaml_key]

            # compute absolute path of URDF or XACRO
            package_share = get_package_share_path(current_model[PACKAGE_KEY])
            path = os.path.join(package_share, current_model[MODEL_PATH_KEY])
            path = str(path)

            pose = current_model[POSE_KEY]
            yaw = current_model[YAW_KEY]

            # load robot from URDF model
            # NOTE: self collision enabled by default

            orientation = self.pb.getQuaternionFromEuler([0.0, 0.0, yaw])

            # test urdf file existance
            if not os.path.isfile(path):
                self.node.get_logger().error('file does not exist : ' + path)
                return None
            # ensure urdf is not xacro, but if it is then make urdf file version out of it
            if 'xacro' in path: # generate URDF from XACRO
                # remove xacro from name
                path_end_without_xacro = path.find('.xacro')
                path_without_xacro = path[0: path_end_without_xacro] + URDF_SUFFIX

                # use xacro command to generate URDF from XACRO and return path of new file
                os.system(f'xacro {path} -o {path_without_xacro}')
                path = path_without_xacro

            self.pb.loadURDF(path, basePosition=pose,
                                        baseOrientation=orientation,
                                        useFixedBase=fixed_base, flags=urdf_flags)   

        self.node.get_logger().info('environment models have been loaded')