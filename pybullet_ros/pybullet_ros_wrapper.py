#!/usr/bin/env python3

import importlib
import os

import pybullet_data
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from std_srvs.srv import Empty

from utils import ModelLoader
from utils import urdf_from_xacro

class pyBulletRosWrapper(Node):
    """ROS wrapper class for pybullet simulator

    Attributes:
        executor (MultiThreadedExecutor): handles the threads running the plugins
        pb (ModuleType): used to access Pybullet API
        environment (_type_): used to define simulation environment
        urdf_flags (int): used to process loaded URDF data
        robot (int): first robot loaded
        connected_to_physics_server (bool): identifies if the process is attached to an instance of Pybullet simulation
        rev_joint_index_name_dic (dict): dictionary mapping joint index to joint name for revolute joints
        prismatic_joint_index_name_dic (dict): dictionary mapping joint index to joint name for prismatic joints
        fixed_joint_index_name_dic (dict): dictionary mapping joint index to joint name for fixed joints
        link_names_to_ids_dic (dict): dictionary mapping link names to link indices
        plugins (list): list of plugins to load
        timer (Timer): used as a handle for the timer running the main loop
        loop_rate (float): determines the rate of the main loop
        is_gui_needed (bool): determines if the simulation starts with a GUI
        pause_simulation (bool): identifies if the simulation is paused or not
        env_plugin (str): name of the environment plugin
        plugin_import_prefix (str): prefix used to import plugins
    """

    def __init__(self):
        """Starts Pybullet engine and runs plugins in parallel"""

        super().__init__('pybullet_ros', automatically_declare_parameters_from_overrides=True)

        # declare handler for multi-threaded processes
        ex = MultiThreadedExecutor()
        self.executor = ex

        # import pybullet
        self.pb = importlib.import_module('pybullet')

        self.pause_simulation = True

        # load parameters
        self.init_parameters()
        # print pybullet stuff in blue 
        print('\033[34m')
        # start physics engine client
        self.start_engine(gui=self.is_gui_needed) # we dont need to store the physics client for now...
        # get pybullet path in your system and store it internally for future use, e.g. to set floor
        self.pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.environment = getattr(importlib.import_module(f'{self.plugin_import_prefix}.{self.env_plugin}'), 'Environment')(self)
        # create ROS 2 services
        self.init_services()
        # load robot URDF model, set gravity, and ground plane
        self.urdf_flags = self.init_environment()
        self.robot = self.init_pybullet_models()
        self.connected_to_physics_server = None
        if not self.robot:
            self.connected_to_physics_server = False
            return # Error while loading urdf file
        else:
            self.connected_to_physics_server = True
        # get all revolute joint names and pybullet index
        self.rev_joint_index_name_dic, self.prismatic_joint_index_name_dic, self.fixed_joint_index_name_dic, self.link_names_to_ids_dic = self.get_properties()
        # import plugins dynamically
        self.plugins = []
        plugins = self.get_parameter('plugins').value
        if not plugins or plugins == ['']:
            self.get_logger().warn('No plugins found, forgot to set param plugins?')
        # return to normal shell color
        print('\033[0m')
        # load plugins
        self.init_plugins(plugins)

        # track number of timesteps per simulation
        self.sim_time_steps = 0

        self.get_logger().info('pybullet ROS wrapper initialized')
        self.timer = self.create_timer(1.0 / self.loop_rate, self.wrapper_callback)

        self.executor.add_node(self)

        self.pause_simulation = False

        try:
            self.executor.spin()
        except Exception as e:
            self.get_logger().error(str(repr(e)))
        finally:
            self.executor.shutdown()
            self.destroy_node()
            for node in self.plugins:
                node.destroy_node()

    def wrapper_callback(self):
        """loop for running physcs simulation"""

        self.pb.stepSimulation()
        self.sim_time_steps += 1
        if not self.connected_to_physics_server:
            self.pb.disconnect()

    def get_properties(self):
        """construct 3 dictionaries:
        - joint index to joint name x2 (1 for revolute, 1 for fixed joints)
        - link name to link index dictionary

        Returns:
            tuple: tuple containing 3 dictionaries
        """

        rev_joint_index_name_dic = {}
        fixed_joint_index_name_dic = {}
        prismatic_joint_index_name_dic = {}
        link_names_to_ids_dic = {}
        for joint_index in range(0, self.pb.getNumJoints(self.robot)):
            info = self.pb.getJointInfo(self.robot, joint_index)
            # build a dictionary of link names to ids
            link_names_to_ids_dic[info[12].decode('utf-8')] = joint_index
            # ensure we are dealing with a revolute joint
            if info[2] == self.pb.JOINT_REVOLUTE:
                # insert key, value in dictionary (joint index, joint name)
                rev_joint_index_name_dic[joint_index] = info[1].decode('utf-8') # info[1] refers to joint name
            elif info[2] == self.pb.JOINT_FIXED:
                # insert key, value in dictionary (joint index, joint name)
                fixed_joint_index_name_dic[joint_index] = info[1].decode('utf-8') # info[1] refers to joint name
            elif info[2] == self.pb.JOINT_PRISMATIC:
                prismatic_joint_index_name_dic[joint_index] = info[1].decode('utf-8') # info[1] refers to joint name
        return rev_joint_index_name_dic, prismatic_joint_index_name_dic, fixed_joint_index_name_dic, link_names_to_ids_dic

    def start_engine(self, gui=True):
        """Starts Pybullet physics engine

        Args:
            gui (bool, optional): Determines if PyBullet starts with or without a GUI. Defaults to True.

        Returns:
            int: a physics client id
        """

        if(gui):
            # start simulation with gui
            self.get_logger().info('Running pybullet with gui')
            self.get_logger().info('-------------------------')
            gui_options = self.get_parameter('gui_options').value # e.g. to maximize screen: options="--width=2560 --height=1440"
            return self.pb.connect(self.pb.GUI, options=gui_options)
        else:
            # start simulation without gui (non-graphical version)
            self.get_logger().info('Running pybullet without gui')
            # hide console output from pybullet
            self.get_logger().info('-------------------------')
            return self.pb.connect(self.pb.DIRECT)

    def load_urdf(self, path, pose, yaw, urdf_flags, fixed_base=False):
        """Loads a single URDF or XACRO robot

        Args:
            path (str): Path of the URDF or XACRO
            pose (list): XYZ coordinates of the robot
            yaw (float): Yaw of the robot
            urdf_flags (int): flags for processing URDF data
            fixed_base (bool, optional): force the base of the loaded object to be static. Defaults to False.

        Returns:
            int: a body unique id, a non-negative integer value
        """

        orientation = self.pb.getQuaternionFromEuler([0.0, 0.0, yaw])

        # test urdf file existance
        if not os.path.isfile(path):
            self.get_logger().error('file does not exist : ' + path)
            return None
        # ensure urdf is not xacro, but if it is then make urdf file version out of it
        if 'xacro' in path:
            # generate URDF from XACRO
            path = urdf_from_xacro(path)

        self.get_logger().info('loading urdf from file: ' + str(path))
        return self.pb.loadURDF(path, basePosition=pose,
                                        baseOrientation=orientation,
                                        useFixedBase=fixed_base, flags=urdf_flags)  

    def init_parameters(self):
        """loads ROS 2 parameters"""

        # get from param server the frequency at which to run the simulation
        self.loop_rate = self.get_parameter('loop_rate').value  
        self.get_logger().info('Loop rate: {}'.format(self.loop_rate))
        # query from param server if gui is needed
        self.is_gui_needed = self.get_parameter('pybullet_gui').value
        # get from param server if user wants to pause simulation at startup
        self.pause_simulation = self.get_parameter('pause_simulation').value
        # create object of environment class for later use
        self.env_plugin = self.get_parameter('environment').value # default : plugins/environment.py
        self.plugin_import_prefix = self.get_parameter('plugin_import_prefix').value 

    def init_services(self):
        """loads ROS 2 services"""

        # setup service to restart simulation
        self.create_service(Empty, 'reset_simulation', self.handle_reset_simulation)
        # setup services for pausing/unpausing simulation
        self.create_service(Empty, 'pause_physics', self.handle_pause_physics)
        self.create_service(Empty, 'unpause_physics', self.handle_unpause_physics)

    def init_pybullet_models(self):
        """load URDF models

        Returns:
            int: unique body id of the first loaded robot
        """

        # load environment, set URDF flags
        fixed_base = self.get_parameter('fixed_base').value
        model_loader_path = self.get_parameter('models_to_load').value
        model_loader = ModelLoader(model_loader_path)
        self.get_logger().info('attempting to load urdf models...')
        model_id_list = []
        for model in model_loader.models:
            # load robot from URDF model
            # NOTE: self collision enabled by default
            loaded_urdf = self.load_urdf(model[model_loader.abs_path_key], model[model_loader.pose_key], model[model_loader.yaw_key], self.urdf_flags, fixed_base)
            if loaded_urdf == None:
                self.get_logger().error('file does not exist : ' + model[0])
                rclpy.shutdown()
            model_id_list.append(loaded_urdf)
        self.get_logger().info('models have been loaded')
        return model_id_list[0]

    def init_environment(self):
        """set gravity, ground plane and environment

        Returns:
            int: flags for processing URDF data
        """

        # load environment
        self.get_logger().info('loading environment')
        self.environment.load_environment()
        # set no realtime simulation, NOTE: no need to stepSimulation if setRealTimeSimulation is set to 1
        self.pb.setRealTimeSimulation(0) # NOTE: does not currently work with effort controller, thats why is left as 0
        # user decides if inertia is computed automatically by pybullet or custom
        if self.get_parameter('use_inertia_from_file').value:
            # combining several boolean flags using "or" according to pybullet documentation
            urdf_flags = self.pb.URDF_USE_INERTIA_FROM_FILE | self.pb.URDF_USE_SELF_COLLISION
        else:
            urdf_flags = self.pb.URDF_USE_SELF_COLLISION

        return urdf_flags

    def init_plugins(self, plugins):
        """finds plugins and sets them up to be executed in multiple threads

        Args:
            plugins (list): list of strings representing plugins to load
        """

        for plugin in plugins:
            module_, class_ = plugin.split(':')
            params_ = {'module': module_, 'class': class_}
            self.get_logger().info('loading plugin: {} class from {}'.format(class_, module_))
            # create object of the imported file class
            obj = getattr(importlib.import_module(module_), class_)(self, self.pb, self.robot,
                          rev_joints=self.rev_joint_index_name_dic,
                          prism_joints=self.prismatic_joint_index_name_dic,
                          fixed_joints=self.fixed_joint_index_name_dic,
                          link_ids=self.link_names_to_ids_dic,
                          **params_)
            # store objects in member variable for future use
            self.plugins.append(obj)
            self.executor.add_node(obj)

    def handle_reset_simulation(self, req, resp):
        """Callback to handle the service offered by this node to reset the simulation

        Args:
            req (SetBool.Request): an empty data structure repreenting request by the client
            resp (SetBool.Response): an empty data structure representing response to the client

        Returns:
            response: an empty data structure representing response to the client
        """

        self.get_logger().info('resetting simulation now')
        # pause simulation to prevent reading joint values with an empty world
        self.pause_simulation = True
        # remove all objects from the world and reset the world to initial conditions
        self.pb.resetSimulation()
        # reset simulation timestep tracker
        self.sim_time_steps = 0
        # set gravity and floor
        self.urdf_flags = self.init_environment()
        # load URDF model again
        self.init_pybullet_models()
        # resume simulation control cycle now that a new robot is in place
        self.pause_simulation = False
        return resp

    def handle_pause_physics(self, req):
        """pause simulation, raise flag to prevent pybullet to execute self.pb.stepSimulation()

        Args:
            req (Empty): an empty data structure

        Returns:
            list: empty list
        """

        self.get_logger().info('pausing simulation')
        self.pause_simulation = False
        return []

    def handle_unpause_physics(self, req):
        """unpause simulation, lower flag to allow pybullet to execute self.pb.stepSimulation()

        Args:
            req (Empty): an empty data structure

        Returns:
            list: empty list
        """

        self.get_logger().info('unpausing simulation')
        self.pause_simulation = True
        return []

    def pause_simulation_function(self):
        """pauses the PyBullet engine simulation

        Returns:
            bool: identifies if the simulation is paused or not
        """

        return self.pause_simulation

def main():
    try:
        rclpy.init()
        pyBulletRosWrapper()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
