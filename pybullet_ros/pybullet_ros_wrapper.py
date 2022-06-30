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
    """ROS wrapper class for pybullet simulator"""
    def __init__(self):

        super().__init__('pybullet_ros', automatically_declare_parameters_from_overrides=True)

        # declare handler for multi-threaded processes
        ex = MultiThreadedExecutor()
        self.executor = ex

        # import pybullet
        self.pb = importlib.import_module('pybullet')

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

        self.get_logger().info('pybullet ROS wrapper initialized')
        self.timer = self.create_timer(1.0 / self.loop_rate, self.wrapper_callback)

        self.executor.add_node(self)

        try:
            self.executor.spin()
        #except Exception as e:
        #    self.get_logger().error(traceback.format_exc())
        finally:
            self.executor.shutdown()
            self.destroy_node()
            for node in self.plugins:
                node.destroy_node()

    def wrapper_callback(self):
        self.pb.stepSimulation()
        if not self.connected_to_physics_server:
            self.pb.disconnect()

    def get_properties(self):
        """
        construct 3 dictionaries:
        - joint index to joint name x2 (1 for revolute, 1 for fixed joints)
        - link name to link index dictionary
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
        """start physics engine (client) with or without gui"""
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
        # setup service to restart simulation
        self.create_service(Empty, 'reset_simulation', self.handle_reset_simulation)
        # setup services for pausing/unpausing simulation
        self.create_service(Empty, 'pause_physics', self.handle_pause_physics)
        self.create_service(Empty, 'unpause_physics', self.handle_unpause_physics)

    def init_pybullet_models(self):
        """load URDF models"""
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
        """set gravity, ground plane and environment"""
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
        for plugin in plugins:
            module_, class_ = plugin.split(':')
            params_ = {'module': module_, 'class': class_}
            self.get_logger().info('loading plugin: {} class from {}'.format(class_, module_))
            # create object of the imported file class
            obj = getattr(importlib.import_module(module_), class_)(self.pb, self.robot,
                          rev_joints=self.rev_joint_index_name_dic,
                          prism_joints=self.prismatic_joint_index_name_dic,
                          fixed_joints=self.fixed_joint_index_name_dic,
                          link_ids=self.link_names_to_ids_dic,
                          **params_)
            # store objects in member variable for future use
            self.plugins.append(obj)
            self.executor.add_node(obj)

    def handle_reset_simulation(self, req):
        """Callback to handle the service offered by this node to reset the simulation"""
        self.get_logger().info('resetting simulation now')
        # pause simulation to prevent reading joint values with an empty world
        self.pause_simulation = True
        # remove all objects from the world and reset the world to initial conditions
        self.pb.resetSimulation()
        # load URDF model again, set gravity and floor
        self.init_pybullet_models()
        # resume simulation control cycle now that a new robot is in place
        self.pause_simulation = False
        return []

    def handle_pause_physics(self, req):
        """pause simulation, raise flag to prevent pybullet to execute self.pb.stepSimulation()"""
        self.get_logger().info('pausing simulation')
        self.pause_simulation = False
        return []

    def handle_unpause_physics(self, req):
        """unpause simulation, lower flag to allow pybullet to execute self.pb.stepSimulation()"""
        self.get_logger().info('unpausing simulation')
        self.pause_simulation = True
        return []

    def pause_simulation_function(self):
        return self.pause_simulation

def main():
    try:
        rclpy.init()
        pyBulletRosWrapper()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
