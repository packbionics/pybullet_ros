#!/usr/bin/env python3

import importlib
import yaml
import os

from ament_index_python import get_package_share_path

import pybullet_data
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from std_srvs.srv import Empty

class pyBulletRosWrapper(Node):
    """ROS wrapper class for pybullet simulator"""

    def __init__(self):
        """Starts Pybullet engine and runs plugins in parallel"""

        # Registers class as a ROS Node class
        super().__init__('pybullet_ros')

        # declare handler for multi-threaded processes
        ex = MultiThreadedExecutor()
        self.executor = ex

        # import pybullet
        self.pb = importlib.import_module('pybullet')

        # tracks if the simulation is paused or resetting
        self.pause_simulation = True

        # declare ros 2 parameters
        self.declare_parameter('loop_rate', 120.0)
        self.declare_parameter('gravity', -9.81)
        self.declare_parameter('pause_simulation', False)
        self.declare_parameter('pybullet_gui', False)
        self.declare_parameter('gui_options', '')
        self.declare_parameter('use_deformable_world', False)
        self.declare_parameter('environment', 'environment')
        self.declare_parameter('plugin_import_prefix', 'pybullet_ros.plugins')
        self.declare_parameter('use_inertia_from_file', False)
        self.declare_parameter('fixed_base', False)
        self.declare_parameter('plugins', [''])
        self.declare_parameter('models_to_load', '')
        self.declare_parameter('robot_path', str(os.path.join(get_package_share_path('pybullet_ros'), 'common/test/urdf/r2d2.urdf.xacro')))
        self.declare_parameter('robot_pose_x', 0.0)
        self.declare_parameter('robot_pose_y', 0.0)
        self.declare_parameter('robot_pose_z', 1.0)
        self.declare_parameter('robot_pose_yaw', 0.0)

        # get from param server the frequency at which to run the simulation
        self.loop_rate = self.get_parameter('loop_rate').value  
        self.get_logger().info('Loop rate: {}'.format(self.loop_rate))

        # get from param server if user wants to pause simulation at startup
        self.pause_simulation = self.get_parameter('pause_simulation').value

        # create object of environment class for later use
        self.env_plugin = self.get_parameter('environment').value # default : plugins/environment.py

        # path to plugins to use for import
        self.plugin_import_prefix = self.get_parameter('plugin_import_prefix').value 

        # print pybullet stuff in blue 
        print('\033[34m')

        # query from param server if gui should be displayed
        is_gui_needed = self.get_parameter('pybullet_gui').value
        # start physics engine client
        self.start_engine(gui=is_gui_needed)

        # get pybullet path in your system and store it internally for future use, e.g. to set floor
        self.pb.setAdditionalSearchPath(pybullet_data.getDataPath())

        # Load the environment plugin for defining the environment of the simulation
        self.environment = getattr(importlib.import_module(f'{self.plugin_import_prefix}.{self.env_plugin}'), 'Environment')(self)
        
        # create ROS 2 services for pausing, resuming, resetting the simulation, etc.
        self.init_services()

        # load robot URDF model, set gravity, and ground plane
        self.init_environment()
        self.robot = self.init_pybullet_robot()
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
        if not plugins:
            self.get_logger().warn('No plugins found, forgot to set param plugins?')

        # return to normal shell color
        print('\033[0m')

        # load plugins
        self.init_plugins(plugins)

        # track number of timesteps per simulation
        self.sim_time_steps = 0

        # start pybullet simulation
        self.get_logger().info('pybullet ROS wrapper initialized')
        self.timer = self.create_timer(1.0 / self.loop_rate, self.wrapper_callback)

        self.executor.add_node(self)

        self.pause_simulation = False

        try:
            self.executor.spin()
        except KeyboardInterrupt as e:
            pass

        # remove connection from physics server
        self.pb.disconnect()

        # stop plugin and callback executions
        self.executor.shutdown()

        # release node resources of plugins
        # for node in self.plugins:
        #     node.destroy_node()

        # release resources of the pybullet ros wrapper
        # self.destroy_node()

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

        :return: tuple containing 3 dictionaries
        :rtype: tuple
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

        :param gui: determines if PyBullet starts with or without a GUI, defaults to True
        :type gui: bool, optional
        :return: a physics client id
        :rtype: int
        """

        connection_mode = self.pb.GUI
        gui_options = self.get_parameter('gui_options').value # e.g. to maximize screen: options="--width=2560 --height=1440"

        if(gui):
            # start simulation with gui
            self.get_logger().info('Running pybullet with gui')
            self.get_logger().info('-------------------------')
        else:
            connection_mode = self.pb.DIRECT
            # start simulation without gui (non-graphical version)
            self.get_logger().info('Running pybullet without gui')
            # hide console output from pybullet
            self.get_logger().info('-------------------------')

        # return physics client id
        return self.pb.connect(connection_mode, options=gui_options)

    def load_robot(self, path: str, pose: list, yaw: float, urdf_flags: int, fixed_base=False):   
        """Loads a single URDF or XACRO robot

        :param path: Path of the URDF or XACRO
        :type path: str
        :param pose: XYZ coordinates of the robot
        :type pose: list
        :param yaw: Yaw of the robot
        :type yaw: float
        :param urdf_flags: flags for processing URDF data
        :type urdf_flags: int
        :param fixed_base: force the base of the loaded object to be static, defaults to False
        :type fixed_base: bool, optional
        :return: int
        :rtype: a body unique id, a non-negative integer value
        """        

        URDF_SUFFIX = '.urdf'

        orientation = self.pb.getQuaternionFromEuler([0.0, 0.0, yaw])

        # test urdf file existance
        if not os.path.isfile(path):
            self.get_logger().error('file does not exist : ' + path)
            return None
        # ensure urdf is not xacro, but if it is then make urdf file version out of it
        if 'xacro' in path: # generate URDF from XACRO
            # remove xacro from name
            path_end_without_xacro = path.find('.xacro')
            path_without_xacro = path[0: path_end_without_xacro] + URDF_SUFFIX

            # use xacro command to generate URDF from XACRO and return path of new file
            os.system(f'xacro {path} -o {path_without_xacro}')
            path = path_without_xacro 

        return self.pb.loadURDF(path, basePosition=pose,
                                        baseOrientation=orientation,
                                        useFixedBase=fixed_base, flags=urdf_flags)  

    def init_services(self):
        """loads ROS 2 services"""

        # setup service to restart simulation
        self.create_service(Empty, 'reset_simulation', self.handle_reset_simulation)
        # setup services for pausing/unpausing simulation
        self.create_service(Empty, 'pause_physics', self.handle_pause_physics)
        self.create_service(Empty, 'unpause_physics', self.handle_unpause_physics)

    def init_pybullet_robot(self):
        """load URDF models

        :return: unique body id of the robot
        :rtype: int
        """        

        # load environment
        fixed_base = self.get_parameter('fixed_base').value

        # load path to robot
        path = self.get_parameter('robot_path').value

        # load world config of robot
        pose_x = self.get_parameter('robot_pose_x').value
        pose_y = self.get_parameter('robot_pose_y').value
        pose_z = self.get_parameter('robot_pose_z').value

        pose_yaw = self.get_parameter('robot_pose_yaw').value

        urdf_flags = self.pb.URDF_USE_SELF_COLLISION
        if self.get_parameter('use_inertia_from_file').value:
            # combining several boolean flags using "or" according to pybullet documentation
            urdf_flags |= self.pb.URDF_USE_INERTIA_FROM_FILE
        
        return self.load_robot(path, [pose_x, pose_y, pose_z], pose_yaw, urdf_flags, fixed_base)

    def init_environment(self):
        """set gravity, ground plane and environment"""        

        # load environment
        self.get_logger().info('loading environment')
        self.environment.load_environment()
        # set no realtime simulation, NOTE: no need to stepSimulation if setRealTimeSimulation is set to 1
        self.pb.setRealTimeSimulation(0) # NOTE: does not currently work with effort controller, thats why is left as 0
        # user decides if inertia is computed automatically by pybullet or custom

    def init_plugins(self, plugins: list):
        """finds plugins and sets them up to be executed in multiple threads

        :param plugins: list of strings representing plugins to load
        :type plugins: list
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

    def handle_reset_simulation(self, req: Empty.Request, resp: Empty.Response):
        """Callback to handle the service offered by this node to reset the simulation

        :param req: an empty data structure repreenting request by the client
        :type req: Empty.Request
        :param resp: an empty data structure representing response to the client
        :type resp: Empty.Response
        :return: an empty data structure representing response to the client
        :rtype: Empty.Response
        """        

        self.get_logger().info('resetting simulation now')
        # pause simulation to prevent reading joint values with an empty world
        self.pause_simulation = True
        # remove all objects from the world and reset the world to initial conditions
        self.pb.resetSimulation()
        # reset simulation timestep tracker
        self.sim_time_steps = 0
        # set gravity and floor
        self.init_environment()
        # load URDF model again
        self.init_pybullet_robot()
        # resume simulation control cycle now that a new robot is in place
        self.pause_simulation = False
        return resp

    def handle_pause_physics(self, req: Empty.Request, resp: Empty.Response):
        """pause simulation, raise flag to prevent pybullet to execute self.pb.stepSimulation()

        :param req: an empty data structure
        :type req: Empty.Request
        :param resp: an empty data structure representing response to the client
        :type resp: Empty.Response
        :raises flag: _description_
        :return: an empty data structure representing response to the client
        :rtype: Empty.Response
        """        

        self.get_logger().info('pausing simulation')
        self.pause_simulation = False
        return resp

    def handle_unpause_physics(self, req: Empty.Request, resp: Empty.Response):
        """unpause simulation, lower flag to allow pybullet to execute self.pb.stepSimulation()

        :param req: an empty data structure
        :type req: Empty.Request
        :param resp: an empty data structure representing response to the client
        :type resp: Empty.Response
        :return: an empty data structure representing response to the client
        :rtype: Empty.Response
        """

        self.get_logger().info('unpausing simulation')
        self.pause_simulation = True
        return resp


def main():
    rclpy.init()

    try:
        pyBulletRosWrapper()
    except Exception as ex:
        print(repr(ex))

if __name__ == '__main__':
    main()
