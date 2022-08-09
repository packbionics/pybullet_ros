#!/usr/bin/env python3

"""
position, velocity and effort control for all revolute joints on the robot
"""

from rclpy.node import Node
from std_msgs.msg import Float64

# NOTE: 2 classes are implemented here, scroll down to the next class (Control) to see the plugin!

class PveControl:
    """helper class to receive position, velocity or effort (pve) control commands

    Attributes:
        node (Node): ROS 2 node
        subscription (Subscription): subscription used to receive input commands from ROS 2 topic
        cmd (float): value of the last received command
        data_available (bool): determines if there is more data to process
        joint_index (int): index of joint used by Pybullet
        joint_name (str): name of joint
    """

    def __init__(self, node, joint_index, joint_name, controller_type):
        """constructor

        Assumes joint_name is unique, creates multiple subscribers to receive commands
        joint_index - stores an integer joint identifier
        joint_name - string with the name of the joint as described in urdf model
        controller_type - position, velocity or effort

        Args:
            node (Node): ROS 2 node
            joint_index (int): index of joint used by Pybullet
            joint_name (str): name of joint
            controller_type (str): type of control used. Possible values may be 'position', 'velocity', or 'effort'
        """

        assert(controller_type in ['position', 'velocity', 'effort'])
        self.node = node
        self.node.get_logger().info("subscribing to "+ joint_name + '_' + controller_type + '_controller/command')
        self.subscription = self.node.create_subscription(
            Float64,
            joint_name + '_' + controller_type + '_controller/command',
            self.pve_controlCB, 
            1
        )
        self.cmd = 0.0
        self.data_available = False
        self.joint_index = joint_index
        self.joint_name = joint_name

    def pve_controlCB(self, msg):
        """position, velocity or effort callback

        Args:
            msg (Float64): the msg passed by the ROS network via topic publication
        """

        self.data_available = True
        self.cmd = msg.data

    def get_last_cmd(self):
        """method to fetch the last received command

        Returns:
            float: value of the last received command
        """

        self.data_available = False
        return self.cmd

    def get_is_data_available(self):
        """method to retrieve flag to indicate that a command has been received

        Returns:
            bool: determines if there is more data to process
        """

        return self.data_available

    def get_joint_name(self):
        """Unused method provided for completeness (pybullet works based upon joint index, not names)

        Returns:
            str: name of associated joint
        """
        
        return self.joint_name

    def get_joint_index(self):
        """method used to retrieve the joint int index that this class points to

        Returns:
            int: index of associated joint
        """

        return self.joint_index

# plugin is implemented below
class Control(Node):
    """position, velocity and effort control for all revolute and prismatic joints on the robot

    Attributes:
        rate (float): determines the rate of execute()
        timer (Timer): handles the main loop of the plugin
        pb (ModuleType): used to access Pybullet API
        robot (int): id for the first loaded robot
        position_joint_commands (list): contains history of position commands
        velocity_joint_commands (list): contains history of velocity commands
        effort_joint_commands (list): contains history of effort commands
        max_effort (float): maximum effort to process for any type of control
        joint_index_name_dic (dict): dictionary of joint index to joint names
        pc_subscribers (list): list of position control command subscribers
        vc_subscribers (list): list of velocity control command subscribers
        ec_subscribers (list): list of effort control command subscribers
        joint_indices (list): list of joint indices
    """

    def __init__(self, pybullet, robot, **kargs):
        """constructor

        Args:
            pybullet (ModuleType): used to access Pybullet API
            robot (int): first robot loaded
        """

        super().__init__('pybullet_ros_control',
                         automatically_declare_parameters_from_overrides=True)
        self.rate = self.get_parameter('loop_rate').value
        self.timer = self.create_timer(1.0/self.rate, self.execute)

        # get "import pybullet as pb" and store in self.pb
        self.pb = pybullet
        # get robot from parent class
        self.robot = robot
        # lists to recall last received command (useful when controlling multiple joints)
        self.position_joint_commands = []
        self.velocity_joint_commands = []
        self.effort_joint_commands = []
        # this parameter will be set for all robot joints
        self.max_effort = self.get_parameter('max_effort').value
        self.get_logger().info("Control: Max effort={}".format(self.max_effort))

        # get joints names and store them in dictionary, combine both revolute and prismatic dic
        for k in kargs.keys():
            self.get_logger().info(k)
        self.joint_index_name_dic = {**kargs['rev_joints'], **kargs['prism_joints']}
        # setup subscribers
        self.pc_subscribers = []
        self.vc_subscribers = []
        self.ec_subscribers = []
        self.joint_indices = []
        # revolute joints - joint position, velocity and effort control command individual subscribers
        for joint_index in self.joint_index_name_dic:
            self.position_joint_commands.append(0.0)
            self.velocity_joint_commands.append(0.0)
            self.effort_joint_commands.append(0.0)
            # get joint name from dictionary, is used for naming the subscriber
            joint_name = self.joint_index_name_dic[joint_index]
            # create list of joints for later use in pve_ctrl_cmd(...)
            self.joint_indices.append(joint_index)
            # create position control object
            self.pc_subscribers.append(PveControl(self, joint_index, joint_name, 'position'))
            # create position control object
            self.vc_subscribers.append(PveControl(self, joint_index, joint_name, 'velocity'))
            # create position control object
            self.ec_subscribers.append(PveControl(self, joint_index, joint_name, 'effort'))

    def execute(self):
        """this function gets called from pybullet ros main update loop

        check if user has commanded a joint
        and forward the request to pybullet
        """
        
        self.position_ctrl_joint_indices = []
        self.velocity_ctrl_joint_indices = []
        self.effort_ctrl_joint_indices = []
        
        self.position_joint_commands = []
        self.velocity_joint_commands = []
        self.effort_joint_commands = []
        
        # the max force to apply to the joint
        self.pos_force_commands = []
        self.vel_force_commands = []

        # create arrays for motor control, position is prioritized first, velocity second, effort third
        for index, subscriber in enumerate(self.pc_subscribers):
            if subscriber.get_is_data_available():
                self.position_joint_commands.append(subscriber.get_last_cmd())
                self.position_ctrl_joint_indices.append(self.joint_indices[index])
                self.pos_force_commands.append(self.max_effort)
 
        for index, subscriber in enumerate(self.vc_subscribers):
            if subscriber.get_is_data_available() and index not in self.position_ctrl_joint_indices:
                self.velocity_joint_commands.append(subscriber.get_last_cmd())
                self.velocity_ctrl_joint_indices.append(self.joint_indices[index])
                self.vel_force_commands.append(self.max_effort)
                
        for index, subscriber in enumerate(self.ec_subscribers):
            if (subscriber.get_is_data_available() 
                and index not in self.position_ctrl_joint_indices 
                and index not in self.velocity_ctrl_joint_indices):
                
                self.effort_joint_commands.append(subscriber.get_last_cmd())
                self.effort_ctrl_joint_indices.append(self.joint_indices[index])
                
        # forward commands to pybullet
        
        # position control
        self.pb.setJointMotorControlArray(bodyUniqueId=self.robot, jointIndices=self.position_ctrl_joint_indices,
                                    controlMode=self.pb.POSITION_CONTROL, targetPositions=self.position_joint_commands, forces=self.pos_force_commands)
        # velocity control
        self.pb.setJointMotorControlArray(bodyUniqueId=self.robot, jointIndices=self.velocity_ctrl_joint_indices,
                                    controlMode=self.pb.VELOCITY_CONTROL, targetVelocities=self.velocity_joint_commands, forces=self.vel_force_commands)
        # effort control
        self.pb.setJointMotorControlArray(bodyUniqueId=self.robot, jointIndices=self.effort_ctrl_joint_indices,
                                    controlMode=self.pb.POSITION_CONTROL, forces=[0.0] * len(self.effort_joint_commands))
        self.pb.setJointMotorControlArray(bodyUniqueId=self.robot, jointIndices=self.effort_ctrl_joint_indices,
                                    controlMode=self.pb.TORQUE_CONTROL, forces=self.effort_joint_commands)
