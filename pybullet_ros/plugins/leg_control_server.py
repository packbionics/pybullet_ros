import rclpy
from rclpy.action import ActionServer

from jetleg_interfaces.action import LegAction

from std_msgs.msg import Float64

from sensor_msgs.msg import JointState

from pybullet_ros.plugins.ros_plugin import RosPlugin

from pybullet_ros.plugins.joint_state_pub import JointStatePub
from pybullet_ros.plugins.link_state_pub import LinkStatePub

import numpy as np

class LegControlServer(RosPlugin):

    def __init__(self, wrapper, pybullet, robot, **kargs):
        super().__init__(wrapper, 'pybullet_ros_leg_control_server', pybullet, robot, automatically_declare_parameters_from_overrides=True)

        # Contains current state information
        self.state_dict = {}

        self.joint_state_pub = JointStatePub(wrapper, pybullet, robot, **kargs)
        self.link_state_pub = LinkStatePub(wrapper, pybullet, robot, **kargs)

        # Names of topics to control leg joints
        pub_topics = [
            'ankle_joint_position_controller/command',
            'knee_joint_position_controller/command'
        ]

        # List of publishers for controlling leg joints
        self.joint_controllers = []

        self.current_state = []

        # Create publishers for controlling leg joints
        for topic_name in pub_topics:
            self.joint_controllers.append(self.create_publisher(Float64, topic_name, rclpy.qos.qos_profile_system_default))

        # Initializes action server to respond to action clients
        self.action_server = ActionServer(self, LegAction, 'leg_control', self.execute_callback)

    def execute_callback(self, goal_handle):

        # Execute requested action
        self.perform_action(np.array(goal_handle.request.action))

        # Mark the action as successful
        goal_handle.succeed()

        # Construct result to send back to action client
        result = LegAction.Result()

        # Retrieve results of the performed action
        result.state = self.retrieve_state()

        result.reward = self.wrapper.sim_time_steps
        result.done = self.has_leg_fallen()

        return result

    def perform_action(self, action):
        delta_pos = 0.1
        idx = np.argmax(action)

        try:
            # if idx == 0:
            #     self.move_knee(delta_pos)
            # elif idx == 1:
            #     self.move_knee(-delta_pos)
            # elif idx == 2:
            #     self.move_ankle(delta_pos)
            # elif idx == 3:
            #     self.move_ankle(-delta_pos)
            if idx == 0:
                self.move_knee(delta_pos * 0.1)
            elif idx == 1:
                self.move_knee(-delta_pos * 0.1)
            elif idx == 2:
                self.move_ankle(delta_pos * 0.1)
            elif idx == 3:
                self.move_ankle(-delta_pos * 0.1)
            elif idx == 4:
                pass
        except KeyError:
            raise KeyError('Invalid name for joint used for state_dict access')

    def move_knee(self, delta_pos):
        current_knee_angle = self.state_dict['knee_joint'][0]
        desired_knee_angle = current_knee_angle + delta_pos

        signal = Float64()
        signal.data = float(desired_knee_angle)

        self.joint_controllers[1].publish(signal)

    def move_ankle(self, delta_pos):
        current_ankle_angle = self.state_dict['ankle_joint'][0]
        desired_ankle_angle = current_ankle_angle + delta_pos

        signal = Float64()
        signal.data = float(desired_ankle_angle)

        self.joint_controllers[0].publish(signal)

    def retrieve_state(self):
        # Update latest joint and link states
        self.joint_state_callback(self.joint_state_pub.get_joint_state())
        self.link_state_callback(self.link_state_pub.get_link_state())

        current_state = []
        for key in self.state_dict:
            for i in self.state_dict[key]:
                current_state.append(i)
        
        return current_state

    def joint_state_callback(self, msg):
        # Reads joint position and joint velocity
        for i in range(len(msg.name)):
            joint_state = [msg.position[i], msg.velocity[i]]
            self.state_dict[msg.name[i]] = joint_state

    def link_state_callback(self, msg):

        # Reads link position and link orientation
        for i in range(len(msg.name)):
            # Reads the XYZ position of a link
            link_pos = [msg.position[i].x, msg.position[i].y, msg.position[i].z]
            # Reads the orientation of a link in quaternion space
            link_orientation = [msg.orientation[i].x, msg.orientation[i].y, msg.orientation[i].z, msg.orientation[i].w]
            
            link_state = link_pos + link_orientation
            self.state_dict[msg.name[i]] = link_state

    def has_leg_fallen(self):
        """
        Checks if the leg has fallen down in simulation
        Returns:
            bool: true if the leg is in a fallen state or false otherwise
        """

        THIGH_HORIZONTAL_THRESHOLD = 0.4

        if self.state_dict['thigh'][2] < THIGH_HORIZONTAL_THRESHOLD:
            return True

        return False

    def get_joint_state(self):
        # setup msg placeholder
        joint_msg = JointState()
        # get joint states
        for joint_index in self.joint_index_name_dic:

            # get joint state from pybullet
            try:
                joint_state = self.pb.getJointState(self.robot, joint_index)
            except Exception as ex:
                raise ConnectionError('An error occurred while trying to access robot joint state. Please ensure robot is fully loaded in the environment.')
            # fill msg
            joint_msg.name.append(self.joint_index_name_dic[joint_index])
            joint_msg.position.append(joint_state[0])
            joint_msg.velocity.append(joint_state[1])
            joint_msg.effort.append(joint_state[3]) # applied effort in last sim step
        # update msg time using ROS time api
        joint_msg.header.stamp = self.get_clock().now().to_msg()

        return joint_msg

    def execute(self):
        pass