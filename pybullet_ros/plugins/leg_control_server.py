from sympy import false
import rclpy
from rclpy.action import ActionServer

from jetleg_interfaces.action import LegAction
from jetleg_interfaces.msg import LinkState

from std_msgs.msg import Float64
from std_srvs.srv import Empty

from sensor_msgs.msg import JointState

from pybullet_ros.plugins.ros_plugin import RosPlugin

import numpy as np

class LegControlServer(RosPlugin):

    def __init__(self, wrapper, pybullet, robot, **kargs):
        super().__init__(wrapper, 'pybullet_ros_leg_control_server', pybullet, robot, automatically_declare_parameters_from_overrides=True)

        # Contains current state information
        self.state_dict = {}

        # Initialize JointState subscription to read joint states for action feedback and result
        self.joint_state_sub = self.create_subscription(JointState, 'joint_states', self.joint_state_callback, rclpy.qos.qos_profile_system_default)

        # Initialize LinkState subscription to read Poselink states for action feedback and result
        self.link_state_sub = self.create_subscription(LinkState, 'link_states', self.link_state_callback, rclpy.qos.qos_profile_system_default)

        # Initialize service client to reset simulation after the leg falls down
        self.reset_sim_client = self.create_client(Empty, 'reset_simulation')

        # Names of topics to control leg joints
        pub_topics = [
            'ankle_joint_position_controller/command',
            'knee_joint_position_controller/command'
        ]

        # List of publishers for controlling leg joints
        self.joint_controllers = []

        # Create publishers for controlling leg joints
        for topic_name in pub_topics:
            self.joint_controllers.append(self.create_publisher(Float64, topic_name, rclpy.qos.qos_profile_system_default))

        # Initializes action server to respond to action clients
        self.action_server = ActionServer(self, LegAction, 'leg_control', self.execute_callback)

    def execute_callback(self, goal_handle):
        # self.get_logger().info('Executing goal...')

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
            if idx == 0:
                self.move_knee(delta_pos)
            elif idx == 1:
                self.move_knee(-delta_pos)
            elif idx == 2:
                self.move_ankle(delta_pos)
            elif idx == 3:
                self.move_ankle(-delta_pos)
        except KeyError:
            self.get_logger().error('Invalid name for joint used for state_dict access')
            exit(1)

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

    def reset_sim(self):
        req = Empty()
        
        self.future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)

    def has_leg_fallen(self):
        """
        Checks if the leg has fallen down in simulation
        Returns:
            bool: true if the leg is in a fallen state or false otherwise
        """

        

        return False

    def execute(self):
        pass