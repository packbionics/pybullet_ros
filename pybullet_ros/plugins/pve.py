from std_msgs.msg import Float64

class PveControl:
    """helper class to receive position, velocity or effort (pve) control commands"""

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