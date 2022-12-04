import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    pybullet_ros_dir = get_package_share_directory('pybullet_ros')
    cartpole_description_dir = get_package_share_directory('cartpole_description')

    pybullet_ros_launch_dir = os.path.join(pybullet_ros_dir, 'launch')

    bringup_robot_example_path = os.path.join(pybullet_ros_launch_dir, 'bringup_robot_example.launch.py')
    cartpole_urdf_path = os.path.join(cartpole_description_dir, 'robot/urdf/robot.urdf')

    launch_arguments = {
        'model': cartpole_urdf_path
    }

    jetleg_pybullet_ros = IncludeLaunchDescription(PythonLaunchDescriptionSource(bringup_robot_example_path), launch_arguments=launch_arguments.items())

    return LaunchDescription([jetleg_pybullet_ros])
