import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    pybullet_ros_dir = get_package_share_directory('pybullet_ros')
    jetleg_description_dir = get_package_share_directory('jetleg_description')

    jetleg_pybullet_ros_path = [os.path.join(pybullet_ros_dir, 'launch'), '/jetleg_pybullet_ros.launch.py']

    config_file_path = os.path.join(pybullet_ros_dir, "config/pybullet/leg_control_params.yaml")
    model_config_file_path = os.path.join(pybullet_ros_dir, "config/model/jetleg_single_leg.yaml")
    single_jetleg_xacro_path = os.path.join(jetleg_description_dir, 'urdf/jetleg_single.xacro')

    launch_arguments = {
        'config_file': config_file_path,
        'model_config_file': model_config_file_path,
        'model': single_jetleg_xacro_path
    }

    jetleg_pybullet_ros = IncludeLaunchDescription(PythonLaunchDescriptionSource(jetleg_pybullet_ros_path), launch_arguments=launch_arguments.items())

    return LaunchDescription([jetleg_pybullet_ros])
