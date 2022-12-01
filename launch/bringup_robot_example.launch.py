import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    pybullet_ros_dir = get_package_share_directory('pybullet_ros')
    
    jetleg_pybullet_ros_path = [os.path.join(pybullet_ros_dir, 'launch'), '/jetleg_pybullet_ros.launch.py']
    bringup_robot_xacro_path = os.path.join(pybullet_ros_dir, 'common/test/urdf/r2d2_robot/r2d2.urdf.xacro')

    launch_arguments = {
        'model': bringup_robot_xacro_path
    }

    jetleg_pybullet_ros = IncludeLaunchDescription(PythonLaunchDescriptionSource(jetleg_pybullet_ros_path), launch_arguments=launch_arguments.items())

    return LaunchDescription([jetleg_pybullet_ros])
