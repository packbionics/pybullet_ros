from launch import LaunchDescription
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():

    share_dir = get_package_share_directory('pybullet_ros')
    
    # partial configuration params for pybullet_ros node, rest will be loaded from config_file
    
    config_file_arg = DeclareLaunchArgument(
        "config_file", 
        default_value=TextSubstitution(
            text=os.path.join(
                share_dir, 
                "config/pybullet_params_example.yaml"
            )
        )
    )
    plugin_import_prefix_arg = DeclareLaunchArgument(
        "plugin_import_prefix", 
        default_value=TextSubstitution(
            text="pybullet_ros.plugins"
        )
    )
    environment_arg = DeclareLaunchArgument(
        "environment", 
        default_value=TextSubstitution(
            text="environment"
        )
    )
    pybullet_gui_arg = DeclareLaunchArgument(
        "pybullet_gui", 
        default_value=TextSubstitution(
            text="True"
        )
    )
    robot_urdf_path_arg = DeclareLaunchArgument(
        "robot_urdf_path", 
        default_value=TextSubstitution(
            text=os.path.join(
                share_dir,
                "common/test/urdf/r2d2_robot/r2d2.urdf.xacro"
            )
        )
    )
    pause_simulation_arg = DeclareLaunchArgument(
        "pause_simulation", 
        default_value=TextSubstitution(
            text="False"
        )
    )
    parallel_plugin_execution_arg = DeclareLaunchArgument(
        "parallel_plugin_execution", 
        default_value=TextSubstitution(
            text="True"
        )
    )
    robot_pose_x_arg = DeclareLaunchArgument(
        "robot_pose_x", 
        default_value=TextSubstitution(
            text="0.0"
        )
    )
    robot_pose_y_arg = DeclareLaunchArgument(
        "robot_pose_y", 
        default_value=TextSubstitution(
            text="0.0"
        )
    )
    robot_pose_z_arg = DeclareLaunchArgument(
        "robot_pose_z", 
        default_value=TextSubstitution(
            text="1.0"
        )
    )
    robot_pose_yaw_arg = DeclareLaunchArgument(
        "robot_pose_yaw", 
        default_value=TextSubstitution(
            text="0.0"
        )
    )
    fixed_base_arg = DeclareLaunchArgument(
        "fixed_base", 
        default_value=TextSubstitution(
            text="False"
        )
    )
    use_deformable_world_arg = DeclareLaunchArgument(
        "use_deformable_world", 
        default_value=TextSubstitution(
            text="False"
        )
    )
    gui_options_arg = DeclareLaunchArgument(
        "gui_options", 
        default_value=TextSubstitution(
            text=""
        )
    )

    return LaunchDescription([
        config_file_arg,
        plugin_import_prefix_arg,
        environment_arg,
        pybullet_gui_arg,
        robot_urdf_path_arg,
        pause_simulation_arg,
        parallel_plugin_execution_arg,
        robot_pose_x_arg,
        robot_pose_y_arg,
        robot_pose_z_arg,
        robot_pose_yaw_arg,
        fixed_base_arg,
        use_deformable_world_arg,
        gui_options_arg,
        Node(
            package='pybullet_ros',
            executable='pybullet_ros_node',
            output='screen',
            parameters=[
                LaunchConfiguration('config_file'),
                {
                    "robot_description": os.path.join(
                        share_dir,
                        "common/test/urdf/r2d2_robot/r2d2.urdf.xacro"),
                    "plugin_import_prefix": LaunchConfiguration('plugin_import_prefix'),
                    "environment": LaunchConfiguration('environment'),
                    "pybullet_gui": LaunchConfiguration('pybullet_gui'),
                    "robot_urdf_path": LaunchConfiguration('robot_urdf_path'),
                    "pause_simulation": LaunchConfiguration('pause_simulation'),
                    "robot_pose_x": LaunchConfiguration('robot_pose_x'),
                    "robot_pose_y": LaunchConfiguration('robot_pose_y'),
                    "robot_pose_z": LaunchConfiguration('robot_pose_z'),
                    "robot_pose_yaw": LaunchConfiguration('robot_pose_yaw'),
                    "fixed_base": LaunchConfiguration('fixed_base'),
                    "use_deformable_world": LaunchConfiguration('use_deformable_world'),
                    "gui_options": LaunchConfiguration('gui_options'),
                }
            ]
        )
    ])
