from email.policy import default
import os

from ament_index_python import get_package_share_directory
from ament_index_python import get_package_share_path
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.substitutions import Command, TextSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration

def generate_launch_description():
    description_dir = get_package_share_path('jetleg_description')
    default_model_path = description_dir / 'urdf/testrig_vision.xacro'

    pybullet_ros_dir = get_package_share_directory('pybullet_ros')
    rviz_config_file = os.path.join(pybullet_ros_dir, 'config/pybullet_pointcloud_vision_config.rviz')

    # partial configuration params for pybullet_ros node, rest will be loaded from config_file
    
    config_file_arg = DeclareLaunchArgument(
        "config_file", 
        default_value=TextSubstitution(
            text=os.path.join(
                pybullet_ros_dir, 
                "config/jetleg_pybullet_vision_params.yaml"
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

    config_file = LaunchConfiguration('config_file')
    plugin_import_prefix = LaunchConfiguration('plugin_import_prefix')
    environment = LaunchConfiguration('environment')
    pybullet_gui = LaunchConfiguration('pybullet_gui')
    pause_simulation = LaunchConfiguration('pause_simulation')
    fixed_base = LaunchConfiguration('fixed_base')
    use_deformable_world = LaunchConfiguration('use_deformable_world')
    gui_options = LaunchConfiguration('gui_options')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    robot_urdf = Command(['xacro',' ', str(default_model_path)])

    default_pybullet_ros_params = {
            "plugin_import_prefix": plugin_import_prefix,
            "environment": environment,
            "pybullet_gui": pybullet_gui,
            "pause_simulation": pause_simulation,
            "fixed_base": fixed_base,
            "use_deformable_world": use_deformable_world,
            #"gui_options": gui_options, FIXME: CAUSES ERROR WHEN WHEN RUNNING LAUNCH FILE
            "use_sim_time": use_sim_time
    }
    
    pybullet_ros_parameters=[
        config_file, 
        default_pybullet_ros_params
    ]

    return LaunchDescription([
        config_file_arg,
        plugin_import_prefix_arg,
        environment_arg,
        pybullet_gui_arg,
        pause_simulation_arg, 
        parallel_plugin_execution_arg,
        fixed_base_arg,
        use_deformable_world_arg,
        gui_options_arg,
        Node(
            package='pybullet_ros',
            executable='pybullet_ros_wrapper',
            output='screen',
            parameters=pybullet_ros_parameters
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 
                         'robot_description': robot_urdf}],
            arguments=[robot_urdf]
        ),
        Node(
            package='pointcloud_proc_cpp',
            executable='gen_pointcloud',
            output='screen',
            remappings=[('image', 'camera/depth/image_raw'),
                        ('pointcloud', '/zed2i/zed_node/point_cloud/cloud_registered'),
                        ('camera_state', 'camera/state'),
                        ('camera_params', 'camera/params')]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        )
    ])
