import os

from ament_index_python import get_package_share_directory
from ament_index_python import get_package_share_path
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.substitutions import Command, TextSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration


def generate_launch_description():

    pybullet_ros_dir = get_package_share_directory('pybullet_ros')
    description_dir = get_package_share_path('jetleg_description')
    
    default_model_path = description_dir / 'urdf/jetleg_standalone.xacro'

    model_spec_path = os.path.join(pybullet_ros_dir, 'config/model_descriptions.yaml')

    # partial configuration params for pybullet_ros node, rest will be loaded from config_file
    
    config_file_arg = DeclareLaunchArgument(
        "config_file", 
        default_value=TextSubstitution(
            text=os.path.join(
                pybullet_ros_dir, 
                "config/jetleg_params.yaml"
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
    model_arg = DeclareLaunchArgument(
        name='model', 
        default_value=str(default_model_path),
        description='Absolute path to robot xacro file'
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
            text='""'
        )
    )
    models_to_load_arg = DeclareLaunchArgument(
        "models_to_load",
        default_value=TextSubstitution(
            text=str(model_spec_path)
        )
    )

    config_file = LaunchConfiguration('config_file')
    plugin_import_prefix = LaunchConfiguration('plugin_import_prefix')
    environment = LaunchConfiguration('environment')
    pybullet_gui = LaunchConfiguration('pybullet_gui')
    robot_xacro_path = LaunchConfiguration('model')
    pause_simulation = LaunchConfiguration('pause_simulation')
    fixed_base = LaunchConfiguration('fixed_base')
    use_deformable_world = LaunchConfiguration('use_deformable_world')
    gui_options = LaunchConfiguration('gui_options')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    models_to_load = LaunchConfiguration('models_to_load')

    robot_urdf = Command(['xacro',' ',LaunchConfiguration('model')])

    pybullet_ros_parameters=[
        config_file, 
        {
            "plugin_import_prefix": plugin_import_prefix,
            "environment": environment,
            "pybullet_gui": pybullet_gui,
            "robot_urdf_path": robot_xacro_path,
            "pause_simulation": pause_simulation,
            "fixed_base": fixed_base,
            "use_deformable_world": use_deformable_world,
            "gui_options": gui_options,
            "use_sim_time": use_sim_time,
            "models_to_load": models_to_load
        }
    ]

    return LaunchDescription([
        config_file_arg,
        plugin_import_prefix_arg,
        environment_arg,
        pybullet_gui_arg,
        model_arg,
        pause_simulation_arg,
        parallel_plugin_execution_arg,
        fixed_base_arg,
        use_deformable_world_arg,
        gui_options_arg,
        models_to_load_arg,
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
        )
    ])
