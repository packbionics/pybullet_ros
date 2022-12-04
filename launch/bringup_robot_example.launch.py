import os

from ament_index_python import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.substitutions import Command, TextSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration

def generate_launch_description():

    # partial configuration params for pybullet_ros node, rest will be loaded from config_file

    config_file = LaunchConfiguration('config_file')
    model_config_file = LaunchConfiguration('model_config_file')
    plugin_import_prefix = LaunchConfiguration('plugin_import_prefix')
    environment = LaunchConfiguration('environment')
    pybullet_gui = LaunchConfiguration('pybullet_gui')
    pause_simulation = LaunchConfiguration('pause_simulation')
    fixed_base = LaunchConfiguration('fixed_base')
    use_deformable_world = LaunchConfiguration('use_deformable_world')
    gui_options = LaunchConfiguration('gui_options')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    models_to_load = LaunchConfiguration('models_to_load')
    model = LaunchConfiguration('model')
    robot_pose_x = LaunchConfiguration('robot_pose_x')
    robot_pose_y = LaunchConfiguration('robot_pose_y')
    robot_pose_z = LaunchConfiguration('robot_pose_z')
    robot_pose_yaw = LaunchConfiguration('robot_pose_yaw')

    pybullet_ros_dir = get_package_share_directory('pybullet_ros')

    # path of file describing robots to load
    default_model_config_file_path = "''"
    default_model_path = os.path.join(get_package_share_directory('pybullet_ros'), 'common/test/urdf/r2d2_robot/r2d2.urdf.xacro')

    # config file defining pybullet parameters
    default_config_file_path = os.path.join(pybullet_ros_dir, "config/pybullet/jetleg_params.yaml")
    
    config_file_arg = DeclareLaunchArgument(
        "config_file", 
        default_value=TextSubstitution(
            text=default_config_file_path
        )
    )
    model_config_file_arg = DeclareLaunchArgument(
        "model_config_file",
        default_value=TextSubstitution(
            text=default_model_config_file_path
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
            text='""'
        )
    )
    models_to_load_arg = DeclareLaunchArgument(
        "models_to_load",
        default_value=model_config_file
    )
    model_arg = DeclareLaunchArgument(
        "model",
        default_value=default_model_path
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

    pybullet_ros_parameters=[
        config_file,
        {
            "plugin_import_prefix": plugin_import_prefix,
            "environment": environment,
            "pybullet_gui": pybullet_gui,
            "pause_simulation": pause_simulation,
            "fixed_base": fixed_base,
            "use_deformable_world": use_deformable_world,
            "gui_options": gui_options,
            "use_sim_time": use_sim_time,
            "models_to_load": models_to_load,
            "robot_path": model,
            "robot_pose_x": robot_pose_x,
            "robot_pose_y": robot_pose_y,
            "robot_pose_z": robot_pose_z,
            "robot_pose_yaw": robot_pose_yaw,
        }
    ]

    robot_urdf = Command(['xacro',' ', model])

    return LaunchDescription([
        config_file_arg,
        model_config_file_arg,
        plugin_import_prefix_arg,
        environment_arg,
        pybullet_gui_arg,
        pause_simulation_arg,
        parallel_plugin_execution_arg,
        fixed_base_arg,
        use_deformable_world_arg,
        gui_options_arg,
        models_to_load_arg,
        model_arg,
        robot_pose_x_arg,
        robot_pose_y_arg,
        robot_pose_z_arg,
        robot_pose_yaw_arg,
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
        )
    ])
