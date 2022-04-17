import os

from ament_index_python import get_package_share_directory
from ament_index_python import get_package_share_path
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer

from launch_ros.descriptions import ComposableNode

from launch import LaunchDescription
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.substitutions import Command, TextSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration


def generate_launch_description():

    pybullet_ros_dir = get_package_share_directory('pybullet_ros')
    description_dir = get_package_share_path('jetleg_description')
    obstacle_dir = get_package_share_path('obstacles')
    
    default_model_path = description_dir / 'urdf/testrig_vision.xacro'
    default_obstacle_path = obstacle_dir / 'urdf/prebuilts/staircase_standalone.xacro'

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
    num_urdf_models_arg = DeclareLaunchArgument(
        name='num_urdf_models',
        default_value=TextSubstitution(
            text="2"
        ),
        description='Number of models to load into PyBullet simulation'
    )
    model_arg = DeclareLaunchArgument(
        name='model', 
        default_value=str(default_model_path),
        description='Absolute path to robot xacro file'
    )
    obstacle_arg = DeclareLaunchArgument(
        name='obstacle', 
        default_value=str(default_obstacle_path),
        description='Absolute path to obstacle xacro file'
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
            text="0.1"
        )
    )
    robot_pose_yaw_arg = DeclareLaunchArgument(
        "robot_pose_yaw", 
        default_value=TextSubstitution(
            text="0.0"
        )
    )
    obstacle_pose_x_arg = DeclareLaunchArgument(
        "obstacle_pose_x", 
        default_value=TextSubstitution(
            text="0.5"
        )
    )
    obstacle_pose_y_arg = DeclareLaunchArgument(
        "obstacle_pose_y", 
        default_value=TextSubstitution(
            text="0.0"
        )
    )
    obstacle_pose_z_arg = DeclareLaunchArgument(
        "obstacle_pose_z", 
        default_value=TextSubstitution(
            text="0.1"
        )
    )
    obstacle_pose_yaw_arg = DeclareLaunchArgument(
        "obstacle_pose_yaw", 
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

    config_file = LaunchConfiguration('config_file')
    plugin_import_prefix = LaunchConfiguration('plugin_import_prefix')
    environment = LaunchConfiguration('environment')
    pybullet_gui = LaunchConfiguration('pybullet_gui')
    robot_xacro_path = LaunchConfiguration('model')
    obstacle_xacro_path = LaunchConfiguration('obstacle')
    pause_simulation = LaunchConfiguration('pause_simulation')
    num_urdf_models = LaunchConfiguration('num_urdf_models')
    robot_pose_x = LaunchConfiguration('robot_pose_x')
    robot_pose_y = LaunchConfiguration('robot_pose_y')
    robot_pose_z = LaunchConfiguration('robot_pose_z')
    robot_pose_yaw = LaunchConfiguration('robot_pose_yaw')
    obstacle_pose_x = LaunchConfiguration('obstacle_pose_x')
    obstacle_pose_y = LaunchConfiguration('obstacle_pose_y')
    obstacle_pose_z = LaunchConfiguration('obstacle_pose_z')
    obstacle_pose_yaw = LaunchConfiguration('obstacle_pose_yaw')
    fixed_base = LaunchConfiguration('fixed_base')
    use_deformable_world = LaunchConfiguration('use_deformable_world')
    gui_options = LaunchConfiguration('gui_options')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    robot_urdf = Command(['xacro',' ',LaunchConfiguration('model')])

    pybullet_ros_parameters=[
        config_file, 
        {
            "plugin_import_prefix": plugin_import_prefix,
            "environment": environment,
            "pybullet_gui": pybullet_gui,
            "urdf_model_path_0": robot_xacro_path,
            "urdf_model_path_1": obstacle_xacro_path,
            "pause_simulation": pause_simulation,
            "num_urdf_models": num_urdf_models,
            "model_pose_x_0": robot_pose_x,
            "model_pose_y_0": robot_pose_y,
            "model_pose_z_0": robot_pose_z, 
            "model_pose_yaw_0": robot_pose_yaw,
            "model_pose_x_1": obstacle_pose_x,
            "model_pose_y_1": obstacle_pose_y,
            "model_pose_z_1": obstacle_pose_z, 
            "obstacle_pose_yaw": obstacle_pose_yaw,
            "fixed_base": fixed_base,
            "use_deformable_world": use_deformable_world,
            #"gui_options": gui_options, FIXME: CAUSES ERROR WHEN WHEN RUNNING LAUNCH FILE
            "use_sim_time": use_sim_time
        }
    ]

    return LaunchDescription([
        config_file_arg,
        plugin_import_prefix_arg,
        environment_arg,
        pybullet_gui_arg,
        num_urdf_models_arg,
        model_arg,
        obstacle_arg,
        pause_simulation_arg, 
        parallel_plugin_execution_arg,
        robot_pose_x_arg,
        robot_pose_y_arg,
        robot_pose_z_arg,
        robot_pose_yaw_arg,
        obstacle_pose_x_arg,
        obstacle_pose_y_arg,
        obstacle_pose_z_arg,
        obstacle_pose_yaw_arg,
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
