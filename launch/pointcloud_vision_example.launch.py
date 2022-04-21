from email.policy import default
import os
import csv

from matplotlib.pyplot import text

from ament_index_python import get_package_share_directory
from ament_index_python import get_package_share_path
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer

from launch_ros.descriptions import ComposableNode

from launch import LaunchDescription
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.substitutions import Command, TextSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration

obstacle_dir = get_package_share_path('obstacles')
default_obstacle_path = obstacle_dir / 'urdf/prebuilts/staircase_standalone.xacro'

description_dir = get_package_share_path('jetleg_description')
default_model_path = description_dir / 'urdf/testrig_vision.xacro'

def read_csv(path):
    file = open(path)
    csv_reader = csv.reader(file)

    # Skip header
    next(csv_reader)

    # Read model name and pose: x y z
    rows = []
    for row in csv_reader:
        rows.append(row)
    return rows

def gen_model_path_pose_args(rows, params_dict):
    arg_list = []

    count = 0
    for row in rows:
        
        # Identify which urdf to load
        curr_model_path = ''
        if row[0] == 'testrig':
            curr_model_path = str(default_model_path)
        if row[0] == 'stairs':
            curr_model_path = str(default_obstacle_path)

        # set model path
        curr_model_path_arg = DeclareLaunchArgument(
            name="model_path_" + str(count),
            default_value=TextSubstitution(
                text=curr_model_path
            ) 
        )
        # set x pose
        curr_model_pose_x_arg = DeclareLaunchArgument(
            name="model_pose_x_" + str(count),
            default_value=TextSubstitution(
                text=row[1]
            ) 
        )
        # set y pose
        curr_model_pose_y_arg = DeclareLaunchArgument(
            name="model_pose_y_" + str(count),
            default_value=TextSubstitution(
                text=row[2]
            ) 
        )
        # set z pose
        curr_model_pose_z_arg = DeclareLaunchArgument(
            name="model_pose_z_" + str(count),
            default_value=TextSubstitution(
                text=row[3]
            ) 
        )
        # set yaw pose
        curr_model_pose_yaw_arg = DeclareLaunchArgument(
            name="model_pose_yaw_" + str(count),
            default_value=TextSubstitution(
                text=row[4]
            ) 
        )

        # set params
        params_dict["urdf_model_path_" + str(count)] = LaunchConfiguration('model_path_' + str(count))
        params_dict["model_pose_x_" + str(count)] = LaunchConfiguration('model_pose_x_' + str(count))
        params_dict["model_pose_y_" + str(count)] = LaunchConfiguration('model_pose_y_' + str(count))
        params_dict["model_pose_z_" + str(count)] = LaunchConfiguration('model_pose_z_' + str(count))
        params_dict["model_pose_yaw_" + str(count)] = LaunchConfiguration('model_pose_yaw_' + str(count))
        arg_list += [curr_model_path_arg, curr_model_pose_x_arg, curr_model_pose_y_arg, curr_model_pose_z_arg, curr_model_pose_yaw_arg]

        count += 1 
    
    return arg_list, params_dict

def generate_launch_description():

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
    num_urdf_models = LaunchConfiguration('num_urdf_models')
    fixed_base = LaunchConfiguration('fixed_base')
    use_deformable_world = LaunchConfiguration('use_deformable_world')
    gui_options = LaunchConfiguration('gui_options')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    robot_urdf = Command(['xacro',' ',LaunchConfiguration('model_path_0')])

    required_pybullet_ros_params = {
            "plugin_import_prefix": plugin_import_prefix,
            "environment": environment,
            "pybullet_gui": pybullet_gui,
            "pause_simulation": pause_simulation,
            "num_urdf_models": num_urdf_models,
            "fixed_base": fixed_base,
            "use_deformable_world": use_deformable_world,
            #"gui_options": gui_options, FIXME: CAUSES ERROR WHEN WHEN RUNNING LAUNCH FILE
            "use_sim_time": use_sim_time
    }

    # read csv
    rows = read_csv(os.path.join(pybullet_ros_dir, 'config/model_spawn.csv'))
    # append additional arguments and params for loading models
    arg_list, required_pybullet_ros_params = gen_model_path_pose_args(rows, required_pybullet_ros_params)
    
    pybullet_ros_parameters=[
        config_file, 
        required_pybullet_ros_params
    ]

    num_urdf_models_arg = DeclareLaunchArgument(
        name='num_urdf_models',
        default_value=TextSubstitution(
            text=str(len(rows))
        ),
        description='Number of models to load into PyBullet simulation'
    )

    return LaunchDescription(arg_list + [
        config_file_arg,
        plugin_import_prefix_arg,
        environment_arg,
        pybullet_gui_arg,
        num_urdf_models_arg,
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
