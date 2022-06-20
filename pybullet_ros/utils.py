import csv
import yaml
import os

from ament_index_python import get_package_share_path
from ament_index_python import get_package_share_directory

def model_info_from_yaml(path):
    file = open(path)
    rows = []

    package_key = "Package"
    model_path_key = "Model"
    x_coord_key = "X-Coord"
    y_coord_key = "Y-Coord"
    z_coord_key = "Z-Coord"
    yaw_key = "Yaw"

    try:
        yaml_contents = yaml.safe_load(file)
        for model in yaml_contents:
            description = yaml_contents[model]
            row = []
            row.append(description[package_key])
            row.append(description[model_path_key])
            row.append(description[x_coord_key])
            row.append(description[y_coord_key])
            row.append(description[z_coord_key])
            row.append(description[yaw_key])
            rows.append(row)
    except yaml.YAMLError as exc:
        print(exc)
    return rows

def model_info_from_csv(path):
        file = open(path)
        csv_reader = csv.reader(file)
        # Skip header
        next(csv_reader)
        # Read model name and pose: x y z
        rows = []
        for row in csv_reader:
            rows.append(row)
        return rows
    
def model_path_pose_from_file(path):
    rows = model_info_from_yaml(path)
    model_path_pose = []
    for row in rows:
        # set model path
        model_path = str(get_package_share_path(row[0]) / row[1])

        # set x, y, and z positions
        model_pose = row[2:5]
        model_pose = list(map(float, model_pose))

        # set yaw pose
        pose_yaw = float(row[5])

        model_path_pose.append([model_path, model_pose, pose_yaw])        
    return model_path_pose

def urdf_from_xacro(xacro_path):
    # remove xacro from name
    path_end_without_xacro = xacro_path.find('.xacro')
    path_without_xacro = xacro_path[0: path_end_without_xacro]

    # use xacro command to generate URDF from XACRO
    os.system(f'xacro {xacro_path} -o {path_without_xacro}')
    return path_without_xacro 

def load_model_path_pose():
    pybullet_ros_dir = get_package_share_directory('pybullet_ros')
    path_from_package = os.path.join('config', 'model_descriptions.yaml')
    model_loader_path = os.path.join(pybullet_ros_dir, path_from_package)
    return model_path_pose_from_file(model_loader_path)