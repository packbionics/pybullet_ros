import yaml
import os

from ament_index_python import get_package_share_path

def urdf_from_xacro(xacro_path):
    # remove xacro from name
    path_end_without_xacro = xacro_path.find('.xacro')
    path_without_xacro = xacro_path[0: path_end_without_xacro]

    # use xacro command to generate URDF from XACRO and return path of new file
    os.system(f'xacro {xacro_path} -o {path_without_xacro}')
    return path_without_xacro 


class ModelLoader:

    def __init__(self, path):
        # keys to access values from yaml contents dictionary
        self.package_key = "Package"         # name of package containing URDF or XACRO
        self.model_path_key = "RelPath"      # path of URDF or XACRO relative to package directory
        self.abs_path_key = "AbsPath"        # absolute path of URDF or XACRO (generated from package path and relative URDF or XACRO path)
        self.pose_key = "Pose"               # xyz pose of model
        self.yaw_key = "Yaw"                 # yaw orientation of model

        self.models = self.load_models_from_file(path)

    def load_from_yaml(self, path):
        # open file for reading
        file = open(path, "r")
        yaml_contents = {}

        try:
            # load contents into python dictionary
            yaml_contents = yaml.safe_load(file)
        except yaml.YAMLError as exc:
            print(exc)
        # return yaml contents as dictionary
        return yaml_contents
        
    def load_models_from_file(self, path):

        models = self.load_from_yaml(path)
        model_path_pose = []
        for model_key in models:
            # current model to load
            model = models[model_key]
            # set model path
            model_path = str(get_package_share_path(model[self.package_key]) / model[self.model_path_key])
            # set x, y, and z positions
            model_pose = model[self.pose_key]
            # set yaw pose
            pose_yaw = model[self.yaw_key]

            model_dict = {
                self.abs_path_key: model_path,
                self.pose_key: model_pose,
                self.yaw_key: pose_yaw
            }
            model_path_pose.append(model_dict)        
        return model_path_pose