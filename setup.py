import os
from glob import glob

from setuptools import setup

package_name = 'pybullet_ros'
submodules = [os.path.join(package_name, sub) for sub in ['plugins']]

data_files = [
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]

def glob_recursive(data_files, directory):
    files = glob(directory+'*.*')
    data_files.append((os.path.join('share', package_name, directory), files))
    subdirectories = glob(directory+'*/')
    if (subdirectories == []):
        return
    else:
        for dir in subdirectories:
            glob_recursive(data_files, dir)

data_directories = ['launch', 'config', 'scripts', 'config', 'common', 'doc']

for directory in data_directories:
    glob_recursive(data_files, directory)

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name] + submodules,
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anthonybrown0528',
    maintainer_email='anthonybrown0528@protonmail.com',
    description='ROS2 wrapper for pybullet simulator',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pybullet_ros_wrapper = pybullet_ros.pybullet_ros_wrapper:main'
        ],
    },
)
