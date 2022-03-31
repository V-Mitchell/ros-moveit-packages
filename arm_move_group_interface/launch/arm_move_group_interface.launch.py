import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path

def load_file(package, file_path):
    """Load the contents of a file into a string"""
    try:
        with open(get_package_file(package, file_path), 'r') as file:
            return file.read()
    except EnvironmentError: 
        return None

def load_yaml(package, file_path):
    """Load a yaml file into a dictionary"""
    try:
        with open(get_package_file(package, file_path), 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():

    # Planning Context
    robot_description_config = load_file('arm_assembly', 'urdf/arm_assembly.urdf')
    robot_description = {'robot_description': robot_description_config}

    robot_description_semantic_config = load_file('arm_move_group_interface', 'config/arm_assembly.srdf')
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_config}

    kinematics_yaml = load_yaml('arm_move_group_interface', 'config/kinematics.yaml')

    # Move Group Interface executable
    arm_move_group = Node(
        name='arm_move_group_interface',
        package='arm_move_group_interface',
        executable='arm_move_group_interface',
        prefix='xterm -e',
        output='screen',
        parameters=[robot_description, robot_description_semantic, kinematics_yaml],
    )

    return LaunchDescription([arm_move_group])

