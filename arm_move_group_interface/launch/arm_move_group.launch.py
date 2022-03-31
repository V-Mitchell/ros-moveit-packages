import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import xacro

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
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory('arm_assembly'),
            'urdf/arm_assembly.urdf.xacro',
        )
    )
    robot_description = {'robot_description': robot_description_config.toxml()}

    robot_description_semantic_config = load_file('arm_move_group_interface', 'config/arm_assembly.srdf')
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_config}

    kinematics_yaml = load_yaml('arm_move_group_interface', 'config/kinematics.yaml')
    robot_description_kinematics = {'robot_description_kinematics': kinematics_yaml}

    # Planning Functionality
    ompl_planning_pipeline_config = {
        'planning_pipelines': ['ompl'],
        'ompl': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            'start_state_max_bounds_error': 0.1,
        },
    }
    ompl_planning_yaml = load_yaml('arm_move_group_interface', 'config/ompl_planning.yaml')
    ompl_planning_pipeline_config['ompl'].update(ompl_planning_yaml)

    # Trajectory Execution Functionality
    moveit_controllers_yaml = load_yaml('arm_move_group_interface', 'config/controllers.yaml')
    moveit_controllers = {
        'moveit_simple_controllers_manager' : moveit_controllers_yaml,
        'moveit_controllers_manager' : 'moveit_simple_controllers_manager/MoveItSimpleControllerManager',
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trjectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01
    }

    planning_scene_monitor_config = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_update': True,
        'publish_transform_updates': True
    }

    joint_limits = {
        'robot_description_planning': load_yaml('arm_move_group_interface', 'config/joint_limits.yaml')
    }

    # Start move_group node/action server
    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_config,
            joint_limits,
        ],
    )

    # Rviz
    rviz_config_file = (get_package_share_directory('arm_move_group_interface') + '/launch/arm.rviz')
    rviz_node = Node(
        name='rviz',
        package='rviz2',
        executable='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
            joint_limits,
        ],
    )

    # Static TF
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "arm_link0"],
    )

    # Publish TF
    robot_state_publisher = Node(
        name='robot_state_publisher',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    ros2_controllers_path = os.path.join(get_package_share_directory('arm_move_group_interface'), 'config', 'ros_controllers.yaml',)
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, ros2_controllers_path],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
    )

    return LaunchDescription(
        [
            rviz_node,
            static_tf,
            robot_state_publisher,
            run_move_group_node,
            #ros2_control_node,
        ]
    )

