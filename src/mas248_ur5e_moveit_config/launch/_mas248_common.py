"""Shared launch configuration. Consistent URDF, SRDF, joints, and controllers across launches."""
from pathlib import Path
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import TimerAction

CFG = Path(get_package_share_directory('mas248_ur5e_moveit_config'))
DESC = Path(get_package_share_directory('mas248_ur5e'))

def read_yaml(name):
    with open(CFG / 'config' / name, encoding='utf-8') as f:
        return yaml.safe_load(f)

def robot_description():
    # Offline Lecture 9 model: canonical Lecture 8 UR5e + mock ros2_control.
    model = CFG / 'urdf' / 'ur5e_workcell_mock.urdf.xacro'
    return xacro.process_file(str(model), mappings={'ur_type': 'ur5e', 'tf_prefix': ''}).toxml()

def base_parameters():
    return {'robot_description': robot_description(),
            'robot_description_semantic': (CFG/'config'/'ur5e_workcell.srdf').read_text(),
            'robot_description_kinematics': read_yaml('kinematics.yaml'),
            'robot_description_planning': read_yaml('joint_limits.yaml')}

def planning_parameters():
    params = base_parameters()
    params.update({
        'planning_pipelines': ['ompl'], 'default_planning_pipeline': 'ompl',
        'ompl': read_yaml('ompl_planning.yaml'),
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
        'moveit_simple_controller_manager': read_yaml('moveit_controllers.yaml')['moveit_simple_controller_manager'],
        'trajectory_execution.allowed_execution_duration_scaling': 1.6,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.02,
        'allow_trajectory_execution': True,
        'publish_robot_description': True,
        'publish_robot_description_semantic': True,
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    })
    return params

def controllers_nodes():
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen', parameters=[{'robot_description': ParameterValue(robot_description(), value_type=str)}]),
        Node(package='controller_manager', executable='ros2_control_node', output='screen',
             parameters=[{'robot_description': ParameterValue(robot_description(), value_type=str)}, str(CFG/'config'/'ros2_controllers.yaml')]),
        TimerAction(period=3.0, actions=[
            Node(package='controller_manager', executable='spawner',
                 arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'], output='screen'),
            Node(package='controller_manager', executable='spawner',
                 arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'], output='screen'),
        ])]

def planning_nodes(rviz=True):
    params = planning_parameters()
    nodes = [Node(package='moveit_ros_move_group', executable='move_group',
                  output='screen', parameters=[params])]
    if rviz:
        nodes.append(Node(package='rviz2', executable='rviz2', output='screen',
                          parameters=[params],
                          arguments=['-d', str(CFG/'config'/'mas248_moveit.rviz')]))
    return nodes
