from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('mas248_ur5e')
    urdf_file = PathJoinSubstitution([pkg, 'urdf', 'ur5e_workcell.urdf.xacro'])
    rviz_file = PathJoinSubstitution([pkg, 'rviz', 'ur5e_workcell.rviz'])

    ur_type_arg = DeclareLaunchArgument('ur_type', default_value='ur5e')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_file, ' ur_type:=', LaunchConfiguration('ur_type')]),
        value_type=str
    )

    return LaunchDescription([
        ur_type_arg,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_file],
            output='screen'
        ),
    ])
