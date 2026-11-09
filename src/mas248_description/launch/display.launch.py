"""MAS248 Lecture 8 offline robot model demo (ROS 2 Humble).

No robot driver and no Gazebo are used. Only RViz2 visualisation.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    share = Path(get_package_share_directory('mas248_description'))
    model = share / 'urdf' / 'two_link_arm.urdf.xacro'
    rviz_config = share / 'rviz' / 'display.rviz'

    robot_description = xacro.process_file(str(model)).toxml()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', str(rviz_config)],
        ),
    ])
