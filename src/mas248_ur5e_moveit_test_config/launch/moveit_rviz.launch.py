
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "mas248_ur5e",
            package_name="mas248_ur5e_moveit_test_config"
        ).to_moveit_configs()
    )

    rviz_config = str(
        moveit_config.package_path / "config/moveit.rviz"
    )

    return LaunchDescription([
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz",
            output="screen",
            arguments=["-d", rviz_config],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
            ],
        )
    ])

