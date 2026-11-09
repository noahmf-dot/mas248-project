"""MoveIt move_group + RViz2. Start ros2_control.launch.py separately for /joint_states."""
import sys
from pathlib import Path
from launch import LaunchDescription
sys.path.insert(0, str(Path(__file__).parent))
from _mas248_common import planning_nodes

def generate_launch_description():
    return LaunchDescription(planning_nodes(rviz=True))
