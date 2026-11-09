"""MOCK controllers only. Start this first if launching MoveIt separately."""
import sys
from pathlib import Path
from launch import LaunchDescription
sys.path.insert(0, str(Path(__file__).parent))
from _mas248_common import controllers_nodes

def generate_launch_description():
    return LaunchDescription(controllers_nodes())
