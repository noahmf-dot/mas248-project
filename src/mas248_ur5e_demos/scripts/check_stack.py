"""Inspect most important ros2_control + MoveIt endpoints without moving anything."""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup,ExecuteTrajectory
from moveit_msgs.srv import ApplyPlanningScene,GetCartesianPath
from controller_manager_msgs.srv import ListControllers

def main():
    rclpy.init();node=Node('mas248_check_stack')
    try:
        endpoints=[('/move_action',ActionClient(node,MoveGroup,'/move_action').wait_for_server(timeout_sec=2)),
          ('/execute_trajectory',ActionClient(node,ExecuteTrajectory,'/execute_trajectory').wait_for_server(timeout_sec=2)),
          ('/compute_cartesian_path',node.create_client(GetCartesianPath,'/compute_cartesian_path').wait_for_service(timeout_sec=2)),
          ('/apply_planning_scene',node.create_client(ApplyPlanningScene,'/apply_planning_scene').wait_for_service(timeout_sec=2)),
          ('/controller_manager/list_controllers',node.create_client(ListControllers,'/controller_manager/list_controllers').wait_for_service(timeout_sec=2))]
        for name,ready in endpoints:print(('[OK]   ' if ready else '[MISS] ')+name)
    finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
