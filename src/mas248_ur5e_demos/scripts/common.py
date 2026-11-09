"""Humble-compatible ROS2 MoveIt action/service wrappers. No moveit_py dependency."""
import math
import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (Constraints, JointConstraint, PositionConstraint,
                              OrientationConstraint, BoundingVolume, MoveItErrorCodes)
from moveit_msgs.srv import GetCartesianPath
from shape_msgs.msg import SolidPrimitive

JOINTS = ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint',
          'wrist_1_joint','wrist_2_joint','wrist_3_joint']
WORLD='world'
GROUP='manipulator'
TIP='tcp'

def quat_down():
    """Quaternion for drawing with +tool-Z pointing down: 180 deg around X."""
    from geometry_msgs.msg import Quaternion
    return Quaternion(x=1.0,y=0.0,z=0.0,w=0.0)

def make_pose(x,y,z):
    pose=Pose()
    pose.position.x=float(x);pose.position.y=float(y);pose.position.z=float(z)
    pose.orientation=quat_down()
    return pose

def wait_future(node, future, timeout_sec=120.0):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    if not future.done():
        raise TimeoutError('MoveIt service/action timed out')
    error=future.exception()
    if error is not None: raise RuntimeError(str(error))
    return future.result()

def move_group(node, *, joints=None, pose=None, execute=False, planning_time=12.0,
               start_joint_positions=None):
    if (joints is None) == (pose is None):
        raise ValueError('Specify exactly one of joints or pose')
    client=ActionClient(node, MoveGroup, '/move_action')
    if not client.wait_for_server(timeout_sec=20.0):
        raise RuntimeError('MoveGroup action /move_action not available; start demo.launch.py')
    goal=MoveGroup.Goal()
    goal.request.group_name=GROUP
    goal.request.pipeline_id='ompl'
    goal.request.planner_id='RRTConnectkConfigDefault'
    goal.request.num_planning_attempts=5
    goal.request.allowed_planning_time=float(planning_time)
    goal.request.max_velocity_scaling_factor=0.10
    goal.request.max_acceleration_scaling_factor=0.10
    if start_joint_positions is None:
        goal.request.start_state.is_diff=True
    else:
        if len(start_joint_positions)!=len(JOINTS):
            raise ValueError('Expected six start joint positions')
        goal.request.start_state.is_diff=False
        goal.request.start_state.joint_state.name=list(JOINTS)
        goal.request.start_state.joint_state.position=[float(v) for v in start_joint_positions]
    cs=Constraints()
    if joints is not None:
        if len(joints)!=6: raise ValueError('Expected 6 joint angles in radians')
        for name,val in zip(JOINTS,joints):
            j=JointConstraint();j.joint_name=name;j.position=float(val)
            j.tolerance_above=0.015;j.tolerance_below=0.015;j.weight=1.0
            cs.joint_constraints.append(j)
    else:
        pc=PositionConstraint();pc.header.frame_id=WORLD;pc.link_name=TIP
        pc.weight=1.0
        volume=BoundingVolume()
        primitive=SolidPrimitive();primitive.type=SolidPrimitive.SPHERE
        primitive.dimensions=[0.008]
        volume.primitives.append(primitive)
        volume.primitive_poses.append(pose)
        pc.constraint_region=volume
        cs.position_constraints.append(pc)
        oc=OrientationConstraint();oc.header.frame_id=WORLD;oc.link_name=TIP
        oc.orientation=pose.orientation
        oc.absolute_x_axis_tolerance=0.10
        oc.absolute_y_axis_tolerance=0.10
        oc.absolute_z_axis_tolerance=0.10
        oc.weight=1.0
        cs.orientation_constraints.append(oc)
    goal.request.goal_constraints=[cs]
    goal.planning_options.plan_only=not execute
    goal.planning_options.planning_scene_diff.is_diff=True
    goal.planning_options.planning_scene_diff.robot_state.is_diff=True
    node.get_logger().info('MoveGroup '+('EXECUTE (MOCK)' if execute else 'PLAN ONLY')+' goal requested')
    accepted=wait_future(node,client.send_goal_async(goal),60)
    if not accepted.accepted:raise RuntimeError('MoveGroup goal rejected')
    result=wait_future(node,accepted.get_result_async(),150).result
    if result.error_code.val!=MoveItErrorCodes.SUCCESS:
        raise RuntimeError('MoveGroup failed: MoveItErrorCodes='+str(result.error_code.val))
    node.get_logger().info(f'MoveGroup success, {len(result.planned_trajectory.joint_trajectory.points)} trajectory points')
    return result

def cartesian(node, waypoints, max_step=0.008, *, start_joint_positions=None,
              jump_threshold=1.5, avoid_collisions=True):
    cli=node.create_client(GetCartesianPath, '/compute_cartesian_path')
    if not cli.wait_for_service(timeout_sec=20.0):
        raise RuntimeError('MoveIt /compute_cartesian_path not available')
    req=GetCartesianPath.Request()
    req.header.frame_id=WORLD
    if start_joint_positions is None:
        req.start_state.is_diff=True
    else:
        if len(start_joint_positions)!=len(JOINTS):
            raise ValueError('Expected six start joint positions')
        req.start_state.is_diff=False
        req.start_state.joint_state.name=list(JOINTS)
        req.start_state.joint_state.position=[float(v) for v in start_joint_positions]
    req.group_name=GROUP
    req.link_name=TIP
    req.waypoints=list(waypoints)
    req.max_step=float(max_step)
    req.jump_threshold=float(jump_threshold)
    req.avoid_collisions=bool(avoid_collisions)
    req.max_velocity_scaling_factor=0.10
    req.max_acceleration_scaling_factor=0.10
    result=wait_future(node, cli.call_async(req), 150)
    node.get_logger().info(f'Cartesian completion fraction: {result.fraction:.3f}')
    return result

def execute_trajectory(node, trajectory):
    points=trajectory.joint_trajectory.points
    if not points:raise RuntimeError('Empty Cartesian trajectory')
    if (points[-1].time_from_start.sec==0 and points[-1].time_from_start.nanosec==0):
        raise RuntimeError('Cartesian trajectory is not time-parameterized; refusing execution')
    cli=ActionClient(node, ExecuteTrajectory,'/execute_trajectory')
    if not cli.wait_for_server(timeout_sec=20):
        raise RuntimeError('/execute_trajectory is unavailable')
    goal=ExecuteTrajectory.Goal();goal.trajectory=trajectory
    handle=wait_future(node,cli.send_goal_async(goal),30)
    if not handle.accepted: raise RuntimeError('ExecuteTrajectory goal rejected')
    result=wait_future(node,handle.get_result_async(),180).result
    if result.error_code.val!=MoveItErrorCodes.SUCCESS:
        raise RuntimeError('Trajectory execution failed error '+str(result.error_code.val))
    node.get_logger().info(f'Mock trajectory executed: {len(points)} points')
