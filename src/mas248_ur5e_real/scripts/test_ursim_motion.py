#!/usr/bin/env python3
"""Small URSim-only joint motion test. Plan-only by default."""
import argparse
import math
import subprocess
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import GetParameters
from controller_manager_msgs.srv import ListControllers
from sensor_msgs.msg import JointState
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import MoveItErrorCodes
from action_msgs.msg import GoalStatus

from common import JOINTS, move_group, wait_future
from picture_planning import (
    ordered_positions, check_trajectory, check_timing
)

SIM_IP = '192.168.56.101'
HOST_IP = '192.168.56.1'


def verify_simulator():
    output = subprocess.check_output(
        ['docker', 'inspect', '-f',
         '{{.Config.Image}}|{{.State.Running}}|'
         '{{(index .NetworkSettings.Networks "ursim_net").IPAddress}}',
         'ursim'],
        text=True
    ).strip()

    image, running, ip = output.split('|')

    if not image.startswith('universalrobots/ursim_e-series:'):
        raise RuntimeError('Unexpected Docker image')
    if running != 'true' or ip != SIM_IP:
        raise RuntimeError('URSim container/IP mismatch')

    route = subprocess.check_output(
        ['ip', 'route', 'get', SIM_IP], text=True
    )
    if f'src {HOST_IP}' not in route or 'dev br-' not in route:
        raise RuntimeError('Robot route is not the expected Docker bridge')

    drivers = []
    for path in Path('/proc').glob('[0-9]*/cmdline'):
        try:
            command = path.read_bytes().replace(b'\0', b' ').decode()
        except (OSError, UnicodeError):
            continue
        if ('/opt/ros/humble/bin/ros2 launch '
                'mas248_ur5e_real driver.launch.py') in command:
            drivers.append(command)

    if len(drivers) != 1:
        raise RuntimeError('Expected exactly one MAS248 driver launch')

    driver = drivers[0]
    required = [
        f'robot_ip:={SIM_IP}',
        f'reverse_ip:={HOST_IP}',
        'calibration:=/home/noah/ursim_calibration.yaml',
    ]
    if not all(value in driver.split() for value in required):
        raise RuntimeError('Driver launch arguments do not match URSim')

    print('Docker and driver checks: OK')


def verify_ros(node):
    client = node.create_client(
        GetParameters, '/move_group/get_parameters')
    if not client.wait_for_service(timeout_sec=10):
        raise RuntimeError('MoveIt unavailable')

    request = GetParameters.Request()
    request.names = ['robot_description', 'allow_trajectory_execution']
    response = wait_future(node, client.call_async(request), 15)
    node.destroy_client(client)

    if len(response.values) != 2:
        raise RuntimeError('Missing MoveIt parameters')

    description, execution = response.values
    if description.type != 4 or execution.type != 1:
        raise RuntimeError('Invalid MoveIt parameter types')

    root = ET.fromstring(description.string_value)
    controls = root.findall('ros2_control')
    if len(controls) != 1:
        raise RuntimeError('Expected one hardware system')

    plugin = controls[0].findtext('./hardware/plugin', '')
    robot_ip = controls[0].findtext(
        "./hardware/param[@name='robot_ip']", '')

    if not plugin.startswith('ur_robot_driver/'):
        raise RuntimeError('Expected UR robot driver')
    if robot_ip != SIM_IP:
        raise RuntimeError(
            f'URDF robot IP mismatch: {robot_ip!r}')

    if not execution.bool_value:
        raise RuntimeError('MoveIt execution is disabled')

    cli = node.create_client(
        ListControllers, '/controller_manager/list_controllers')
    if not cli.wait_for_service(timeout_sec=10):
        raise RuntimeError('Controller manager unavailable')

    controllers = wait_future(
        node, cli.call_async(ListControllers.Request()), 15)
    node.destroy_client(cli)

    states = {c.name: c.state for c in controllers.controller}
    if states.get('scaled_joint_trajectory_controller') != 'active':
        raise RuntimeError('Scaled controller is not active')

    for other in ('joint_trajectory_controller',
                  'passthrough_trajectory_controller'):
        if states.get(other) == 'active':
            raise RuntimeError(f'Unexpected active controller: {other}')

    print('MoveIt, URDF and controller checks: OK')


def read_joints(node):
    received = []

    def callback(msg):
        if set(msg.name) == set(JOINTS):
            received.append((
                ordered_positions(msg.name, msg.position),
                ordered_positions(msg.name, msg.velocity)
            ))

    sub = node.create_subscription(
        JointState, '/joint_states', callback, qos_profile_sensor_data)

    deadline = time.monotonic() + 10
    try:
        while len(received) < 2 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_subscription(sub)

    if len(received) < 2:
        raise RuntimeError('No current joint states')

    positions, velocities = received[-1]
    if max(abs(v) for v in velocities) > 0.02:
        raise RuntimeError('Robot is not stationary')

    return positions


def execute(node, trajectory):
    client = ActionClient(
        node, ExecuteTrajectory, '/execute_trajectory')

    if not client.wait_for_server(timeout_sec=10):
        raise RuntimeError('ExecuteTrajectory unavailable')

    goal = ExecuteTrajectory.Goal()
    goal.trajectory = trajectory

    handle = wait_future(node, client.send_goal_async(goal), 20)
    if not handle.accepted:
        raise RuntimeError('Execution goal rejected')

    final = trajectory.joint_trajectory.points[-1].time_from_start
    duration = final.sec + final.nanosec * 1e-9
    timeout = max(60.0, 4.0 * duration + 30.0)

    try:
        outcome = wait_future(
            node, handle.get_result_async(), timeout)
    except (TimeoutError, KeyboardInterrupt):
        print('Execution interrupted: requesting cancellation')
        try:
            cancel = wait_future(
                node, handle.cancel_goal_async(), 10)
            print('Cancel response:', cancel)
        except Exception as exc:
            print('Cancellation not confirmed:', exc)
        raise RuntimeError(
            'Verify robot has stopped in PolyScope before continuing')

    if (outcome.status != GoalStatus.STATUS_SUCCEEDED or
            outcome.result.error_code.val != MoveItErrorCodes.SUCCESS):
        raise RuntimeError(
            f'Execution failed: status={outcome.status}, '
            f'code={outcome.result.error_code.val}')

    print('EXECUTION SUCCESSFUL')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()

    verify_simulator()

    rclpy.init()
    node = Node('mas248_ursim_motion_test')
    try:
        verify_ros(node)
        start = read_joints(node)

        target = list(start)
        target[0] += 0.03

        print('Planning small joint motion...')
        result = move_group(
            node, joints=target, execute=False,
            planning_time=20.0,
            start_joint_positions=start)

        trajectory = result.planned_trajectory
        planned_start = ordered_positions(
            result.trajectory_start.joint_state.name,
            result.trajectory_start.joint_state.position)

        if max(abs(a-b) for a, b in zip(start, planned_start)) > 0.01:
            raise RuntimeError('Planning start does not match live state')

        check_trajectory(trajectory, planned_start, 0.35)
        check_timing(trajectory)

        for point in trajectory.joint_trajectory.points:
            positions = ordered_positions(
                trajectory.joint_trajectory.joint_names,
                point.positions)
            if max(abs(a-b) for a, b in zip(positions, start)) > 0.08:
                raise RuntimeError('Planned motion exceeds test bounds')

        final = ordered_positions(
            trajectory.joint_trajectory.joint_names,
            trajectory.joint_trajectory.points[-1].positions)

        if not 0.01 <= final[0] - start[0] <= 0.05:
            raise RuntimeError('Unexpected shoulder motion')

        print('PLANNING SUCCESSFUL')
        print('Trajectory points:',
              len(trajectory.joint_trajectory.points))
        print('Shoulder movement:',
              round(math.degrees(final[0]-start[0]), 3), 'degrees')

        if not args.execute:
            print('PLAN ONLY: no movement sent')
            return

        verify_simulator()
        verify_ros(node)

        current = read_joints(node)
        if max(abs(a-b) for a, b in zip(current, start)) > 0.01:
            raise RuntimeError('Robot moved after planning')

        confirmation = input(
            'Type URSIM to execute the small motion: ')
        if confirmation != 'URSIM':
            raise RuntimeError('Execution not confirmed')

        execute(node, trajectory)

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
