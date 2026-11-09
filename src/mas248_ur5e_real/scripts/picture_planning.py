#!/usr/bin/env python3
"""MAS248: Cartesian pen-up transfers between strokes; plan-only by default."""
import argparse
import copy
import json
import math
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from moveit_msgs.msg import MoveItErrorCodes
from common import (JOINTS, make_pose, move_group, cartesian,
                    execute_trajectory, wait_future)


class InvalidPlan(RuntimeError):
    pass


def ordered_positions(names, values):
    if len(names) != len(values) or len(set(names)) != len(names):
        raise InvalidPlan('Invalid joint names/positions')
    positions = dict(zip(names, values))
    if set(names) != set(JOINTS):
        raise InvalidPlan('Expected exactly the six UR5e joints')
    result = [float(positions[name]) for name in JOINTS]
    if not all(math.isfinite(v) for v in result):
        raise InvalidPlan('Non-finite joint position')
    return result


def check_trajectory(trajectory, start, max_delta):
    jt = trajectory.joint_trajectory
    if not jt.points or trajectory.multi_dof_joint_trajectory.points:
        raise InvalidPlan('Empty or unsupported trajectory')
    previous = start
    for index, point in enumerate(jt.points):
        current = ordered_positions(jt.joint_names, point.positions)
        # Raw differences intentionally reject a 2*pi discontinuity as well.
        if previous is not None:
            delta = max(abs(a-b) for a, b in zip(current, previous))
            limit = min(max_delta, 0.02) if index == 0 else max_delta
            if delta > limit:
                raise InvalidPlan(f'Joint step {delta:.4f} rad > {limit:.4f}')
        previous = current
    return previous


def check_timing(trajectory):
    previous = -1
    for point in trajectory.joint_trajectory.points:
        stamp = point.time_from_start
        ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
            raise InvalidPlan('Invalid trajectory duration')
        if ns <= previous:
            raise InvalidPlan('Trajectory times must strictly increase')
        previous = ns
        for values in (point.velocities, point.accelerations, point.effort):
            if values and (len(values) != len(JOINTS) or
                           not all(math.isfinite(v) for v in values)):
                raise InvalidPlan('Invalid trajectory dynamics')
    if previous <= 0:
        raise InvalidPlan('Trajectory is not time-parameterized; no execution')


def plan_cartesian(node, waypoints, start, args):
    result = cartesian(node, waypoints, max_step=args.max_step,
                       start_joint_positions=start, jump_threshold=3.0,
                       avoid_collisions=True)
    if result.error_code.val != MoveItErrorCodes.SUCCESS:
        raise InvalidPlan(f'Cartesian error code: {result.error_code.val}')
    fraction = float(result.fraction)
    # Do not accept 99.9%: the omitted tail could contain a required endpoint.
    if not math.isfinite(fraction) or not 1.0-1e-9 <= fraction <= 1.0:
        raise InvalidPlan(f'Incomplete Cartesian path: {fraction:.6%}')
    end = check_trajectory(result.solution, start, args.max_joint_step)
    return result.solution, end


def check_transfer_travel(trajectory, start):
    """Reject gradual winding that the per-point jump check cannot detect."""
    jt = trajectory.joint_trajectory
    previous = list(start)
    travel = [0.0] * len(JOINTS)
    for point in jt.points:
        current = ordered_positions(jt.joint_names, point.positions)
        travel = [total + abs(a-b)
                  for total, a, b in zip(travel, current, previous)]
        previous = current
    # Conservative limit for a short transfer within this drawing, per joint.
    # This is path length, not merely endpoint displacement.
    if max(travel) > math.pi / 2:
        raise InvalidPlan(
            f'Pen-up transfer joint travel {max(travel):.3f} rad exceeds pi/2; '
            'no automatic MoveGroup fallback')
    return travel


def plan_stroke(node, points, start, args):
    first, last = points[0], points[-1]
    # Retain the exact goal orientation at all approach/lift waypoints.
    # The old make_pose() always reset orientation to quat_down().
    hover_start = copy.deepcopy(first)
    hover_start.position.z += 0.04
    hover_end = copy.deepcopy(last)
    hover_end.position.z += 0.04
    if start is not None:
        # Previous stroke ends at its hover point. Keep TCP orientation and
        # connect directly to the next hover point, with collision checking.
        # A partial/invalid connection stops preflight: never silently replace
        # it with a free joint-space motion that might wind a wrist around.
        print('Planning Cartesian pen-up transfer', flush=True)
        trajectory, end = plan_cartesian(node, [hover_start], start, args)
        travel = check_transfer_travel(trajectory, start)
        print(f'Pen-up transfer: max accumulated joint travel '
              f'{math.degrees(max(travel)):.1f} deg', flush=True)
        candidate = [('Cartesian pen-up transfer', trajectory)]
        phases = [('lower TCP', [first])]
        if len(points) > 1:
            phases.append(('draw', points[1:]))
        phases.append(('lift TCP', [hover_end]))
        for phase, waypoints in phases:
            trajectory, end = plan_cartesian(node, waypoints, end, args)
            candidate.append((phase, trajectory))
        return candidate, end
    for attempt in range(1, args.attempts + 1):
        phase = 'MoveGroup to start'
        try:
            result = move_group(node, pose=hover_start, execute=False,
                                planning_time=20.0,
                                start_joint_positions=start)
            if result.error_code.val != MoveItErrorCodes.SUCCESS:
                raise InvalidPlan(f'MoveGroup error: {result.error_code.val}')
            # First request uses the live state; use MoveIt's returned start
            # to check its first trajectory point as well.
            initial = start
            if initial is None:
                js = result.trajectory_start.joint_state
                initial = ordered_positions(js.name, js.position)
            trajectory = result.planned_trajectory
            end = check_trajectory(trajectory, initial, args.max_joint_step)
            candidate = [('move to start', trajectory)]
            phases = [('lower TCP', [first])]
            if len(points) > 1:
                phases.append(('draw', points[1:]))
            phases.append(('lift TCP', [hover_end]))
            for phase, waypoints in phases:
                trajectory, end = plan_cartesian(node, waypoints, end, args)
                candidate.append((phase, trajectory))
            return candidate, end
        except TimeoutError:
            # An action may still be running after a timeout: never retry it.
            raise
        except RuntimeError as exc:
            node.get_logger().warning(
                f'Attempt {attempt}/{args.attempts}, {phase}: {exc}')
    raise InvalidPlan('No complete candidate; no trajectories executed')


def require_mock_description(node):
    client = node.create_client(
        GetParameters, '/robot_state_publisher/get_parameters')
    try:
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('Cannot verify mock robot_description')
        request = GetParameters.Request()
        request.names = ['robot_description']
        response = wait_future(node, client.call_async(request), 15.0)
        if len(response.values) != 1 or response.values[0].type != 4:
            raise RuntimeError('robot_description must be a string parameter')
    finally:
        node.destroy_client(client)
    root = ET.fromstring(response.values[0].string_value)
    plugins = [p.text.strip() if p.text else ''
               for p in root.findall('./ros2_control/hardware/plugin')]
    if not plugins or any(p != 'mock_components/GenericSystem' for p in plugins):
        raise RuntimeError('Expected only mock_components/GenericSystem hardware')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--targets', default='/tmp/mas248_targets.json')
    parser.add_argument('--strokes', type=int, default=1)
    parser.add_argument('--attempts', type=int, default=5)
    parser.add_argument('--max-step', type=float, default=0.005)
    parser.add_argument('--max-joint-step', type=float, default=0.35,
                        help='Absolute per-joint step limit in radians')
    parser.add_argument('--execute-mock', action='store_true')
    args = parser.parse_args()
    if args.strokes < 1 or args.attempts < 1:
        parser.error('--strokes and --attempts must be positive')
    if not (math.isfinite(args.max_step) and 0 < args.max_step <= 0.01):
        parser.error('--max-step must be in (0, 0.01] metres')
    if not (math.isfinite(args.max_joint_step) and 0 < args.max_joint_step <= 0.35):
        parser.error('--max-joint-step must be in (0, 0.35] radians')
    with open(args.targets, encoding='utf-8') as file:
        strokes = json.load(file)['strokes']
    if len(strokes) < args.strokes:
        parser.error('Target file contains fewer strokes than requested')
    selected = []
    for stroke in strokes[:args.strokes]:
        poses = stroke['poses']
        if not poses:
            parser.error('Selected stroke is empty')
        xyz = [[float(p[k]) for k in ('x', 'y', 'z')] for p in poses]
        if not all(math.isfinite(v) for point in xyz for v in point):
            parser.error('Target coordinates must be finite')
        selected.append([make_pose(*point) for point in xyz])
    rclpy.init()
    node = Node('mas248_run_picture')
    try:
        if args.execute_mock:
            require_mock_description(node)
        chain, current = [], None
        for number, points in enumerate(selected, 1):
            print(f'Planning stroke {number}/{len(selected)}', flush=True)
            candidate, current = plan_stroke(node, points, current, args)
            chain.extend((f'stroke {number}: {label}', traj)
                         for label, traj in candidate)
        print('ALL SELECTED STROKES PLANNED', flush=True)
        if args.execute_mock:
            # Validate the entire chain before the first action is sent.
            for _, trajectory in chain:
                check_timing(trajectory)
            for label, trajectory in chain:
                print(f'Executing {label}', flush=True)
                # This wrapper already waits and raises on execution failure.
                # It returns None on success: do not test its truth value.
                execute_trajectory(node, trajectory)
            print('ALL SELECTED STROKES EXECUTED (MOCK)', flush=True)
        else:
            print('PLAN ONLY: no execution requests sent', flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
