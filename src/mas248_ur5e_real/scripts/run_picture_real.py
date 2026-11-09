#!/usr/bin/env python3
"""Plan drawing paths using the real robot state. No execution requests."""
import argparse
import json
import math
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from common import make_pose, wait_future
from picture_planning import plan_stroke, check_timing


def verify_real_model(node):
    client = node.create_client(
        GetParameters, '/move_group/get_parameters')
    try:
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('MoveIt is unavailable')
        request = GetParameters.Request()
        request.names = ['robot_description', 'allow_trajectory_execution']
        response = wait_future(node, client.call_async(request), 15.0)
        if len(response.values) != 2:
            raise RuntimeError('Missing MoveIt parameters')
        description, execution = response.values
        if description.type != 4:
            raise RuntimeError('Missing robot_description')
        if execution.type != 1 or execution.bool_value:
            raise RuntimeError(
                'Start moveit_real.launch.py with enable_execution:=false')

        root = ET.fromstring(description.string_value)
        controls = root.findall('ros2_control')
        if len(controls) != 1:
            raise RuntimeError('Expected one hardware system')
        plugins = [
            (p.text or '').strip()
            for p in controls[0].findall('./hardware/plugin')
        ]
        if (len(plugins) != 1
                or not plugins[0].startswith('ur_robot_driver/')
                or 'mock' in plugins[0].lower()):
            raise RuntimeError('Expected real UR driver; found ' + repr(plugins))
    finally:
        node.destroy_client(client)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--targets', required=True)
    parser.add_argument('--strokes', type=int, default=1)
    parser.add_argument('--attempts', type=int, default=5)
    args = parser.parse_args()
    args.max_step = 0.001
    args.max_joint_step = 0.35

    if args.strokes < 1 or args.attempts < 1:
        parser.error('strokes and attempts must be positive')

    with open(args.targets, encoding='utf-8') as f:
        strokes = json.load(f)['strokes']
    if len(strokes) < args.strokes:
        parser.error('Target file contains fewer strokes than requested')

    selected = []
    for stroke in strokes[:args.strokes]:
        xyz = [
            [float(p[k]) for k in ('x', 'y', 'z')]
            for p in stroke['poses']
        ]
        if not xyz or not all(math.isfinite(v) for p in xyz for v in p):
            parser.error('Empty stroke or invalid coordinates')
        selected.append([make_pose(*p) for p in xyz])

    rclpy.init()
    node = Node('mas248_run_picture_real')
    try:
        verify_real_model(node)
        current = None
        chain = []
        for number, points in enumerate(selected, 1):
            print(f'Planning stroke {number}/{len(selected)}', flush=True)
            candidate, current = plan_stroke(node, points, current, args)
            chain.extend(candidate)
        for label, trajectory in chain:
            check_timing(trajectory)
        print('ALL SELECTED STROKES PLANNED')
        print('PLAN ONLY: no execution requests sent')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
