#!/usr/bin/env python3
"""Physical UR5e drawing. Plan-only by default."""

import argparse
import json
import math
import os
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from rcl_interfaces.srv import GetParameters
from controller_manager_msgs.srv import ListControllers
from ur_dashboard_msgs.srv import (
    GetProgramState, GetRobotMode, GetSafetyMode
)
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import MoveItErrorCodes
from action_msgs.msg import GoalStatus

from common import make_pose, wait_future
from picture_planning import (
    plan_stroke, check_timing, ordered_positions
)
from test_ursim_motion import read_joints

MAX_START_ERROR = 0.02
MAX_END_ERROR = 0.03


def call_service(node, name, srv_type):
    client = node.create_client(srv_type, name)
    try:
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f"Service unavailable: {name}")
        return wait_future(
            node, client.call_async(srv_type.Request()), 15.0
        )
    finally:
        node.destroy_client(client)


def verify_model(node, execution_required):
    response = call_service(
        node, "/move_group/get_parameters", GetParameters
    )

    # This service needs parameter names, so use a dedicated request.
    return response


def verify_moveit(node, execution_required):
    client = node.create_client(
        GetParameters, "/move_group/get_parameters"
    )
    try:
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("MoveIt unavailable")

        req = GetParameters.Request()
        req.names = [
            "robot_description",
            "allow_trajectory_execution"
        ]

        response = wait_future(
            node, client.call_async(req), 15.0
        )

        if len(response.values) != 2:
            raise RuntimeError("Missing MoveIt parameters")

        description, execution = response.values

        if description.type != 4 or execution.type != 1:
            raise RuntimeError("Invalid MoveIt configuration")

        if execution_required and not execution.bool_value:
            raise RuntimeError(
                "Start MoveIt with enable_execution:=true"
            )

        root = ET.fromstring(description.string_value)
        controls = root.findall("ros2_control")

        if len(controls) != 1:
            raise RuntimeError("Unexpected ros2_control configuration")

        plugins = [
            (p.text or "").strip()
            for p in controls[0].findall("./hardware/plugin")
        ]

        if (
            len(plugins) != 1
            or not plugins[0].startswith("ur_robot_driver/")
            or "mock" in plugins[0].lower()
        ):
            raise RuntimeError(
                f"Expected UR hardware driver, got {plugins}"
            )

        print("MoveIt hardware model: OK")
        print("Execution enabled:", execution.bool_value)

    finally:
        node.destroy_client(client)


def verify_robot_ready(node):
    program = call_service(
        node, "/dashboard_client/program_state",
        GetProgramState
    )
    robot = call_service(
        node, "/dashboard_client/get_robot_mode",
        GetRobotMode
    )
    safety = call_service(
        node, "/dashboard_client/get_safety_mode",
        GetSafetyMode
    )
    controllers = call_service(
        node, "/controller_manager/list_controllers",
        ListControllers
    )

    if program.state.state != "PLAYING":
        raise RuntimeError("External Control is not playing")

    if robot.robot_mode.mode != 7:
        raise RuntimeError("Robot is not in RUNNING mode")

    if safety.safety_mode.mode != 1:
        raise RuntimeError("Safety mode is not NORMAL")

    active = [
        c.name for c in controllers.controller
        if c.state == "active"
    ]

    if "scaled_joint_trajectory_controller" not in active:
        raise RuntimeError("Scaled trajectory controller inactive")

    print("Robot, safety and controller checks: OK")


def slow_trajectory(trajectory, factor):
    points = trajectory.joint_trajectory.points

    if not points:
        raise RuntimeError("Empty trajectory")

    for point in points:
        stamp = point.time_from_start
        ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        new_ns = round(ns * factor)
        stamp.sec, stamp.nanosec = divmod(new_ns, 1_000_000_000)

        if point.velocities:
            point.velocities = [v / factor for v in point.velocities]

        if point.accelerations:
            point.accelerations = [
                a / (factor * factor)
                for a in point.accelerations
            ]


def positions(trajectory, first):
    jt = trajectory.joint_trajectory
    point = jt.points[0] if first else jt.points[-1]
    return ordered_positions(jt.joint_names, point.positions)


def joint_error(actual, expected):
    if len(actual) != len(expected):
        raise RuntimeError("Joint count mismatch")
    return max(abs(a - b) for a, b in zip(actual, expected))


def execute_trajectory(node, trajectory):
    client = ActionClient(
        node, ExecuteTrajectory, "/execute_trajectory"
    )

    if not client.wait_for_server(timeout_sec=10.0):
        raise RuntimeError("ExecuteTrajectory unavailable")

    goal = ExecuteTrajectory.Goal()
    goal.trajectory = trajectory

    handle = wait_future(
        node, client.send_goal_async(goal), 30.0
    )

    if not handle.accepted:
        raise RuntimeError("Trajectory rejected")

    end = trajectory.joint_trajectory.points[-1].time_from_start
    duration = end.sec + end.nanosec * 1e-9

    # Allows for speed reduction on the teach pendant.
    timeout = max(180.0, 30.0 * duration + 60.0)

    try:
        result = wait_future(
            node, handle.get_result_async(), timeout
        )
    except (TimeoutError, KeyboardInterrupt):
        try:
            wait_future(node, handle.cancel_goal_async(), 10.0)
        except Exception:
            pass
        raise RuntimeError(
            "Execution interrupted. Verify robot stopped before retry."
        )

    if (
        result.status != GoalStatus.STATUS_SUCCEEDED
        or result.result.error_code.val != MoveItErrorCodes.SUCCESS
    ):
        raise RuntimeError(
            f"Execution failed: status={result.status}, "
            f"code={result.result.error_code.val}"
        )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--targets", required=True)
    parser.add_argument("--strokes", type=int, default=1)
    parser.add_argument("--attempts", type=int, default=5)
    parser.add_argument("--height-offset", type=float, default=0.10)
    parser.add_argument("--slowdown", type=float, default=3.0)
    parser.add_argument("--flip-tool", action="store_true",
                        help="Rotate drawing target orientation 180 degrees around X; test in air first")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--segments", type=int, default=1)
    parser.add_argument("--confirm-real", action="store_true")
    parser.add_argument("--allow-multiple-segments", action="store_true")
    args = parser.parse_args()

    if (
        args.strokes < 1
        or args.segments < 1
        or args.attempts < 1
        or not math.isfinite(args.height_offset)
        or args.height_offset < 0
        or not math.isfinite(args.slowdown)
        or args.slowdown < 1
    ):
        parser.error("Invalid arguments")

    if args.execute and not args.confirm_real:
        parser.error("--confirm-real required for execution")

    if args.execute and args.segments > 1:
        if not args.allow_multiple_segments:
            parser.error(
                "--allow-multiple-segments required"
            )

    args.max_step = 0.001
    args.max_joint_step = 0.35

    with open(args.targets, encoding="utf-8") as f:
        strokes = json.load(f)["strokes"]

    if args.strokes > len(strokes):
        parser.error("Not enough strokes in target file")

    selected = []

    for stroke in strokes[:args.strokes]:
        poses = []

        for p in stroke["poses"]:
            x, y, z = (
                float(p[k]) for k in ("x", "y", "z")
            )

            if not all(math.isfinite(v) for v in (x, y, z)):
                raise RuntimeError("Invalid target coordinates")

            pose = make_pose(x, y, z + args.height_offset)

            # Optional 180-degree change to drawing goal orientation only.
            # Preserve the measured pen TCP transform in URDF/Xacro.
            if args.flip_tool:
                pose.orientation.x = 0.0
                pose.orientation.y = 0.0
                pose.orientation.z = 0.0
                pose.orientation.w = 1.0

            poses.append(pose)

        if not poses:
            raise RuntimeError("Empty stroke")

        selected.append(poses)

    if os.environ.get("ROS_DOMAIN_ID") != "48":
        raise RuntimeError("Expected ROS_DOMAIN_ID=48")

    print("Selected strokes:", len(selected))
    print("Height offset:", args.height_offset, "m")
    print("Trajectory slowdown:", args.slowdown)
    print("Drawing orientation:", "FLIPPED 180 deg about X" if args.flip_tool else "original")
    print("Execution requested:", args.execute)

    rclpy.init()
    node = Node("mas248_physical_drawing")

    try:
        verify_moveit(node, args.execute)

        initial = read_joints(node)
        chain = []
        planned_end = None

        for i, points in enumerate(selected, 1):
            print(f"Planning stroke {i}/{len(selected)}")
            candidate, planned_end = plan_stroke(
                node, points, planned_end, args
            )
            chain.extend(candidate)

        if not chain:
            raise RuntimeError("No trajectories planned")

        previous = None

        for label, trajectory in chain:
            slow_trajectory(trajectory, args.slowdown)
            check_timing(trajectory)

            start = positions(trajectory, True)

            if previous is not None:
                if joint_error(start, previous) > 0.02:
                    raise RuntimeError("Trajectory discontinuity")

            previous = positions(trajectory, False)
            print(f"Validated: {label}")

        if joint_error(
            initial, positions(chain[0][1], True)
        ) > MAX_START_ERROR:
            raise RuntimeError(
                "Initial robot position does not match first trajectory"
            )

        print("ALL SELECTED STROKES PLANNED AND VALIDATED")

        if not args.execute:
            print("PLAN ONLY: NO ROBOT MOVEMENT")
            return

        if args.segments > len(chain):
            raise RuntimeError("Requested too many segments")

        verify_robot_ready(node)

        print(f"Will execute {args.segments}/{len(chain)} segments")
        print("Physical robot must be supervised.")
        print("Verify TCP, workcell, clearance and pendant speed.")

        phrase = (
            "REAL-UR5E-MULTI"
            if args.segments > 1
            else "REAL-UR5E"
        )

        confirmation = input(f"Type {phrase} to execute: ")

        if confirmation != phrase:
            print("Execution cancelled")
            return

        for i, (label, trajectory) in enumerate(
            chain[:args.segments], 1
        ):
            verify_robot_ready(node)

            actual = read_joints(node)
            start = positions(trajectory, True)

            if joint_error(actual, start) > MAX_START_ERROR:
                raise RuntimeError(
                    "Robot start position mismatch"
                )

            print(f"Executing {i}/{args.segments}: {label}")
            execute_trajectory(node, trajectory)

            actual_end = read_joints(node)
            expected_end = positions(trajectory, False)

            if joint_error(
                actual_end, expected_end
            ) > MAX_END_ERROR:
                raise RuntimeError(
                    "Robot final position mismatch"
                )

            print(f"Completed: {label}")

        print("SELECTED PHYSICAL SEGMENTS COMPLETED")

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
