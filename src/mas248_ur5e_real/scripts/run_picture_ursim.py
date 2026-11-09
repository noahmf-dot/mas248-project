#!/usr/bin/env python3
"""URSim-only air drawing. Plan-only unless --execute is given."""

import argparse
import json
import math
import os
from pathlib import Path
from types import SimpleNamespace

import rclpy
from rclpy.node import Node

from common import make_pose
from picture_planning import (
    plan_stroke, check_timing, ordered_positions
)
from test_ursim_motion import (
    verify_simulator, verify_ros, read_joints, execute
)

TARGET_FILE = Path.home() / "mas248_ws/pumpkin_air.json"
MAX_START_ERROR = 0.02
MAX_END_ERROR = 0.03



def slow_first_approach(trajectory, factor=3.0):
    """Retiming of first approach without changing joint positions."""
    if factor <= 1.0:
        raise ValueError("Time scaling factor must exceed 1")

    points = trajectory.joint_trajectory.points
    if not points:
        raise RuntimeError("Empty approach trajectory")

    for point in points:
        stamp = point.time_from_start
        old_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        new_ns = round(old_ns * factor)

        stamp.sec, stamp.nanosec = divmod(new_ns, 1_000_000_000)

        if point.velocities:
            point.velocities = [v / factor for v in point.velocities]

        if point.accelerations:
            point.accelerations = [a / (factor ** 2) for a in point.accelerations]

    last = points[-1].time_from_start
    duration = last.sec + last.nanosec * 1e-9

    print(f"Trajectory slowed {factor}x")
    print(f"New trajectory duration: {duration:.2f} seconds")


def joint_error(actual, expected):
    return max(abs(a - b) for a, b in zip(actual, expected))


def trajectory_start(trajectory):
    jt = trajectory.joint_trajectory
    return ordered_positions(jt.joint_names, jt.points[0].positions)


def trajectory_end(trajectory):
    jt = trajectory.joint_trajectory
    return ordered_positions(jt.joint_names, jt.points[-1].positions)


def load_targets():
    with open(TARGET_FILE, encoding="utf-8") as file:
        strokes = json.load(file)["strokes"]

    if len(strokes) != 8:
        raise RuntimeError("Expected 8 drawing contours")

    points = strokes[0]["poses"]

    if len(points) != 100:
        raise RuntimeError("Unexpected first contour size")

    poses = []

    for point in points:
        x, y, z = (
            float(point[k]) for k in ("x", "y", "z")
        )

        if not all(math.isfinite(v) for v in (x, y, z)):
            raise RuntimeError("Invalid drawing coordinate")

        if abs(z - 0.030) > 0.0001:
            raise RuntimeError("This test requires Z = 30 mm")

        if abs(x - 0.5785) > 0.095:
            raise RuntimeError("Drawing X outside expected area")

        if abs(y + 0.085) > 0.095:
            raise RuntimeError("Drawing Y outside expected area")

        poses.append(make_pose(x, y, z))

    print("Loaded first contour:", len(poses), "points")
    print("Drawing height: 30 mm above paper")

    return poses


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()

    if os.environ.get("ROS_DOMAIN_ID") != "48":
        raise RuntimeError("Expected ROS_DOMAIN_ID=48")

    verify_simulator()

    poses = load_targets()

    settings = SimpleNamespace(
        max_step=0.001,
        max_joint_step=0.35,
        attempts=5
    )

    rclpy.init()
    node = Node("mas248_ursim_air_drawing")

    try:
        verify_ros(node)

        initial = read_joints(node)

        print("Planning first air-drawing contour...", flush=True)

        chain, planned_end = plan_stroke(
            node, poses, None, settings
        )

        if not chain or chain[0][0] != "move to start":
            raise RuntimeError("Unexpected first trajectory phase")

        for label, trajectory in chain:
            slow_first_approach(trajectory, factor=3.0)

        previous = None

        for label, trajectory in chain:
            check_timing(trajectory)

            start = trajectory_start(trajectory)

            if previous is not None:
                if joint_error(start, previous) > 0.02:
                    raise RuntimeError(
                        "Discontinuity between trajectory segments"
                    )

            previous = trajectory_end(trajectory)

            print(
                f"Validated {label}: "
                f"{len(trajectory.joint_trajectory.points)} points"
            )

        if joint_error(
            initial, trajectory_start(chain[0][1])
        ) > MAX_START_ERROR:
            raise RuntimeError("Initial robot state mismatch")

        print("ALL TRAJECTORIES PLANNED AND VALIDATED")

        if not args.execute:
            print("PLAN ONLY: no robot movement")
            return

        verify_simulator()
        verify_ros(node)

        current = read_joints(node)

        if joint_error(
            current, trajectory_start(chain[0][1])
        ) > MAX_START_ERROR:
            raise RuntimeError("Robot moved since planning")

        confirmation = input(
            "Type AIR-URSIM to execute first contour: "
        )

        if confirmation != "AIR-URSIM":
            print("Execution cancelled")
            return

        for number, (label, trajectory) in enumerate(chain, 1):
            verify_simulator()
            verify_ros(node)

            actual = read_joints(node)
            expected_start = trajectory_start(trajectory)

            if joint_error(actual, expected_start) > MAX_START_ERROR:
                raise RuntimeError(
                    f"Start position mismatch in segment {number}"
                )

            print(
                f"Executing {number}/{len(chain)}: {label}",
                flush=True
            )

            execute(node, trajectory)

            actual_end = read_joints(node)
            expected_end = trajectory_end(trajectory)

            if joint_error(actual_end, expected_end) > MAX_END_ERROR:
                raise RuntimeError(
                    f"Final position mismatch in segment {number}"
                )

            print(f"Completed: {label}", flush=True)


        print("FIRST AIR-DRAWING CONTOUR EXECUTED SUCCESSFULLY")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
