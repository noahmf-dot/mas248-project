
#!/usr/bin/env python3

import json
import argparse
import rclpy

from rclpy.node import Node
from common import JOINTS, make_pose, move_group, cartesian


def get_end_joints(trajectory):
    """Hent siste leddkonfigurasjon fra en planlagt bane."""
    jt = trajectory.joint_trajectory

    if not jt.points:
        raise RuntimeError("Planlagt bane er tom")

    final = dict(zip(jt.joint_names, jt.points[-1].positions))
    return [final[name] for name in JOINTS]


def plan_cartesian(node, waypoints, start_joints):
    """Planlegg en sammenhengende kartesisk bevegelse."""
    result = cartesian(
        node,
        waypoints,
        max_step=0.005,
        start_joint_positions=start_joints,
        jump_threshold=3.0,
	avoid_collisions=True
    )

    if result.fraction < 0.999:
        raise RuntimeError(
            f"Kartesisk bane ufullstendig: {result.fraction:.1%}"
        )

    return get_end_joints(result.solution)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--strokes", type=int, default=1)
    args = parser.parse_args()

    with open("/tmp/mas248_targets.json") as file:
        data = json.load(file)

    strokes = data["strokes"]

    rclpy.init()
    node = Node("mas248_run_picture")

    # 4 cm ekstra løftehøyde over hvert TCP-tegnepunkt
    lift_height = 0.04

    # Første plan starter fra robotens aktuelle tilstand
    current_joints = None

    try:
        count = min(args.strokes, len(strokes))

        for i, stroke in enumerate(strokes[:count]):
            poses = stroke["poses"]

            if not poses:
                continue

            print(f"\nTegnestrek {i + 1}/{count}")
            print(f"Antall punkter: {len(poses)}", flush=True)

            points = [
                make_pose(p["x"], p["y"], p["z"])
                for p in poses
            ]

            first = points[0]
            last = points[-1]

            # 1. Beveg roboten over første punkt
            hover_start = make_pose(
                first.position.x,
                first.position.y,
                first.position.z + lift_height
            )

            print("Planlegger bevegelse til startpunkt...", flush=True)

            result = move_group(
                node,
                pose=hover_start,
                execute=False,
                planning_time=20.0,
                start_joint_positions=current_joints
            )

            current_joints = get_end_joints(
                result.planned_trajectory
            )

            # 2. Senk TCP til tegnestrekens høyde
            print("Planlegger senking av TCP...", flush=True)

            current_joints = plan_cartesian(
                node, [first], current_joints
            )

            # 3. Følg konturen
            print("Planlegger tegnestrek...", flush=True)

            if len(points) > 1:
                current_joints = plan_cartesian(
                    node, points[1:], current_joints
                )

            # 4. Løft TCP etter tegnestreken
            hover_end = make_pose(
                last.position.x,
                last.position.y,
                last.position.z + lift_height
            )

            print("Planlegger løft av TCP...", flush=True)

            current_joints = plan_cartesian(
                node, [hover_end], current_joints
            )

            print(f"Tegnestrek {i + 1} planlagt!", flush=True)

        print("\nALLE VALGTE TEGNESTREKER PLANLAGT!")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
