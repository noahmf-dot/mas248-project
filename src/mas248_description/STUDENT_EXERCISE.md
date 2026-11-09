# MAS248 Lecture 8 — two-link robot exercise (students)

**Aim:** understand the relationship between URDF/Xacro, joint limits, fixed joints, TF2 and RViz2. **No physical robot is used.**

## Start

1. Run `ros2 launch mas248_description display.launch.py`.
2. Locate the two arm sliders `joint1` and `joint2`.
3. In a second terminal run `ros2 run tf2_ros tf2_echo base_link tcp`.
4. Move one slider at a time and describe how the TCP frame changes.

## Task A — change the second joint limit

Open `urdf/two_link_arm.urdf.xacro`, find the `<joint name="joint2" ...>` block and change its `<limit>` from ±1.57 rad to **lower = -0.80 rad, upper = +0.80 rad**.

- Do not change joint1.
- Build again, source the workspace, and restart the launch.
- Check the range of the `joint2` slider.
- Answer: why does the attainable TCP workspace change when we restrict the second joint?

## Task B — add a FIXED structure and its TF frame

In the same Xacro file, add a `fixture_link` with visible **box geometry**, for example `size="0.16 0.04 0.10"`, in a contrasting colour. Connect it to `base_link` with a `fixture_joint` of type `fixed`.

Use the following frame origin as a starting value:

```xml
<origin xyz="-0.12 0.22 0.12" rpy="0 0 0"/>
```

- The parent of `fixture_joint` must be `base_link`.
- The child must be `fixture_link`.
- The existing `tcp` and `tcp_joint` must remain unchanged.
- Check the new fixed frame with `ros2 run tf2_ros tf2_echo base_link fixture_link`.
- Move `joint1` and `joint2`; the frame `base_link → fixture_link` must **not** change.
- Check `ros2 topic echo /tf_static --once` or `ros2 run tf2_tools view_frames` to identify the fixed transform.

## Task C — explain what you observed

Answer in one or two sentences for each:

1. Which ROS2 node publishes joint states from the GUI?
2. Which ROS2 node generates `/tf` and `/tf_static` from the URDF and joint states?
3. Why is `fixture_link` static relative to `base_link` even when the TCP moves?
4. Does adding a fixed frame alone create a MoveIt planning-scene collision object? Why not?

## Deliverables

- Edited `two_link_arm.urdf.xacro`.
- RViz2 screenshot displaying both links, TCP and fixture.
- Screenshot or copied terminal output confirming `base_link → tcp` and `base_link → fixture_link` transforms.
- Short explanation of the new joint limit and fixed-frame behaviour.


```
