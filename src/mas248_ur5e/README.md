# MAS248 — UR5e workcell description (ROS2 Humble)

**ROS2 package name: `mas248_ur5e`** (always lower-case in ROS2 commands).
The ZIP folder uses the *same* name, so the source and installed package are easy to identify.

This is a **Lecture 8 visualisation/description** package. It instantiates the official
`ur_description` UR5e model and adds a pedestal, a spindle/tool and TCP, a worktable,
and an illustrative overhead camera frame. The camera is optional and is **not a
required student task**. Vendor files are never modified.

The workcell's `ur_robot` macro uses `generate_ros2_control_tag="false"`, intentionally:
this demonstration publishes transforms for RViz2 and joint-state sliders without
starting the real robot driver. **Do not use this launch file to execute physical
UR5e motion.** For Lecture 10, use the official calibrated UR ROS2 driver/External
Control setup and the corresponding MoveIt 2 configuration.

## 1. Install dependencies

Ubuntu 22.04, ROS2 Humble:

```bash
sudo apt update
sudo apt install ros-humble-ur ros-humble-xacro \
  ros-humble-robot-state-publisher ros-humble-joint-state-publisher-gui \
  ros-humble-rviz2 ros-humble-tf2-tools
```

If necessary, set up rosdep and check dependencies after copying the package.

## 2. Copy package to your ROS2 workspace

Make the resulting source path **exactly**:

```
~/mas248_ws/src/mas248_ur5e/package.xml
```

Avoid leaving an old copy in `src/` under another name, since that can cause
`Duplicate package names not supported` or ambiguous builds.

## 3. Build

In a **new terminal** (ROS2 Humble first):

```bash
source /opt/ros/humble/setup.bash
cd ~/mas248_ws
rosdep check --from-paths src/mas248_ur5e --ignore-src --rosdistro humble
colcon build --symlink-install --packages-select mas248_ur5e
source install/setup.bash
ros2 pkg prefix mas248_ur5e
```

## 4. Test Xacro independently

```bash
ros2 run xacro xacro \
  $(ros2 pkg prefix mas248_ur5e)/share/mas248_ur5e/urdf/ur5e_workcell.urdf.xacro \
  ur_type:=ur5e -o /tmp/ur5e_workcell.urdf
check_urdf /tmp/ur5e_workcell.urdf
```

If `check_urdf` is missing: `sudo apt install liburdfdom-tools`.
The Xacro file is evaluated using the installed vendor package from `/opt/ros/humble`.

## 5. Visualise (WSL2)

```bash
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch mas248_ur5e view_ur5e_workcell.launch.py
```

RViz2 uses the `world` Fixed Frame. The joint-state GUI moves the visualised arm;
no real motor is connected to this launch.

In another terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/mas248_ws/install/setup.bash
ros2 run tf2_ros tf2_echo world tcp
```

The installed `config/initial_positions.yaml` is a reference for later simulation
or mock-hardware setups. It is **not** loaded by this joint-state GUI launch.
The `srdf/` file is only a starter kinematic-group example, **not** a fully configured
MoveIt 2 SRDF. Do not use it directly for real-robot motion planning.


