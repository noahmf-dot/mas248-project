# MAS248 UR5e drawing workspace

Source copied from the 5 November laboratory backup. The original archive is preserved separately. A fresh build and mock execution were verified on Ubuntu 22.04.5 LTS with ROS 2 Humble.

## Workspace

- `src/mas248_ur5e`: workcell model.
- `src/mas248_ur5e_moveit_config`: mock control and MoveIt launch/configuration.
- `src/mas248_ur5e_demos`: image conversion and drawing tools.
- `src/mas248_ur5e_real`: physical UR driver integration and drawing scripts.
- `src/mas248_description`, `src/mas248_project` and `src/mas248_ur5e_moveit_test_config`: earlier course/workspace packages preserved from the backup.
- `data/`: separate recorded mock and physical target files.

## Setup and verification

On Ubuntu with ROS 2 Humble, open a terminal in this repository. Confirm that the UR driver and MoveIt versions match the laboratory setup. Resolve the dependencies declared by the packages, then build the workspace.

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

For the mock setup, see `src/MAS248_L9_Student_Image_Centered_Guide.txt` and the package README. Use the mock launch files and `data/mock_targets.json`. The archived `scripts/run_picture_smooth.py` defaults to plan-only and has a separate `--execute-mock` option. The script checks for the `mock_components/GenericSystem` hardware plugin before mock execution.

From separate terminals with the workspace sourced, start mock control and MoveIt respectively.

```bash
ros2 launch mas248_ur5e_moveit_config ros2_control.launch.py
```

```bash
ros2 launch mas248_ur5e_moveit_config moveit.launch.py
```

In a third sourced terminal, run the plan-only command from the repository root.

```bash
python3 src/mas248_ur5e_demos/scripts/run_picture_smooth.py \
  --targets data/mock_targets.json --strokes 4
```

### Verified run on 9 November 2026

A clean source archive of commit `d9428a9` built all seven packages. All 12 offline Python tests passed. Running the drawing script with `--execute-mock` completed all four contours and exited successfully. This exercises planning and the simulated joint trajectory controller. The mock target plane is at `world` Z = 0.49 m and is separate from the paper mesh in the workcell model.

RViz displayed the robot model, but its MotionPlanning panel reported a joint-limit parameter type mismatch and could not render the planned trajectories during this run. To view the **target contours** separately, run the following from a sourced terminal and add a `MarkerArray` display in RViz for `/mas248/drawing_targets` with fixed frame `world`.

```bash
ros2 run mas248_ur5e_demos show_targets_rviz --json data/mock_targets.json
```

The physical package has separate driver and MoveIt launches and a plan-only default. Use the approved laboratory setup and matching calibration. Keep the robot-specific calibration separate from the Git repository; it is preserved in the local Results folder. Do not reuse mock coordinates on the physical robot. Final live timing parameters were not preserved in the success note.

The operator reported using `--height-offset 0.164` m for the successful physical run. The physical target file in `data/` contains the input targets, to which the execution script applies that offset.

## Before publishing

Fill in group contributions and distinguish supplied teaching code, vendor dependencies and group changes. Check course permission and applicable licences before sharing. Keep the raw videos and original backup outside this code repository.
