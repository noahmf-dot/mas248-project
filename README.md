# MAS248 UR5e drawing workspace

Source copied from the 5 November laboratory backup. Generated Python caches and backup variants were omitted from this copy. The original archive is preserved separately. The workspace has been inspected on Windows; ROS build and execution still need verification on Ubuntu with ROS 2 Humble. No robot motion was run while preparing this folder.

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

For the mock setup, see `src/MAS248_L9_Student_Image_Centered_Guide.txt` and the package README. Use the mock launch files and `data/mock_targets.json`. The archived `scripts/run_picture_smooth.py` defaults to plan-only and has a separate `--execute-mock` option. Confirm that the actual loaded hardware plugin is the mock system before executing. Record a fresh build and planning/mock result here before submission.

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

The physical package has separate driver and MoveIt launches and a plan-only default. Use the approved laboratory setup and matching calibration. Keep the robot-specific calibration separate from the Git repository; it is preserved in the local Results folder. Do not reuse mock coordinates on the physical robot. Final live timing parameters were not preserved in the success note.

The operator reported using `--height-offset 0.164` m for the successful physical run. The physical target file in `data/` contains the input targets, to which the execution script applies that offset.

## Before publishing

Fill in group contributions and distinguish supplied teaching code, vendor dependencies and group changes. Check course permission and applicable licences before sharing. Confirm ROS build/test status and remove any unnecessary files from the intended Git commit. Keep the raw videos and original backup outside this code repository.
