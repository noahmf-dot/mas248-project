#!/usr/bin/env python3
"""Instructor-only, ROS 2 Humble UR5e physical driver.

Reuses L8 model (via derived xacro) + official UR driver's controllers.
No student fake controller, no changes to L8/L9.
Scaled trajectory controller ACTIVE by default after hardware acknowledgement;
pass enable_execution:=false to load it inactive for special diagnostics.
"""
from pathlib import Path
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
sys.path.insert(0, str(Path(__file__).parent))
from _real_common import (share, validate_ip, validate_calibration, render_description,
                          HARDWARE_DEFAULTS, DRIVER_DEFAULTS)

# A single source of defaults shared with MoveIt and system_real.
FORWARD = tuple(DRIVER_DEFAULTS)
HARDWARE_XACRO = set(HARDWARE_DEFAULTS)
DEFAULTS = DRIVER_DEFAULTS

def setup(context, *args, **kwargs):
    ip = validate_ip(LaunchConfiguration('robot_ip').perform(context))
    cal = validate_calibration(LaunchConfiguration('calibration').perform(context))
    if LaunchConfiguration('acknowledge_hardware').perform(context).lower() != 'yes':
        raise RuntimeError('Explicit acknowledge_hardware:=yes required for real hardware')
    ctl = LaunchConfiguration('joint_controller').perform(context)
    if ctl not in ('scaled_joint_trajectory_controller', 'joint_trajectory_controller'):
        raise RuntimeError('Only standard trajectory controllers supported; got ' + ctl)
    settings = {name:LaunchConfiguration(name).perform(context) for name in FORWARD}
    if settings['safety_limits'].lower() != 'true':
        raise RuntimeError('L10 instructor wrapper requires safety_limits:=true')
    if settings['headless_mode'].lower() != 'false':
        raise RuntimeError('Use pendant External Control: headless_mode must stay false')
    if LaunchConfiguration('enable_execution').perform(context).lower() == 'true' and ctl != 'scaled_joint_trajectory_controller':
        raise RuntimeError('Physical execution requires scaled_joint_trajectory_controller')
    # Local sanity validation before starting driver's networking/hardware.
    render_description(ip, cal, **{k:v for k,v in settings.items() if k in HARDWARE_XACRO})
    vendor_launch = share('ur_robot_driver') / 'launch' / 'ur_control.launch.py'
    if not vendor_launch.is_file():
        raise RuntimeError('Missing official Humble UR launch: ' + str(vendor_launch))
    supplied = {
        'ur_type':'ur5e', 'robot_ip':ip,
        'description_package':'mas248_ur5e_real',
        'description_file':'ur5e_workcell_real.urdf.xacro',
        'kinematics_params_file':cal,
        # Visible Lecture 10 YAML, derived from official Humble controller definitions.
        'runtime_config_package':'mas248_ur5e_real',
        'controllers_file':'ur_controllers.yaml',
        'tf_prefix':'',
        'use_fake_hardware':'false',
        'fake_sensor_commands':'false',
        'initial_joint_controller':ctl,
        'activate_joint_controller':LaunchConfiguration('enable_execution').perform(context),
        'launch_rviz':'false',
        **settings,
    }
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(vendor_launch)),
                                     launch_arguments=supplied.items())]

def generate_launch_description():
    args = [
        DeclareLaunchArgument('robot_ip', description='Actual UR5e IPv4 address'),
        DeclareLaunchArgument('calibration', description='Absolute YAML from ur_calibration'),
        DeclareLaunchArgument('joint_controller', default_value='scaled_joint_trajectory_controller',
                              description='Use scaled trajectory controller for physical movements'),
        DeclareLaunchArgument('enable_execution',default_value='true', choices=['true','false'],
                              description='true (default): scaled joint controller starts ACTIVE; false: starts INACTIVE. MoveIt execution is controlled separately.'),
        DeclareLaunchArgument('acknowledge_hardware',default_value='no',choices=['yes','no'],
                              description='Must explicitly acknowledge this connects to real robot'),
    ]
    args += [DeclareLaunchArgument(name,default_value=value) for name,value in DEFAULTS.items()]
    return LaunchDescription(args+[OpaqueFunction(function=setup)])
