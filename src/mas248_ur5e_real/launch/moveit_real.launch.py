#!/usr/bin/env python3
"""Calibrated L8 workcell + unmodified L9 SRDF/OMPL/RViz for actual UR5e."""
from pathlib import Path
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
sys.path.insert(0,str(Path(__file__).parent))
from _real_common import (validate_ip, validate_calibration, render_description,
                          moveit_parameters, l9_config, HARDWARE_DEFAULTS)

def setup(context,*args,**kwargs):
    ip=validate_ip(LaunchConfiguration('robot_ip').perform(context))
    cal=validate_calibration(LaunchConfiguration('calibration').perform(context))
    ctl=LaunchConfiguration('joint_controller').perform(context)
    if ctl not in ('scaled_joint_trajectory_controller','joint_trajectory_controller'):
        raise RuntimeError('Unsupported MoveIt action controller: '+ctl)
    active=LaunchConfiguration('enable_execution').perform(context).lower()=='true'
    if active and ctl!='scaled_joint_trajectory_controller':
        raise RuntimeError('Physical execution requires scaled_joint_trajectory_controller')
    # This must be exactly the same Xacro argument set as the vendor driver.
    # In combined launch, system_real forwards one shared LaunchConfiguration
    # to both children. In separate launches users must pass matching overrides.
    hardware={name:LaunchConfiguration(name).perform(context) for name in HARDWARE_DEFAULTS}
    if hardware['safety_limits'].lower()!='true':
        raise RuntimeError('Physical description requires safety_limits:=true')
    if hardware['headless_mode'].lower()!='false':
        raise RuntimeError('Physical description expects pendant External Control (headless_mode:=false)')
    desc=render_description(ip,cal,**hardware)
    params=moveit_parameters(desc,ctl) # reads existing Lecture 9 YAML, SRDF, OMPL
    params['allow_trajectory_execution']=active
    params['trajectory_execution.allowed_execution_duration_scaling']=2.0
    params['trajectory_execution.allowed_goal_duration_margin']=1.0
    params['trajectory_execution.allowed_start_tolerance']=0.02
    # Execution-duration monitoring is valid with scaled control: do not disable casually.
    params['robot_description']=ParameterValue(desc,value_type=str)
    params['robot_description_semantic']=ParameterValue(params['robot_description_semantic'],value_type=str)
    nodes=[Node(package='moveit_ros_move_group',executable='move_group',output='screen',
                parameters=[params])]
    if LaunchConfiguration('launch_rviz').perform(context).lower()=='true':
        nodes.append(Node(package='rviz2',executable='rviz2',output='screen',
                          parameters=[params],
                          arguments=['-d',str(l9_config()/'mas248_moveit.rviz')]))
    return nodes

def generate_launch_description():
    return LaunchDescription([
      DeclareLaunchArgument('robot_ip',description='Real UR5e IP'),
      DeclareLaunchArgument('calibration',description='Actual UR5e calibration file'),
      DeclareLaunchArgument('joint_controller',default_value='scaled_joint_trajectory_controller'),
      DeclareLaunchArgument('enable_execution',default_value='false',choices=['true','false']),
      DeclareLaunchArgument('launch_rviz',default_value='true',choices=['true','false']),
      *[DeclareLaunchArgument(name,default_value=value) for name,value in HARDWARE_DEFAULTS.items()],
      OpaqueFunction(function=setup),
    ])
