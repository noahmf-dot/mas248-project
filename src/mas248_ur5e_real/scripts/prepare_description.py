#!/usr/bin/env python3
import argparse
import shutil
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
from ament_index_python.packages import get_package_share_directory

parser = argparse.ArgumentParser()
parser.add_argument('--source', required=True)
parser.add_argument('--output', required=True)
parser.add_argument('--config-output', required=True)
args = parser.parse_args()

source = Path(args.source)
output = Path(args.output)
config_output = Path(args.config_output)
root = ET.parse(source).getroot()

uri = 'http://www.ros.org/wiki/xacro'
ET.register_namespace('xacro', uri)
tag = lambda name: '{' + uri + '}' + name

robots = root.findall(tag('ur_robot'))
if len(robots) != 1:
    raise RuntimeError('Expected exactly one ur_robot macro')
robot = robots[0]

settings = yaml.safe_load(
    (Path(__file__).resolve().parents[1] /
     'config/real_robot_defaults.yaml').read_text()
)['hardware']

defaults = {
    'name': 'ur5e',
    'ur_type': 'ur5e',
    'tf_prefix': '',
    'robot_ip': '0.0.0.0',
    'use_fake_hardware': 'false',
    'kinematics_params': '',
    'joint_limit_params':
        '$(find mas248_ur5e_real)/config/ur5e/joint_limits.yaml',
    'physical_params':
        '$(find mas248_ur5e_real)/config/ur5e/physical_parameters.yaml',
    'visual_params':
        '$(find mas248_ur5e_real)/config/ur5e/visual_parameters.yaml',
    'script_filename':
        '$(find ur_client_library)/resources/external_control.urscript',
    'input_recipe_filename':
        '$(find ur_robot_driver)/resources/rtde_input_recipe.txt',
    'output_recipe_filename':
        '$(find ur_robot_driver)/resources/rtde_output_recipe.txt',
}
defaults.update(settings)

existing = {e.get('name'): e for e in root.findall(tag('arg'))}
for name, value in defaults.items():
    value = str(value).lower() if isinstance(value, bool) else str(value)
    if name in existing:
        existing[name].set('default', value)
    else:
        root.insert(0, ET.Element(tag('arg'),
                                 {'name': name, 'default': value}))

mapping = {
    'joint_limits_parameters_file': 'joint_limit_params',
    'kinematics_parameters_file': 'kinematics_params',
    'physical_parameters_file': 'physical_params',
    'visual_parameters_file': 'visual_params',
}
for name in (
    'name', 'ur_type', 'tf_prefix', 'robot_ip',
    'script_filename', 'input_recipe_filename', 'output_recipe_filename',
    *settings,
):
    mapping[name] = name

for attribute, argument in mapping.items():
    robot.set(attribute, '$(arg ' + argument + ')')

robot.set('generate_ros2_control_tag', 'true')
robot.set('use_fake_hardware', 'false')
robot.set('fake_sensor_commands', 'false')
robot.set('sim_gazebo', 'false')
robot.set('sim_ignition', 'false')

vendor = Path(get_package_share_directory('ur_description')) / 'config/ur5e'
config_output.mkdir(parents=True, exist_ok=True)
for name in ('joint_limits.yaml', 'physical_parameters.yaml',
             'visual_parameters.yaml', 'default_kinematics.yaml'):
    shutil.copy2(vendor / name, config_output / name)

output.parent.mkdir(parents=True, exist_ok=True)
ET.ElementTree(root).write(output, encoding='utf-8', xml_declaration=True)
print('Generated real workcell:', output)
