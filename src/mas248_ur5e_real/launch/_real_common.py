"""MAS248 L10 Humble-specific real UR5e model, reusing lecture 9 planning files."""
from pathlib import Path
from copy import deepcopy
import yaml
import ipaddress
import xml.etree.ElementTree as ET

# Hardware description arguments shared by BOTH launches and by the combined launcher.
# Keep aligned with the Humble ur_control.launch.py interface. Do not duplicate defaults.
# Config located alongside the launch directory in both src/ and install/share/.
_defaults_path = Path(__file__).resolve().parent.parent / 'config' / 'real_robot_defaults.yaml'
if not _defaults_path.is_file():
    raise RuntimeError('Missing Lecture 10 configuration: '+str(_defaults_path))
_defaults = yaml.safe_load(_defaults_path.read_text(encoding='utf-8'))
HARDWARE_DEFAULTS = {k:str(v).lower() if isinstance(v,bool) else str(v)
                     for k,v in _defaults['hardware'].items()}
DRIVER_ONLY_DEFAULTS = {k:str(v).lower() if isinstance(v,bool) else str(v)
                        for k,v in _defaults['driver_only'].items()}
DRIVER_DEFAULTS = {**HARDWARE_DEFAULTS, **DRIVER_ONLY_DEFAULTS}

JOINTS=('shoulder_pan_joint','shoulder_lift_joint','elbow_joint',
        'wrist_1_joint','wrist_2_joint','wrist_3_joint')


def share(pkg):
 from ament_index_python.packages import get_package_share_directory
 return Path(get_package_share_directory(pkg))


def validate_ip(value):
 try: addr=ipaddress.IPv4Address(value)
 except ipaddress.AddressValueError as exc: raise RuntimeError('Invalid robot IPv4 address: '+value) from exc
 if addr.is_unspecified or addr.is_multicast or addr.is_loopback or addr.is_link_local or addr.is_reserved:
  raise RuntimeError('Not a valid physical robot IP: '+value)
 return str(addr)


def validate_calibration(value):
 import yaml
 p=Path(value).expanduser().resolve()
 if not p.is_file():raise RuntimeError('Missing robot calibration YAML: '+str(p))
 if 'default_kinematics' in p.name.lower():raise RuntimeError('Use actual robot calibration, not default_kinematics.yaml')
 d=yaml.safe_load(p.read_text(encoding='utf-8'))
 if not isinstance(d,dict) or not isinstance(d.get('kinematics'),dict) or not d['kinematics'].get('hash'):
  raise RuntimeError('Calibration missing the kinematics hash')
 return str(p)


def description_file():
 return share('mas248_ur5e_real')/'urdf'/'ur5e_workcell_real.urdf.xacro'


def l9_config():
 return share('mas248_ur5e_moveit_config')/'config'


def render_description(robot_ip,calibration,**hardware_arguments):
 import xacro
 ip=validate_ip(robot_ip)
 cal=validate_calibration(calibration)
 desc=description_file()
 if not desc.is_file():raise RuntimeError('Generated L10 workcell missing: '+str(desc)+'; rebuild L10')
 lc=share('mas248_ur5e_real')/'config'/'ur5e'
 mappings={
  'ur_type':'ur5e','robot_ip':ip,'name':'ur5e','tf_prefix':'',
  'use_fake_hardware':'false','mock_control':'false','headless_mode':'false',
  'kinematics_params':cal,
  'joint_limit_params':str(lc/'joint_limits.yaml'),
  'physical_params':str(lc/'physical_parameters.yaml'),
  'visual_params':str(lc/'visual_parameters.yaml'),
  'script_filename':str(share('ur_client_library')/'resources'/'external_control.urscript'),
  'input_recipe_filename':str(share('ur_robot_driver')/'resources'/'rtde_input_recipe.txt'),
  'output_recipe_filename':str(share('ur_robot_driver')/'resources'/'rtde_output_recipe.txt'),
  'verify_robot_model':'true',
 }
 # All names here are top-level args in the generated Xacro.
 # This prevents a non-default driver TCP port or reverse_ip from being silently ignored.
 allowed=set(HARDWARE_DEFAULTS)
 unknown=set(hardware_arguments)-allowed
 if unknown:raise RuntimeError('Unknown hardware description arguments: '+str(sorted(unknown)))
 mappings.update({k:str(v).lower() if isinstance(v,bool) else str(v) for k,v in hardware_arguments.items()})
 desc_xml=xacro.process_file(str(desc),mappings=mappings).toxml()
 validate_rendered(desc_xml)
 return desc_xml


def validate_rendered(xml):
 root=ET.fromstring(xml)
 if root.get('name')!='mas248_ur5e':raise RuntimeError('Unexpected robot name in rendered URDF')
 links={e.get('name') for e in root.findall('link')}
 required={'world','pedestal_link','base_link','spindle','tcp'}
 if not required.issubset(links):raise RuntimeError('Missing workcell links: '+repr(sorted(required-links)))
 joints={e.get('name') for e in root.findall('joint')}
 if not set(JOINTS).issubset(joints):raise RuntimeError('Missing joints: '+repr(sorted(set(JOINTS)-joints)))
 controls=root.findall('ros2_control')
 if len(controls)!=1:raise RuntimeError('Expected precisely 1 real ros2_control, found '+str(len(controls)))
 plugins=[(p.text or '').strip() for p in controls[0].findall('.//hardware/plugin')]
 if len(plugins)!=1 or 'ur_robot_driver/' not in plugins[0] or 'mock' in plugins[0].lower():
  raise RuntimeError('Vendor real hardware expected; got '+repr(plugins))
 return True


def read_yaml(n):
 import yaml
 data=yaml.safe_load((l9_config()/n).read_text(encoding='utf-8'))
 if not isinstance(data,dict):raise RuntimeError('Invalid YAML: '+n)
 return data


def moveit_parameters(description,controller):
 srdf=(l9_config()/'ur5e_workcell.srdf').read_text(encoding='utf-8')
 root=ET.fromstring(srdf)
 if root.get('name')!='mas248_ur5e':raise RuntimeError('SRDF mismatch')
 group=root.find("./group[@name='manipulator']/chain")
 if group is None or group.get('tip_link')!='tcp':raise RuntimeError('Expected manipulator group ending at tcp')
 l9=read_yaml('moveit_controllers.yaml')['moveit_simple_controller_manager']
 source=share('mas248_ur5e_real')/'config'/'moveit_real_controllers.yaml'
 m=deepcopy(yaml.safe_load(source.read_text(encoding='utf-8'))['moveit_simple_controller_manager'])
 if controller not in m['controller_names'] or controller not in m:
  raise RuntimeError('Controller missing in Lecture 10 real MoveIt configuration: '+controller)
 if m[controller]['joints'] != l9[l9['controller_names'][0]]['joints']:
  raise RuntimeError('L9/L10 MoveIt joint list mismatch')
 m['controller_names']=[controller]
 m[controller]['default']=True
 for other in set(('scaled_joint_trajectory_controller','joint_trajectory_controller'))-{controller}:
  m.pop(other,None)
 return {
  'robot_description':description,
  'robot_description_semantic':srdf,
  'robot_description_kinematics':read_yaml('kinematics.yaml'),
  'robot_description_planning':read_yaml('joint_limits.yaml'),
  'planning_pipelines':['ompl'],'default_planning_pipeline':'ompl',
  'ompl':read_yaml('ompl_planning.yaml'),
  'moveit_controller_manager':'moveit_simple_controller_manager/MoveItSimpleControllerManager',
  'moveit_simple_controller_manager':m,
  'moveit_manage_controllers':False,
  'allow_trajectory_execution':False,
  'publish_robot_description':True,'publish_robot_description_semantic':True,
  'publish_planning_scene':True,'publish_geometry_updates':True,
  'publish_state_updates':True,'publish_transforms_updates':True,
 }
