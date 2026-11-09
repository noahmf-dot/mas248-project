from glob import glob
from setuptools import setup

package_name = 'mas248_ur5e_demos'

setup(
    name=package_name,
    version='0.4.0',
    # The actual importable Python package is the scripts/ folder.
    # The ROS package name remains mas248_ur5e_demos.
    # A standard package layout is required for colcon --symlink-install.
    packages=['scripts'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/images',
         glob('images/*.png') + glob('images/*.jpg') + glob('images/*.jpeg')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MAS248',
    maintainer_email='mas248@example.invalid',
    description='MAS248 Lecture 9 student starter: image-centred TCP targets, run_picture student assignment',
    license='BSD-3-Clause',
    entry_points={'console_scripts': [
        'add_cube = scripts.add_cube:main',
        'image_to_targets = scripts.image_to_targets:main',
        'show_targets_rviz = scripts.show_targets_rviz:main',
        'move_two_targets = scripts.move_two_targets:main',
        'check_stack = scripts.check_stack:main',
    ]},
)
