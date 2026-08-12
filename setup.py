import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'diy_autonomous_drone'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name,
            ['package.xml', 'README.md', 'requirements-vision.txt']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'docs'),
            glob(os.path.join('docs', '*.md'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dylan Finlay',
    maintainer_email='dylan.finlay33@gmail.com',
    description='ROS 2 nodes for a vision-guided DIY autonomous drone.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vision_node = diy_autonomous_drone.vision_node:main',
            'tracking_bridge_node = '
            'diy_autonomous_drone.tracking_bridge_node:main',
            'safety_supervisor_node = '
            'diy_autonomous_drone.safety_supervisor_node:main',
            'fc_interface_node = '
            'diy_autonomous_drone.fc_interface_node:main',
            'sitl_smoke_test = '
            'diy_autonomous_drone.sitl_smoke_test:main',
        ],
    },
)
