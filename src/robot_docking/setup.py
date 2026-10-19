import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_docking'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vu',
    maintainer_email='quangvu82@gmail.com',
    description='Auto-docking for the hoverboard robot: saved map dock, Nav2 to the staging pose, reverse in',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'dock_manager = robot_docking.dock_manager:main',
        ],
    },
)
