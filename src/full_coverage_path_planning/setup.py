import os
from glob import glob

from setuptools import setup

package_name = 'full_coverage_path_planning'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vu',
    maintainer_email='quangvu82@gmail.com',
    description='Full coverage path planning for the hoverboard robot (ROS 2 port of globaledgesoft FCPP-on-TurtleBot3)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'coverage_runner = full_coverage_path_planning.coverage_runner:main',
            'coverage_progress = full_coverage_path_planning.coverage_progress:main',
        ],
    },
)
