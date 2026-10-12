import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_web_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'web'), glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vu',
    maintainer_email='quangvu82@gmail.com',
    description='Navigate the robot from a web page over a websocket (rosbridge)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'web_nav_bridge = robot_web_nav.web_nav_bridge:main',
        ],
    },
)
