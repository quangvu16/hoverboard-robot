from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'hoverboard_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob(os.path.join('launch','*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vu',
    maintainer_email='quangvu82@gmail.com',
    description='Mapping, navigation and map saving launches for the hoverboard robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stop_car = hoverboard_nav.stop_car:main',
            'stop_robot1_car = hoverboard_nav.stop_car:main_robot1',
            'stop_robot2_car = hoverboard_nav.stop_car:main_robot2',
            'app_send_goal = hoverboard_nav.app_send_goal:main',
        ],
    },
)
