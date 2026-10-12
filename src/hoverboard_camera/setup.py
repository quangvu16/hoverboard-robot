from setuptools import setup
import os
from glob import glob

package_name = 'hoverboard_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob(os.path.join('launch','*launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vu',
    maintainer_email='quangvu82@gmail.com',
    description='Camera source for the hoverboard vision packages, plus patrol and odometry calibration',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_source = hoverboard_camera.camera_source:main',
            'sub_img = hoverboard_camera.sub_img:main',
            'patrol = hoverboard_camera.patrol:main',
            'calibrate_linear = hoverboard_camera.calibrate_linear:main',
            'calibrate_angular = hoverboard_camera.calibrate_angular:main',
        ],
    },
)
