import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'hoverboard_ai_car'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'data'), glob(os.path.join('data', '*'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vu',
    maintainer_email='quangvu82@gmail.com',
    description='Camera-based behaviours for the hoverboard robot: colour/line/face/QR tracking, hand and gesture control',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'qrTracker = hoverboard_ai_car.qrTracker:main',
            'control_shape = hoverboard_ai_car.control_shape:main',
            'FingerCtrl = hoverboard_ai_car.FingerCtrl:main',
            'RobotCtrl = hoverboard_ai_car.RobotCtrl:main',
            'HandCtrl = hoverboard_ai_car.HandCtrl:main',
            'colorHSV = hoverboard_ai_car.colorHSV:main',
            'colorTracker = hoverboard_ai_car.colorTracker:main',
            'mono_Tracker = hoverboard_ai_car.mono_Tracker:main',
            'face_fllow = hoverboard_ai_car.face_fllow:main',
            'follow_line = hoverboard_ai_car.follow_line:main',
            
            
            
            
        ],
    },
)
