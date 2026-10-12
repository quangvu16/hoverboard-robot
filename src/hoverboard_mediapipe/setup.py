import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'hoverboard_mediapipe'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'data'), [f for f in glob('data/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'data', 'img'), glob('data/img/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vu',
    maintainer_email='quangvu82@gmail.com',
    description='MediaPipe hand/pose/face demos for the hoverboard robot camera',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            '01_HandDetector = hoverboard_mediapipe.01_HandDetector:main',
            '02_PoseDetector = hoverboard_mediapipe.02_PoseDetector:main',
            '03_Holistic = hoverboard_mediapipe.03_Holistic:main',
            '04_FaceMesh = hoverboard_mediapipe.04_FaceMesh:main',
            '05_FaceEyeDetection = hoverboard_mediapipe.05_FaceEyeDetection:main',
            '06_FaceLandmarks = hoverboard_mediapipe.06_FaceLandmarks:main',
            '07_FaceDetection = hoverboard_mediapipe.07_FaceDetection:main',
            '08_Objectron = hoverboard_mediapipe.08_Objectron:main',
            '09_VirtualPaint = hoverboard_mediapipe.09_VirtualPaint:main',
            '10_HandCtrl = hoverboard_mediapipe.10_HandCtrl:main',
            '11_GestureRecognition = hoverboard_mediapipe.11_GestureRecognition:main',
            'control_servo = hoverboard_mediapipe.control_servo:main',
            
            
            
        ],
    },
)
