from setuptools import find_packages, setup

package_name = 'hoverboard_laser'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vu',
    maintainer_email='quangvu82@gmail.com',
    description='LiDAR obstacle avoidance, following and warning for the hoverboard robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'laser_Avoidance = hoverboard_laser.laser_Avoidance:main',
        'laser_Tracker = hoverboard_laser.laser_Tracker:main',
        'laser_Warning = hoverboard_laser.laser_Warning:main',
        ],
    },
)
