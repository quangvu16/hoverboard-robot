from setuptools import setup
import os
from glob import glob
package_name = 'hoverboard_multi'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob(os.path.join('launch','*.launch.xml'))),
        (os.path.join('share',package_name,'launch'),glob(os.path.join('launch','*launch.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vu',
    maintainer_email='quangvu82@gmail.com',
    description='Several hoverboard robots: namespaced bringup and Nav2, formation driving, multi-robot control',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'formation_leader = hoverboard_multi.formation_leader:main',
            'formation_follower = hoverboard_multi.formation_follower:main',
            'tf_relay = hoverboard_multi.tf_relay:main',
            'multi_keyboard_ctrl = hoverboard_multi.multi_ctrl:keyboard',
        ],
    },
)
