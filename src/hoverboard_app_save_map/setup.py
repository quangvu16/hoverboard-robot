from setuptools import setup

package_name = 'hoverboard_app_save_map'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/hoverboard_app_save_map.launch.py',]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vu',
    maintainer_email='quangvu82@gmail.com',
    description='Save the current map on request (Yahboom app compatible service) and catalogue it',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        "server=hoverboard_app_save_map.hoverboard_app_save_map:main",
        "client=hoverboard_app_save_map.hoverboard_app_save_map_client:main"
        ],
    },
)
