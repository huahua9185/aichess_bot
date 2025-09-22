from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'chess_arm'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.py'))),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chess Robot Team',
    maintainer_email='chess@robot.com',
    description='象棋机器人机械臂控制包，负责DOFBOT Pro 6DOF机械臂的控制和运动规划',
    license='MIT',
    entry_points={
        'console_scripts': [
            'arm_controller = chess_arm.arm_controller:main',
            'moveit_arm_controller = chess_arm.moveit_arm_controller:main',
        ],
    },
)