from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'chess_vision'

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
    description='计算机视觉处理包，用于棋盘检测和棋子识别',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'board_detector = chess_vision.board_detector:main',
            'piece_detector = chess_vision.piece_detector:main',
            'vision_coordinator = chess_vision.vision_coordinator:main',
        ],
    },
)