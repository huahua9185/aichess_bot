from setuptools import find_packages, setup

package_name = 'chess_vision'

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
    maintainer='jetson',
    maintainer_email='huahua9185@gmail.com',
    description='Computer vision for chess board detection and piece recognition',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'board_detector = chess_vision.board_detector:main',
            'vision_sim_node = chess_vision.international_chess_vision_sim:main',
        ],
    },
)
