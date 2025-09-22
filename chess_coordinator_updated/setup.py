from setuptools import find_packages, setup

package_name = 'chess_coordinator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/chess_basic_test.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='huahua9185@gmail.com',
    description='System state machine and coordination controller',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'system_coordinator = chess_coordinator.system_coordinator:main',
            'chess_coordinator = chess_coordinator.international_chess_coordinator:main',
        ],
    },
)
