from setuptools import find_packages, setup

package_name = 'chess_game'

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
    description='Chess game logic and Stockfish engine integration',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'game_engine = chess_game.game_engine:main',
            'stockfish_engine = chess_game.stockfish_engine:main',
            'international_chess_engine = chess_game.international_chess_engine:main',
        ],
    },
)
