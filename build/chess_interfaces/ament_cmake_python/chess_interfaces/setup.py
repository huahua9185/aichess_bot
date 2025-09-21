from setuptools import find_packages
from setuptools import setup

setup(
    name='chess_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('chess_interfaces', 'chess_interfaces.*')),
)
