from setuptools import find_packages
from setuptools import setup

setup(
    name='chess_coordinator',
    version='0.0.1',
    packages=find_packages(
        include=('chess_coordinator', 'chess_coordinator.*')),
)
