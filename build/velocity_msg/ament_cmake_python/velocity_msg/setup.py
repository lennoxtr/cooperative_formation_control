from setuptools import find_packages
from setuptools import setup

setup(
    name='velocity_msg',
    version='0.0.0',
    packages=find_packages(
        include=('velocity_msg', 'velocity_msg.*')),
)
