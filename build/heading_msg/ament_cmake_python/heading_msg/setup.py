from setuptools import find_packages
from setuptools import setup

setup(
    name='heading_msg',
    version='0.0.0',
    packages=find_packages(
        include=('heading_msg', 'heading_msg.*')),
)
