from setuptools import find_packages
from setuptools import setup

setup(
    name='ur_moveit_config',
    version='2.8.1',
    packages=find_packages(
        include=('ur_moveit_config', 'ur_moveit_config.*')),
)
