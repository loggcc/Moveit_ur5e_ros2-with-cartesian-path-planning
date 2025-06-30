from setuptools import find_packages
from setuptools import setup

setup(
    name='ur_robot_driver',
    version='2.8.1',
    packages=find_packages(
        include=('ur_robot_driver', 'ur_robot_driver.*')),
)
