from setuptools import find_packages
from setuptools import setup

setup(
    name='ur_dashboard_msgs',
    version='2.8.1',
    packages=find_packages(
        include=('ur_dashboard_msgs', 'ur_dashboard_msgs.*')),
)
