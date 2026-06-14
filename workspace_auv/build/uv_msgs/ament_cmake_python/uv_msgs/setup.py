from setuptools import find_packages
from setuptools import setup

setup(
    name='uv_msgs',
    version='0.1.0',
    packages=find_packages(
        include=('uv_msgs', 'uv_msgs.*')),
)
