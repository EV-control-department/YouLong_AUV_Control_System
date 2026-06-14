from setuptools import find_packages
from setuptools import setup

setup(
    name='zit6_interfaces',
    version='0.0.1',
    packages=find_packages(
        include=('zit6_interfaces', 'zit6_interfaces.*')),
)
