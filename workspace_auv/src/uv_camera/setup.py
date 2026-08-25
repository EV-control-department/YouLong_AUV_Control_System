import os

from setuptools import setup

package_name = 'uv_camera'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml', 'GO2RTC.md']),
    ('share/' + package_name + '/launch', ['launch/perception_launch.py']),
    ('share/' + package_name + '/config', ['config/front.npz', 'config/down.npz']),
]

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='origin',
    maintainer_email='origin@example.com',
    description='Camera + AI perception package for YouLong AUV (uv_sensor + uv_ai + position)',
    license='GPL-3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'uv_camera = uv_camera.composed:main',
            'position = uv_camera.position:main',
        ],
    },
)
